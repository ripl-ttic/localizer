/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation;
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "localize3dcore.h"
#include "localize3d_messages.h"
#include "localize3d_motion.h"

#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>

#include <er_common/lcm_utils.h>

#include <interfaces/map3d_interface.h>
#include <laser_utils/laser_util.h>

//we should phase these out at some point
//#include <common3d/carmen3d_common.h>
#include <GL/gl.h>

//bot param stuff
#include <bot_param/param_client.h>
#include <lcmtypes/bot2_param.h>

#include <lcmtypes/er_lcmtypes.h>
#include <er_lcmtypes/lcm_channel_names.h>

/* global variables */
static lcm_t * lcm = NULL;
static bot_lcmgl_t * lcmgl = NULL;
static bot_lcmgl_t * lcmgl_particles = NULL;
static bot_lcmgl_t * lcmgl_laser = NULL;
static Laser_projector *laser_proj;
static double laser_max_range;

erlcm_multi_gridmap_t_subscription_t * sub;
erlcm_multi_gridmap_t_subscription_t * sub_1;

static erlcm_multi_gridmap_t *mmap_msg = NULL;
static carmen_map_p *global_carmen_mmap = NULL;
static erlcm_map_p *global_carmen3d_mmap = NULL;
static carmen3d_localize_map_p *mmap = NULL;
int converged_floor = -1;
int no_of_floors = 0;
int no_valid_floors = 0;
int *floor_map = NULL;
int current_floor_ind = 0;//-1;
int filter_initialized = 0;
int prev_mode = 0; //0 - global reinit 1-gaussian

static double last_pose[3] = {.0,.0,.0};
static int64_t last_pose_utime = .0;
//carmen3d_localize_map_t map;
carmen_map_placelist_t placelist;
//carmen3d_localize_particle_filter_p filter;
//carmen3d_localize_summary_t summary;

carmen3d_localize_particle_filter_p *mfilter = NULL;
carmen3d_localize_summary_t *msummary = NULL;

carmen3d_localize_param_t param;

static carmen_robot_laser_message carmen_msg;//front_laser;

/* publish a global position message */

int getFloorIndex(int floor_no){
  int floor_ind = -1;
  if(floor_map !=NULL){
    for(int i=0;i<no_valid_floors; i++){
      if(floor_map[i] == floor_no){ 
	floor_ind = i;
	return floor_ind;
      }    
    }
  }
  return floor_ind;
}

static void publish_slam_pose(int map_id, carmen_point_p particleLoc, double timestamp)
{
  static erlcm_slam_pose_t slam_pose;
  memset(&slam_pose, 0, sizeof(slam_pose));

  carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*particleLoc, global_carmen3d_mmap[map_id]);

  slam_pose.pos[0] = summary_mean_global.x;
  slam_pose.pos[1] = summary_mean_global.y;
  double rpy[3] = { 0, 0, summary_mean_global.theta };
  bot_roll_pitch_yaw_to_quat(rpy, slam_pose.orientation);
  slam_pose.utime = bot_timestamp_from_double(timestamp);
  erlcm_slam_pose_t_publish(lcm, SLAM_POSITION_CHANNEL, &slam_pose);
}

static void publish_localizer_pose(int map_id, carmen_point_p particleLoc, double timestamp)
{
  static bot_core_pose_t slam_pose;
  memset(&slam_pose, 0, sizeof(slam_pose));

  carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*particleLoc, global_carmen3d_mmap[map_id]);

  slam_pose.pos[0] = summary_mean_global.x;
  slam_pose.pos[1] = summary_mean_global.y;
  fprintf(stderr,"----- P : (%f,%f)\n", slam_pose.pos[0], slam_pose.pos[1]);
  double rpy[3] = { 0, 0, summary_mean_global.theta };

  bot_roll_pitch_yaw_to_quat(rpy, slam_pose.orientation);
  slam_pose.utime = bot_timestamp_from_double(timestamp);
  bot_core_pose_t_publish(lcm, "POSE", &slam_pose);

  last_pose[0] = slam_pose.pos[0];
  last_pose[1] = slam_pose.pos[1];
  last_pose[2] = summary_mean_global.theta;
  last_pose_utime = slam_pose.utime;
  //carmen3d_pose_t_publish(lcm, "LOCALIZER_POSE", &slam_pose);
}

int projectRangesToPoints(float * ranges, int numPoints, double thetaStart, double thetaStep, point2d_t * points,
    double maxRange)
{

  double validRangeStart = -1000;
  double validRangeEnd = 1000;
  double * aveValidRange = NULL;
  double * stddevValidRange = NULL;

  int count = 0;
  double aveRange = 0;
  double aveRangeSq = 0;

  double theta = thetaStart;
  for (int i = 0; i < numPoints; i++) {
    if (ranges[i] > .1 && ranges[i] < maxRange && theta > validRangeStart && theta < validRangeEnd) { //hokuyo driver seems to report maxRanges as .001 :-/
      //project to body centered coordinates
      points[count].x = ranges[i] * cos(theta);
      points[count].y = ranges[i] * sin(theta);
      count++;
      aveRange += ranges[i];
      aveRangeSq += ranges[i] * ranges[i];
    }

    theta += thetaStep;
  }

  aveRange /= (double) count;
  aveRangeSq /= (double) count;

  if (aveValidRange != NULL)
    *aveValidRange = aveRange;
  if (stddevValidRange != NULL)
    *stddevValidRange = sqrt(aveRangeSq - aveRange * aveRange);

  return count;
}

void publish_speech_msg(char* property)
{
  erlcm_speech_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.cmd_type = "LOCALIZER";
  msg.cmd_property = property;
  erlcm_speech_cmd_t_publish(lcm, "MULTI_LOCALIZER", &msg);
}

void publish_floor_speech_msg(char* property)
{
  erlcm_speech_cmd_t msg;
  msg.utime = bot_timestamp_now();
  msg.cmd_type = "FLOOR_AUTO_CHANGE";
  msg.cmd_property = property;
  erlcm_speech_cmd_t_publish(lcm, "MULTI_LOCALIZER", &msg);
}

//drawing front robot laser message 
static void draw_laser_lcmgl_multi(int map_id, carmen_point_p meanLoc, carmen_point_p maxParticleLoc, carmen_robot_laser_message *flaser)
{
  point2d_t * points = (point2d_t *) calloc(flaser->num_readings, sizeof(point2d_t));
  int numValidPoints = projectRangesToPoints(flaser->range, flaser->num_readings, flaser->config.start_angle,
					     flaser->config.fov / flaser->num_readings, points, flaser->config.maximum_range);

  bot_lcmgl_point_size(lcmgl_laser, 3);
  bot_lcmgl_begin(lcmgl_laser, GL_POINTS);

  carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*meanLoc, global_carmen3d_mmap[map_id]);
  carmen_point_t max_part_global = carmen3d_map_map_to_global_coordinates(*maxParticleLoc, global_carmen3d_mmap[map_id]);

  BotTrans mean_tranny;
  memset(&mean_tranny, 0, sizeof(mean_tranny));
  //change the value to be the point of the front laser

  double fl_offset = param.front_laser_offset;

  mean_tranny.trans_vec[0] = summary_mean_global.x + fl_offset*cos(summary_mean_global.theta);
  mean_tranny.trans_vec[1] = summary_mean_global.y + fl_offset*sin(summary_mean_global.theta);
  double rpy[3] = { 0, 0, summary_mean_global.theta };
  bot_roll_pitch_yaw_to_quat(rpy, mean_tranny.rot_quat);

  BotTrans max_tranny;
  memset(&max_tranny, 0, sizeof(max_tranny));
  max_tranny.trans_vec[0] = max_part_global.x;
  max_tranny.trans_vec[1] = max_part_global.y;
  rpy[2] = max_part_global.theta;
  bot_roll_pitch_yaw_to_quat(rpy, max_tranny.rot_quat);

  for (int i = 0; i < numValidPoints; i++) {
    double p[3] = { points[i].x, points[i].y, 0 };
    double rp[3];
    bot_lcmgl_color3f(lcmgl_laser, 1, 0, 0);
    bot_trans_apply_vec(&mean_tranny, p, rp);
    bot_lcmgl_vertex3f(lcmgl_laser, rp[0], rp[1], rp[2]);
  }
  bot_lcmgl_end(lcmgl_laser);
  free(points);
  bot_lcmgl_switch_buffer(lcmgl_laser);
  
}

//draw all the filter particles 
/* publish a particle message */
static void draw_particles_lcmgl_multi()
{
  
  for(int map_id=0;map_id<no_valid_floors; map_id++){
    carmen3d_localize_particle_filter_p filter =  mfilter[map_id];
    carmen3d_localize_summary_p summary = &msummary[map_id];
    //draw particles
    bot_lcmgl_color3f(lcmgl_particles, 0, 1, 1);
    bot_lcmgl_point_size(lcmgl_particles, 2);
    bot_lcmgl_begin(lcmgl_particles, GL_POINTS);
    for (int i = 0; i < filter->param->num_particles; i++) {
      carmen_point_t particle_map = { filter->particles[i].x, filter->particles[i].y, filter->particles[i].theta };
      carmen_point_t particle_global = carmen3d_map_map_to_global_coordinates(particle_map, global_carmen3d_mmap[map_id]);
      bot_lcmgl_vertex3f(lcmgl_particles, particle_global.x, particle_global.y, 0);
    }
    bot_lcmgl_end(lcmgl_particles);
    fprintf(stderr, "P");
  }
  bot_lcmgl_switch_buffer(lcmgl_particles);

  for(int map_id=0;map_id<no_valid_floors; map_id++){
    carmen3d_localize_particle_filter_p filter =  mfilter[map_id];
    carmen3d_localize_summary_p summary = &msummary[map_id];
    //draw summary
    //mean
    carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(summary->mean, global_carmen3d_mmap[map_id]);
    bot_lcmgl_color3f(lcmgl, 0, 1, 0);
    bot_lcmgl_point_size(lcmgl, 15);
    bot_lcmgl_begin(lcmgl, GL_POINTS);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_end(lcmgl);

    //heading
    double headingLength = .8;
    bot_lcmgl_color3f(lcmgl, 1, 1, 0);
    bot_lcmgl_point_size(lcmgl, 8);
    bot_lcmgl_line_width(lcmgl, 8);
    bot_lcmgl_begin(lcmgl, GL_LINES);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x + headingLength * cos(summary_mean_global.theta),
		       summary_mean_global.y + headingLength * sin(summary_mean_global.theta), 0);
    bot_lcmgl_end(lcmgl);

    //cov
    double cov_factor = 10;
    bot_lcmgl_color3f(lcmgl, 1, 0, 0);
    bot_lcmgl_point_size(lcmgl, 5);
    bot_lcmgl_line_width(lcmgl, 5);
    bot_lcmgl_begin(lcmgl, GL_LINES);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x + cov_factor * summary->std.x, summary_mean_global.y, 0);
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_begin(lcmgl, GL_LINES);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y + cov_factor * summary->std.y, 0);
    bot_lcmgl_end(lcmgl);

    bot_lcmgl_begin(lcmgl, GL_LINES);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, cov_factor * summary->std.theta);
    bot_lcmgl_end(lcmgl);
  }
  bot_lcmgl_switch_buffer(lcmgl);
}


/* publish a particle message */
static void draw_particles_lcmgl_single(int map_id, carmen3d_localize_particle_filter_p filter, carmen3d_localize_summary_p summary)
{

  //draw particles
  bot_lcmgl_color3f(lcmgl_particles, 0, 1, 1);
  bot_lcmgl_point_size(lcmgl_particles, 2);
  bot_lcmgl_begin(lcmgl_particles, GL_POINTS);
  for (int i = 0; i < filter->param->num_particles; i++) {
    carmen_point_t particle_map = { filter->particles[i].x, filter->particles[i].y, filter->particles[i].theta };
    carmen_point_t particle_global = carmen3d_map_map_to_global_coordinates(particle_map, global_carmen3d_mmap[map_id]);
    bot_lcmgl_vertex3f(lcmgl_particles, particle_global.x, particle_global.y, 0);
  }
  bot_lcmgl_end(lcmgl_particles);
  fprintf(stderr, "P");
  bot_lcmgl_switch_buffer(lcmgl_particles);

  //draw summary
  //mean
  carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(summary->mean, global_carmen3d_mmap[map_id]);
  bot_lcmgl_color3f(lcmgl, 0, 1, 0);
  bot_lcmgl_point_size(lcmgl, 15);
  bot_lcmgl_begin(lcmgl, GL_POINTS);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
  bot_lcmgl_end(lcmgl);

  //heading
  double headingLength = .8;
  bot_lcmgl_color3f(lcmgl, 1, 1, 0);
  bot_lcmgl_point_size(lcmgl, 8);
  bot_lcmgl_line_width(lcmgl, 8);
  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x + headingLength * cos(summary_mean_global.theta),
      summary_mean_global.y + headingLength * sin(summary_mean_global.theta), 0);
  bot_lcmgl_end(lcmgl);

  //cov
  double cov_factor = 10;
  bot_lcmgl_color3f(lcmgl, 1, 0, 0);
  bot_lcmgl_point_size(lcmgl, 5);
  bot_lcmgl_line_width(lcmgl, 5);
  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x + cov_factor * summary->std.x, summary_mean_global.y, 0);
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y + cov_factor * summary->std.y, 0);
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_begin(lcmgl, GL_LINES);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
  bot_lcmgl_vertex3f(lcmgl, summary_mean_global.x, summary_mean_global.y, cov_factor * summary->std.theta);
  bot_lcmgl_end(lcmgl);

  bot_lcmgl_switch_buffer(lcmgl);

}

/* process initialization messages */

void carmen_localize_initialize_handler(carmen_localize_initialize_message *initialize_msg)
{
  fprintf(stderr,"Initializing filter\n");
  if (initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN){
    prev_mode  = 1;
    //the localize_initialize should have the current floor - or we should have the current floor ind
    //the floor here is ambiguous if we depend on the floor change to change the floor
    if(current_floor_ind >=0 && current_floor_ind < no_valid_floors){
      carmen3d_localize_initialize_particles_gaussians(mfilter[current_floor_ind], initialize_msg->num_modes, 
						     initialize_msg->mean,
						     initialize_msg->std);
    }
    else{
      fprintf(stderr,"No current floor - not able to initialize\n");
    }
  }
  else if (initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM) {
    prev_mode = 0;
    //carmen3d_localize_initialize_particles_uniform(filter, &front_laser, &map);
    //this should be multiple filters initialized on each floor
    //carmen3d_localize_initialize_particles_uniform(filter, &carmen_msg, &map);
    //carmen3d_localize_initialize_particles_uniform(filter, &carmen_msg, mmap[0]);
    converged_floor = -1;
    for(int i=0;i<no_valid_floors; i++){
      carmen3d_localize_initialize_particles_uniform(mfilter[i], &carmen_msg, mmap[i]);
    }    
  }
  fprintf(stderr,"Done Init\n");
  //draw_particles_lcmgl(filter, &summary);
}

void floor_change_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)), 
			const erlcm_floor_change_msg_t * msg,
			void * user  __attribute__((unused)))
{
  fprintf(stderr,"Floor Change Heard : %d\n", msg->floor_no);
  if(floor_map == NULL){
    fprintf(stderr,"No maps/floors received\n");
    return;
  }
  
  if(current_floor_ind == getFloorIndex(msg->floor_no)){
    fprintf(stderr,"No actual change in floor C:%d New: %d\n", floor_map[current_floor_ind],msg->floor_no );
    return; //no floor change event has occured
  }
  else if(getFloorIndex(msg->floor_no) >=0){
    fprintf(stderr,"Valid Floor received \n");
    current_floor_ind = getFloorIndex(msg->floor_no);
    converged_floor = current_floor_ind;
    fprintf(stderr,"New Floor :%d\n", floor_map[current_floor_ind]);
    fprintf(stderr,"Changing the floor filters\n");
    if(last_pose_utime > 0){//for now use the last pose - prob should check if we have a good pose
      carmen_localize_initialize_message initialize_msg;
      memset(&initialize_msg, 0, sizeof(initialize_msg));
      initialize_msg.host = "self";
      initialize_msg.timestamp = carmen_get_time();

      initialize_msg.distribution = CARMEN_INITIALIZE_GAUSSIAN;
      initialize_msg.num_modes = 1;
      carmen_point_t mean = {last_pose[0],last_pose[1], last_pose[2] };
      //this should have a floor attched - or use the current floor 
      int floor_ind = current_floor_ind;
      carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, global_carmen3d_mmap[floor_ind]);

      carmen_point_t std = {
	sqrt(0.01),
	sqrt(0.01),
	sqrt(carmen_degrees_to_radians(0.1))
      };
      initialize_msg.mean = &mean_map;
      initialize_msg.std = &std;

      carmen_localize_initialize_handler(&initialize_msg);
    }
    //
    return;
  }
  else{
    fprintf(stderr,"Unknown Floor received - Doing nothing\n");
    return;
  }
}


void carmen_localize_initialize_placename_handler(carmen_localize_initialize_placename_message *init_place)
{
  //this place name should have a floor attached to it
  //carmen3d_localize_initialize_particles_placename(filter, &placelist, init_place->placename);
  //draw_particles_lcmgl(filter, &summary);
}

void robot_frontlaser_handler(carmen_robot_laser_message *flaser)
{
  fprintf(stderr, "F");
  static double last_pose[2] = {.0,.0};
  double dist_travelled;
  dist_travelled = hypot(flaser->robot_pose.x - last_pose[0],flaser->robot_pose.y - last_pose[1]);
  last_pose[0] = flaser->robot_pose.x;
  last_pose[1] = flaser->robot_pose.y;
  
  if(dist_travelled==0){
    fprintf(stdout,"-");
    return;
  }

  static int64_t last_floor_change_sent = 0;
  if(converged_floor <0){//no converged floor
    fprintf(stderr,"Floor Unknown - running multi-floor localization\n");
    double *avg_weight = (double *) malloc(no_valid_floors*sizeof(double));
    for(int i=0;i<no_valid_floors; i++){
      carmen3d_localize_run(mfilter[i], mmap[i], flaser, mfilter[i]->param->front_laser_offset, 0);
      if (mfilter[i]->initialized) {
	carmen3d_localize_summarize(mfilter[i], &msummary[i], mmap[i], flaser->num_readings, flaser->range,
				  mfilter[i]->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);
	avg_weight[i] = 0;     
            
	for (int k = 0; k < mfilter[i]->param->num_particles; k++) {
	  avg_weight[i] += mfilter[i]->particles->weight;
	}
	fprintf(stderr,"Total weight %d => %f\n", floor_map[i], avg_weight[i]);
	avg_weight[i] /= mfilter[i]->param->num_particles;      
      }
      else{
	avg_weight[i] = -1000;
      }
    }
  
    int best_ind = -1;
    double best_prob = -10.0;//-5.0;
    int filters_converged = 1;
    int filters_initialized = 0;

    for(int i=0;i<no_valid_floors; i++){    
      if (mfilter[i]->initialized) {
	filters_initialized = 1;
	fprintf(stderr,"==== Floor : %d => Avg Weight : %f Converged : %d\n", 
		floor_map[i], avg_weight[i], msummary[i].converged);
	if(msummary[i].converged ==0){//there is an unconverged filter 
	  filters_converged = 0;
	}
	if((avg_weight[i] > best_prob && msummary[i].converged) && avg_weight[i] !=0.0){
	  best_ind = i;
	  best_prob = avg_weight[i];	  
	}
      }    
    }
    if(best_ind >=0){
      double maxWeight = -1e9;
      carmen_point_t maxParticle;
      memset(&maxParticle, 0, sizeof(maxParticle));
      for (int k = 0; k < mfilter[best_ind]->param->num_particles; k++) {
	if (mfilter[best_ind]->particles->weight > maxWeight) {
	  maxWeight = mfilter[best_ind]->particles->weight;
	  maxParticle.x = mfilter[best_ind]->particles->x;
	  maxParticle.y = mfilter[best_ind]->particles->y;
	  maxParticle.theta = mfilter[best_ind]->particles->theta;
	}
      }
      //do a floor switch 
      erlcm_floor_change_msg_t f_msg;
      f_msg.utime = bot_timestamp_now();
      last_floor_change_sent = f_msg.utime;
      f_msg.floor_no = floor_map[best_ind];
      erlcm_floor_change_msg_t_publish(lcm, "FLOOR_STAUS", &f_msg);
      current_floor_ind = best_ind;
      publish_speech_msg("FOUND");

      char buf[10];
      sprintf(buf,"%d", floor_map[best_ind]);
      publish_floor_speech_msg(buf);
      //when we send this floor change - this causes the map_server to reinitlialze the localizer 

      converged_floor = best_ind;
      draw_laser_lcmgl_multi(best_ind,&msummary[best_ind].mean, &maxParticle, flaser);
      fprintf(stderr,"Found a converged probable floor => %d\n", floor_map[best_ind]);
      publish_slam_pose(best_ind,&msummary[best_ind].mean,flaser->timestamp);
      publish_localizer_pose(best_ind,&msummary[best_ind].mean,flaser->timestamp);
    }
    else{
      if(filters_converged && filters_initialized){//filters have all converged but we do not have a good candidate 
	//reinitializing
	filter_initialized = 0;
	publish_speech_msg("RESTART");
      }
    }
    draw_particles_lcmgl_multi();
  }
  else{
    fprintf(stderr,"Converged Floor - doing single floor localization\n");
    carmen3d_localize_run(mfilter[converged_floor], mmap[converged_floor], flaser, mfilter[converged_floor]->param->front_laser_offset, 0);
    double avg_weight = 0;     
    if (mfilter[converged_floor]->initialized) {
      carmen3d_localize_summarize(mfilter[converged_floor], &msummary[converged_floor], mmap[converged_floor], flaser->num_readings, flaser->range,
				mfilter[converged_floor]->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);

            
      for (int k = 0; k < mfilter[converged_floor]->param->num_particles; k++) {
	avg_weight += mfilter[converged_floor]->particles->weight;
      }
      avg_weight /= mfilter[converged_floor]->param->num_particles;      
    }
    
    if(avg_weight != 0.0){
      static double fast_avg = .0;  
      static double slow_avg = .0;
    
      fast_avg += 0.5 * (avg_weight - fast_avg);
      slow_avg += 0.05 * (avg_weight - slow_avg);

      fprintf(stderr,"==== Current Floor [%d] Stats => Avg Weight : %f Converged : %d\n", 
	      floor_map[converged_floor], avg_weight, msummary[converged_floor].converged);
      fprintf(stderr,"Current Avg : %f , Fast Avg : %f, Slow Avg : %f\n", avg_weight, 
	      fast_avg, slow_avg);
    
      //doesn't work so well 
      /*if((fast_avg - slow_avg  > 4.0 && fast_avg < -35.0) || msummary[converged_floor].converged == 0){
	fprintf(stderr,"Reinitializing Filters\n");
	filter_initialized = 0;
	}*/
    }
    if(avg_weight < -45.0|| msummary[converged_floor].converged == 0){
      fprintf(stderr,"Reinitializing Filters\n");
      filter_initialized = 0;
      publish_speech_msg("LOST");
    }

    //might add code to switch back to global localization - if the prob drops 

    double maxWeight = -1e9;
    carmen_point_t maxParticle;
    memset(&maxParticle, 0, sizeof(maxParticle));
    for (int k = 0; k < mfilter[converged_floor]->param->num_particles; k++) {
      if (mfilter[converged_floor]->particles->weight > maxWeight) {
	maxWeight = mfilter[converged_floor]->particles->weight;
	maxParticle.x = mfilter[converged_floor]->particles->x;
	maxParticle.y = mfilter[converged_floor]->particles->y;
	maxParticle.theta = mfilter[converged_floor]->particles->theta;
      }
    }    
    publish_slam_pose(converged_floor,&msummary[converged_floor].mean,flaser->timestamp);
    publish_localizer_pose(converged_floor,&msummary[converged_floor].mean,flaser->timestamp);

    draw_laser_lcmgl_multi(converged_floor,&msummary[converged_floor].mean, &maxParticle, flaser);
    draw_particles_lcmgl_single(converged_floor, mfilter[converged_floor], &msummary[converged_floor]);
    //draw laser and particles 
  }

  for(int i=0;i<no_valid_floors; i++){
    if(filter_initialized ==0){
      converged_floor = -1;
      fprintf(stderr,"Initializing Filter\n");
      carmen_localize_initialize_message initialize_msg;
      memset(&initialize_msg, 0, sizeof(initialize_msg));
      initialize_msg.host = "self";
      initialize_msg.timestamp = carmen_get_time();

      initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

      initialize_msg.num_modes = 1;
      carmen_point_t mean = { 2, 4, M_PI / 4 };

      carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, global_carmen3d_mmap[0]);
      fprintf(stderr,"mean : (%f,%f)\n", mean_map.x, mean_map.y);

      //  carmen_point_t std={1,1,M_PI/2};
      carmen_point_t std = { .1, .1, .1 };
      initialize_msg.mean = &mean_map;
      initialize_msg.std = &std;
      carmen_localize_initialize_handler(&initialize_msg);
      filter_initialized = 1;
    }
  }
  //fprintf(stderr,"Done with laser message\n");
}

/*void robot_frontlaser_handler_1(carmen_robot_laser_message *flaser)
{
  
    
  fprintf(stderr, "F");
  carmen3d_localize_run(filter, &map, flaser, filter->param->front_laser_offset, 0);
  if (filter->initialized) {
    //carmen3d_localize_summarize(filter, &summary, &map, flaser->num_readings, flaser->range,
        //filter->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);
    
    for(int i=0;i<no_valid_floors; i++){
      carmen3d_localize_summarize(mfilter[i], &msummary[i], mmap[i], flaser->num_readings, flaser->range,
        mfilter[i]->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);

    }
    

    //if (summary.converged) {
    for(int i=0;i<no_valid_floors; i++){
      if (msummary[i].converged) {
	publish_slam_pose(&msummary[i].mean,flaser->timestamp);
	publish_localizer_pose(&msummary[i].mean,flaser->timestamp);
      }
    }
    draw_particles_lcmgl(filter, &summary);

    draw_laser_lcmgl(&summary.mean, &maxParticle, flaser);
  }
  
  if(filter_initialized ==0){
    fprintf(stderr,"Initializing Filter\n");
    carmen_localize_initialize_message initialize_msg;
    memset(&initialize_msg, 0, sizeof(initialize_msg));
    initialize_msg.host = "self";
    initialize_msg.timestamp = carmen_get_time();

    initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

    initialize_msg.num_modes = 1;
    carmen_point_t mean = { 2, 4, M_PI / 4 };

    carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, global_carmen3d_mmap[0]);

    //  carmen_point_t std={1,1,M_PI/2};
    carmen_point_t std = { .1, .1, .1 };
    initialize_msg.mean = &mean_map;
    initialize_msg.std = &std;
    carmen_localize_initialize_handler(&initialize_msg);
    filter_initialized = 1;
  }
}
*/


void lcm_robot_laser_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char *channel __attribute__((unused)), const erlcm_robot_laser_t *msg,
    void *user)
{
  static erlcm_robot_laser_t *static_msg = NULL;
  if (static_msg != NULL)
    erlcm_robot_laser_t_destroy(static_msg);
  static_msg = erlcm_robot_laser_t_copy(msg);

  //set invalid regions to maxrange
  for (int i=0;i<static_msg->laser.nranges;i++){
    if (static_msg->laser.ranges[i]< laser_proj->min_range ||
        static_msg->laser.ranges[i]> laser_proj->max_range ||
        i<laser_proj->surroundRegion[0] ||
        i>laser_proj->surroundRegion[1]){
      static_msg->laser.ranges[i]=laser_proj->max_range+1;
    }
  }

  //static carmen_robot_laser_message carmen_msg;
  memset(&carmen_msg, 0, sizeof(carmen_msg));
  carmen_msg.laser_pose.x = static_msg->pose.pos[0];
  carmen_msg.laser_pose.y = static_msg->pose.pos[1];
  double rpy[3];
  bot_quat_to_roll_pitch_yaw(static_msg->pose.orientation, rpy);
  carmen_msg.laser_pose.theta = rpy[2];
  carmen_msg.robot_pose = carmen_msg.laser_pose;

  carmen_msg.config.accuracy = .1;
  carmen_msg.config.laser_type = HOKUYO_UTM;
  carmen_msg.config.remission_mode = REMISSION_NONE;
  carmen_msg.config.angular_resolution = static_msg->laser.radstep;
  carmen_msg.config.fov = static_msg->laser.nranges * static_msg->laser.radstep;
  carmen_msg.config.maximum_range = laser_proj->max_range;
  carmen_msg.config.start_angle = static_msg->laser.rad0;

  carmen_msg.id = 0;
  carmen_msg.num_readings = static_msg->laser.nranges;
  carmen_msg.range = static_msg->laser.ranges;
  carmen_msg.num_remissions = static_msg->laser.nintensities;
  carmen_msg.remission = static_msg->laser.intensities;
  carmen_msg.timestamp = bot_timestamp_to_double(static_msg->utime);
  carmen_msg.host = (char *) "lcm";

  //TODO:there are some other fields like tooclose and turn_axis etc that I don't think are used...
  robot_frontlaser_handler(&carmen_msg);
}

//this reinitialize doesnt have a floor attached - ideally we should use the new message with the floor also 
void lcm_localize_reinitialize_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char *channel __attribute__((unused)), const erlcm_localize_reinitialize_cmd_t *msg,
    void *user)
{
  if(mmap_msg ==NULL){
    fprintf(stderr,"No map received - unable to localize\n");
    return;
  }
  printf("\ngot new reinitialize msg (%f, %f, %f) (%f, %f, %f)\n\n",
      msg->mean[0], msg->mean[1], msg->mean[2],
      sqrt(msg->variance[0]), sqrt(msg->variance[1]), sqrt(msg->variance[2]));

  carmen_localize_initialize_message initialize_msg;
  memset(&initialize_msg, 0, sizeof(initialize_msg));
  initialize_msg.host = "self";
  initialize_msg.timestamp = carmen_get_time();

  initialize_msg.distribution = CARMEN_INITIALIZE_GAUSSIAN;
  initialize_msg.num_modes = 1;
  carmen_point_t mean = { msg->mean[0], msg->mean[1], msg->mean[2] };
  //this should have a floor attched - or use the current floor 
  int floor_ind = current_floor_ind;
  carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, global_carmen3d_mmap[floor_ind]);

  carmen_point_t std = {
    sqrt(msg->variance[0]),
    sqrt(msg->variance[1]),
    sqrt(msg->variance[2])
  };
  initialize_msg.mean = &mean_map;
  initialize_msg.std = &std;

  carmen_localize_initialize_handler(&initialize_msg);
}

void read_parameters_from_conf(int argc, char **argv, carmen3d_localize_param_p param, lcm_t *lcm, BotParam * b_param )
{


  double integrate_angle_deg;
  integrate_angle_deg = 1.0;

   
  param->front_laser_offset = bot_param_get_double_or_fail(b_param, "robot.frontlaser_offset");
  fprintf(stderr, "%f\n", param->front_laser_offset);
  param->rear_laser_offset = bot_param_get_double_or_fail(b_param, "robot.rearlaser_offset");
  param->front_laser_side_offset =  bot_param_get_double_or_fail(b_param, "robot.frontlaser_side_offset");
  param->rear_laser_side_offset = bot_param_get_double_or_fail(b_param,"robot.rearlaser_side_offset");
  param->front_laser_angle_offset = bot_param_get_double_or_fail(b_param,"robot.frontlaser_angular_offset");
  param->rear_laser_angle_offset = bot_param_get_double_or_fail(b_param,"robot.rearlaser_angular_offset");  
  param->use_rear_laser = bot_param_get_boolean_or_fail(b_param,"localizer.use_rear_laser");
  param->num_particles =bot_param_get_int_or_fail(b_param,"localizer.num_particles");
  param->max_range = bot_param_get_double_or_fail(b_param,"localizer.laser_max_range");
  param->min_wall_prob = bot_param_get_double_or_fail(b_param,"localizer.min_wall_prob");
  param->outlier_fraction = bot_param_get_double_or_fail(b_param,"localizer.outlier_fraction");
  param->update_distance = bot_param_get_double_or_fail(b_param,"localizer.update_distance");
  integrate_angle_deg = bot_param_get_double_or_fail(b_param,"localizer.integrate_angle_deg");
  param->do_scanmatching = bot_param_get_boolean_or_fail(b_param,"localizer.do_scanmatching");
  param->constrain_to_map = bot_param_get_boolean_or_fail(b_param,"localizer.constrain_to_map");

#ifdef OLD_MOTION_MODEL
  param->odom_a1 = bot_param_get_double_or_fail(b_param,"localizer.odom_a1");
  param->odom_a2 = bot_param_get_double_or_fail(b_param,"localizer.odom_a2");
  param->odom_a3 = bot_param_get_double_or_fail(b_param,"localizer.odom_a3");
  param->odom_a4 = bot_param_get_double_or_fail(b_param,"localizer.odom_a4");  
#endif

  param->occupied_prob = bot_param_get_double_or_fail(b_param,"localizer.occupied_prob");
  param->lmap_std = bot_param_get_double_or_fail(b_param,"localizer.lmap_std");
  param->global_lmap_std = bot_param_get_double_or_fail(b_param,"localizer.global_lmap_std");
  param->global_evidence_weight = bot_param_get_double_or_fail(b_param,"localizer.global_evidence_weight");
  param->global_distance_threshold = bot_param_get_double_or_fail(b_param,"localizer.global_distance_threshold");
  param->global_test_samples = bot_param_get_int_or_fail(b_param,"localizer.global_test_samples");
  param->use_sensor = bot_param_get_boolean_or_fail(b_param,"localizer.use_sensor");
  param->tracking_beam_minlikelihood = bot_param_get_double_or_fail(b_param,"localizer.tracking_beam_minlikelihood");
  param->global_beam_minlikelihood = bot_param_get_double_or_fail(b_param,"localizer.global_beam_minlikelihood");
  
  param->integrate_angle = carmen_degrees_to_radians(integrate_angle_deg);
  
}

static void multi_gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const erlcm_multi_gridmap_t *msg, void *user)
{  
  //static carmen3d_multi_gridmap_t *static_msg = NULL;

  if(mmap_msg != NULL){
    erlcm_multi_gridmap_t_destroy(mmap_msg);
  }
  mmap_msg = erlcm_multi_gridmap_t_copy(msg);


  fprintf(stderr,"Multi-Map recieved Floors: %d\n", mmap_msg->no_floors);

  //add code to free the old maps 


  //reallocate for the new maps
  
  no_of_floors = mmap_msg->no_floors;
  floor_map = (int *) realloc(floor_map,sizeof(int)*mmap_msg->no_floors);
  global_carmen_mmap = (carmen_map_p *) realloc(global_carmen_mmap, no_of_floors * sizeof(carmen_map_p));
  global_carmen3d_mmap = (erlcm_map_p *)realloc(global_carmen3d_mmap, no_of_floors * sizeof(erlcm_map_p));
  mmap = (carmen3d_localize_map_p *)realloc(mmap, no_of_floors * sizeof(carmen3d_localize_map_p));
  mfilter = (carmen3d_localize_particle_filter_p *) realloc(mfilter, sizeof(carmen3d_localize_particle_filter_p) * no_of_floors);
  msummary = (carmen3d_localize_summary_t *) realloc(msummary, sizeof(carmen3d_localize_summary_t) * no_of_floors);
  for(int map_id=0;map_id<mmap_msg->no_floors; map_id++){
    mmap[map_id] = (carmen3d_localize_map_p) malloc(sizeof(carmen3d_localize_map_t));
    memset(mmap[map_id], 0, sizeof(carmen3d_localize_map_t));
    global_carmen3d_mmap[map_id] = (erlcm_map_p) malloc(sizeof(erlcm_map_t)); 
    //erlcm_map_t sub_map;
    memset(global_carmen3d_mmap[map_id], 0,sizeof(erlcm_map_t));
    floor_map[map_id] = mmap_msg->maps[map_id].floor_no;
    //if(current_floor_ind == map_id){
    //current_floor_no = static_msg->maps[map_id].floor_no;
    //}
    fprintf(stderr,"Map center : %f,%f\n",mmap_msg->maps[map_id].gridmap.center.x, mmap_msg->maps[map_id].gridmap.center.y);

    carmen3d_map_uncompress_lcm_map(global_carmen3d_mmap[map_id], &mmap_msg->maps[map_id].gridmap); //maybe here is the alloc

    //do we need to alloc space for this pointer also?? - dont think so 
    global_carmen_mmap[map_id] = (carmen_map_p) calloc(1, sizeof(carmen_map_t));
    global_carmen_mmap[map_id] = carmen3d_map_map3d_map_copy(global_carmen3d_mmap[map_id]);
    

    //if(map_id <2){
    no_valid_floors++;
    fprintf(stderr,"Creating likelihood maps...: %d ", map_id);
    //reallocate the filters
    mfilter[map_id] = carmen3d_localize_particle_filter_new(&param);

    //carmen3d_to_localize_map(&global_carmen_map[map_id], mmap[map_id], &param);
    //carmen3d_map_uncompress_lcm_map(global_carmen3d_map, &mmap_msg->maps[map_id].gridmap);
    //global_carmen_map = carmen3d_map_map3d_map_copy(global_carmen3d_map);
    //global_carmen_map = carmen3d_map_map3d_map_copy(global_carmen3d_mmap[map_id]);
    //carmen3d_to_localize_map(global_carmen_map, &map, &param);
    carmen3d_to_localize_map(global_carmen_mmap[map_id], mmap[map_id], &param);
    fprintf(stderr,"Done\n");
    //}
  }

  //unsubscribe 

  erlcm_multi_gridmap_t_unsubscribe(lcm, sub);
  erlcm_multi_gridmap_t_unsubscribe(lcm, sub_1);
}

//this method is not useful anymore as we will be dealing with the multi-floor map messages even if we 
//only travel a single floor 
/*
static void gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const erlcm_gridmap_t *msg, void *user)
{
  fprintf(stderr,"Gridmap Received\n");
  if (global_map_msg != NULL) {
    erlcm_gridmap_t_destroy(global_map_msg);
  }
  global_map_msg = erlcm_gridmap_t_copy(msg);


  fprintf(stderr,"New map recieved\n");

  carmen3d_map_uncompress_lcm_map(global_carmen3d_map, global_map_msg);

  if(global_carmen_map !=NULL){
    free(global_carmen_map);
  }

  global_carmen_map = carmen3d_map_map3d_map_copy(global_carmen3d_map);
  
  // create a localize map 
  carmen_warn("Creating likelihood maps... ");
  carmen3d_to_localize_map(global_carmen_map, &map, &param);
  fprintf(stderr,"Done\n");
  //this just coppies the message - doesnt do anything else with it - need to actually have the localize listen to and switch maps 
  //on the fly
}
*/

void create_localize_mmap()
{
// subscripe to map, and wait for it to come in...
  erlcm_map_request_msg_t msg;
  msg.utime =  carmen_get_time()*1e6;
  msg.requesting_prog = "LOCALIZE_MULTI";

  erlcm_map_request_msg_t_publish(lcm,"MMAP_REQUEST_CHANNEL",&msg);
  
  carmen_warn("Waiting for a map");
  while (mmap_msg == NULL) {
    lcm_sleep(lcm, .25);
    carmen_warn(".");
  }
  carmen_warn("Got a map\n");
}

/* create localize specific maps */

/*void create_localize_map(carmen3d_localize_map_t *map, carmen3d_localize_param_p param)
{
  // subscripe to map, and wait for it to come in...
  erlcm_map_request_msg_t msg;
  msg.utime =  carmen_get_time()*1e6;
  msg.requesting_prog = "LOCALIZE";

  erlcm_map_request_msg_t_publish(lcm,"MAP_REQUEST_CHANNEL",&msg);

  erlcm_gridmap_t_subscription_t * sub = erlcm_gridmap_t_subscribe(lcm, "MAP_SERVER", gridmap_handler,
      NULL);

  erlcm_gridmap_t_subscription_t * sub_1 = erlcm_gridmap_t_subscribe(lcm, "FINAL_SLAM", gridmap_handler,
      NULL);
  carmen_warn("Waiting for a map");
  while (global_map_msg == NULL) {
    lcm_sleep(lcm, .25);
    carmen_warn(".");
  }
  carmen_warn("Got a map\n");

  //copies and uncompress the carmen3d map to global map
  carmen3d_map_uncompress_lcm_map(global_carmen3d_map, global_map_msg);
  global_carmen_map = carmen3d_map_map3d_map_copy(global_carmen3d_map);

  // create a localize map 
  carmen_warn("Creating likelihood maps... ");
  carmen3d_to_localize_map(global_carmen_map, map, param);

  carmen_warn("done.\n");
  }*/

void tagging_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)), 
		    const erlcm_tagged_node_t * msg,
		    void * user  __attribute__((unused)))
{
  char* type = msg->type;
  char* name = msg->label;

  if(strcmp(type,"mode")==0){
    //mode has changed - we should start localizing if we are in navigation mode
    if(!strcmp(name,"navigation")){
      fprintf(stderr,"Starting Localization\n");
    }
  }
}

void shutdown_localize(int x)
{
  if (x == SIGINT) {
    carmen_verbose("Disconnecting from IPC network.\n");
    exit(1);
  }
}

int main(int argc, char **argv)
{  /* initialize carmen */
  
  //****** remove the ipc stuff
  
  carmen_randomize(&argc, &argv);
  fprintf(stderr,"Done\n");
  //carmen_ipc_initialize(argc, argv);
  //carmen_param_check_version(argv[0]);

  lcm = bot_lcm_get_global(NULL);//globals_get_lcm();
  lcmgl = bot_lcmgl_init(lcm, "localize3d");//globals_get_lcmgl("localize3d", 1);
  lcmgl_particles = bot_lcmgl_init(lcm,"localize3d_particles");//globals_get_lcmgl("localize3d_particles", 1);
  lcmgl_laser = bot_lcmgl_init(lcm,"localize3d_laser"); //globals_get_lcmgl("localize3d_laser", 1);

  BotParam * b_param = bot_param_new_from_server(lcm, 1);

 
  /*--------------Multi floor maps------------*/
  global_carmen_mmap = NULL;//(carmen_map_p) calloc(1, sizeof(carmen_map_t));
  //carmen_test_alloc(global_carmen_map);

  global_carmen3d_mmap = NULL;//(erlcm_map_p) calloc(1, sizeof(erlcm_map_t));
  //carmen_test_alloc(global_carmen3d_map);

  laser_proj = laser_projector_new(b_param, NULL, "SKIRT_FRONT",0);

  /* Setup exit handler */
  signal(SIGINT, shutdown_localize);

  /* Initialize all the relevant parameters */
  //read_parameters(argc, argv, &param);
  //read_parameters_from_conf(argc, argv, &param, lcm);
  read_parameters_from_conf(argc, argv, &param, lcm, b_param);

  erlcm_tagged_node_t_subscribe(lcm, "WHEELCHAIR_MODE", tagging_handler, NULL);

  erlcm_localize_reinitialize_cmd_t_subscribe(lcm, LOCALIZE_REINITIALIZE_CHANNEL,
						 lcm_localize_reinitialize_handler, NULL);
  
  sub = erlcm_multi_gridmap_t_subscribe(lcm, "MMAP_SERVER", multi_gridmap_handler, NULL);
  sub_1 = erlcm_multi_gridmap_t_subscribe(lcm, "FINAL_MULTI_SLAM", multi_gridmap_handler, NULL);
  /* get a map */
  //create_localize_map(&map, &param);
  create_localize_mmap();

#ifndef OLD_MOTION_MODEL
  //param.motion_model = carmen_localize_motion_initialize(argc, argv);
  param.motion_model = carmen_localize_motion_initialize_from_conf(b_param);
#endif

  /* Allocate memory for the particle filter */
  //filter = carmen3d_localize_particle_filter_new(&param);

  //we're done with ipc, so disconnect
  //carmen_ipc_disconnect();


  //call initialize
  carmen_localize_initialize_message initialize_msg;
  memset(&initialize_msg, 0, sizeof(initialize_msg));
  initialize_msg.host = "self";
  initialize_msg.timestamp = carmen_get_time();

  initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

  initialize_msg.num_modes = 1;
  carmen_point_t mean = { 2, 4, M_PI / 4 };

  carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, global_carmen3d_mmap[0]);

  //  carmen_point_t std={1,1,M_PI/2};
  carmen_point_t std = { .1, .1, .1 };
  initialize_msg.mean = &mean_map;
  initialize_msg.std = &std;
 
  erlcm_floor_change_msg_t_subscribe(lcm, "FLOOR_STAUS", floor_change_handler, NULL);
  erlcm_robot_laser_t_subscribe(lcm, ROBOT_LASER_CHANNEL, lcm_robot_laser_handler, NULL); //(void*)this);
  //carmen_localize_initialize_handler(&initialize_msg);
  /*erlcm_localize_reinitialize_cmd_t_subscribe(lcm, LOCALIZE_REINITIALIZE_CHANNEL,
    lcm_localize_reinitialize_handler, NULL);*/

  lcm_dispatch(lcm);

  return 0;
}
