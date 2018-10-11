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

 #include <GL/gl.h>
 #include <getopt.h>


#include <carmen_utils/carmen.h>

#include <bot_core/bot_core.h>
#include <bot_lcmgl_client/lcmgl.h>

//bot param stuff
#include <bot_param/param_client.h>
#include <bot_param/param_util.h>
#include <bot_frames/bot_frames.h>

#include <laser_utils/laser_util.h>
#include <interfaces/map3d_interface.h>
#include <carmen_utils/carmen3d_global.h>
#include <common_utils/common_utils.h>

#include <lcmtypes/bot2_param.h>
#include <lcmtypes/map_lcmtypes.h>
#include <lcmtypes/gridmap_lcmtypes.h>
#include <lcmtypes/localizer_reinitialize_cmd_t.h>


#include "localize3dcore.h"
#include "localize3d_messages.h"
#include "localize3d_motion.h"



#define PUBLISH_GLOBAL_TO_LOCAL_HZ 10

typedef struct _state_t{
    lcm_t * lcm;
    bot_lcmgl_t * lcmgl;
    bot_lcmgl_t * lcmgl_particles;
    bot_lcmgl_t * lcmgl_laser;
    Laser_projector *laser_proj;
    gmlcm_gridmap_t* global_map_msg;
    carmen_map_p global_carmen_map;
    ripl_map_p global_carmen3d_map;
    double laser_max_range;
    BotFrames *frames;

    bot_core_pose_t *last_bot_pose;
    bot_core_planar_lidar_t *last_lidar;

    int use_rear_laser;

    carmen3d_localize_map_t map;
    carmen_map_placelist_t placelist;
    carmen3d_localize_particle_filter_p filter;
    carmen3d_localize_summary_t summary;

    carmen3d_localize_param_t param;

    carmen_robot_laser_message front_laser;

    double last_pose[3];
    int64_t last_pose_utime;

    gboolean playback;
    gboolean verbose;
    gboolean solve_for_pose;
    gboolean flip_front_laser;

    int draw_info;
    int new_map;
    int no_draw;

    int64_t last_laser_time;
    int64_t last_global_to_local_utime;

    GMainLoop *mainloop;
} state_t;

//timeout function
gboolean heartbeat_cb (gpointer data)
{
    //set this up to publish localization lost message - if sensor fails

    state_t *s = (state_t *)data;

    //do the periodic stuff

    if(!s->playback){
        //if last laser wasnt hear over 30s before
        if(fabs(s->last_laser_time - bot_timestamp_now()) > 500000){
            if(s->verbose){
                fprintf(stderr, "Laser timeout - error\n");
            }
        }
    }

    return TRUE;
}

/* publish a global position message */
// Nobody subscribes to this message
// static void publish_slam_pose(carmen_point_p particleLoc, double timestamp, state_t *s)
// {
//     static ripl_slam_pose_t slam_pose;
//     memset(&slam_pose, 0, sizeof(slam_pose));
//
//     carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*particleLoc, s->global_carmen3d_map);
//
//     slam_pose.pos[0] = summary_mean_global.x;
//     slam_pose.pos[1] = summary_mean_global.y;
//     double rpy[3] = { 0, 0, summary_mean_global.theta };
//     bot_roll_pitch_yaw_to_quat(rpy, slam_pose.orientation);
//     slam_pose.utime = common_bot_timestamp_from_double(timestamp);
//     ripl_slam_pose_t_publish(s->lcm, SLAM_POSITION_CHANNEL, &slam_pose);
// }

static void publish_localizer_pose(carmen_point_p particleLoc, double timestamp, state_t *self)
{
    static bot_core_pose_t slam_pose;
    memset(&slam_pose, 0, sizeof(slam_pose));

    carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*particleLoc, self->global_carmen3d_map);

    slam_pose.pos[0] = summary_mean_global.x;
    slam_pose.pos[1] = summary_mean_global.y;
    double rpy[3] = { 0, 0, summary_mean_global.theta };
    bot_roll_pitch_yaw_to_quat(rpy, slam_pose.orientation);
    slam_pose.utime = common_bot_timestamp_from_double(timestamp);

    bot_core_rigid_transform_t global_to_local;
    global_to_local.utime = slam_pose.utime;

    BotTrans bt_global_to_local;

    // If we are solving for the pose, publish the SLAM pose and the GLOBAL_TO_LOCAL as identity
    if (self->solve_for_pose == TRUE) {
        bot_core_pose_t_publish(self->lcm, "POSE", &slam_pose);

        // Set the global-to-local transformation to the identity transformation
        bot_trans_set_identity( &bt_global_to_local);
    }
    else {
        // The transformation from GLOBAL to LOCAL is defined as
        // T_global-to-local = T_body-to-local * T_global-to-body
        //                   = T_body-to-local * inv(T_body-to-global)
        BotTrans bt_body_to_global;
        BotTrans bt_global_to_body;
        BotTrans bt_body_to_local;

        bot_trans_set_from_quat_trans (&bt_body_to_global, slam_pose.orientation, slam_pose.pos);
        bot_trans_set_from_quat_trans (&bt_body_to_local, self->last_bot_pose->orientation, self->last_bot_pose->pos);
        bot_trans_invert (&bt_body_to_global);
        bot_trans_copy (&bt_global_to_body, &bt_body_to_global);
        bot_trans_apply_trans_to (&bt_body_to_local, &bt_global_to_body, &bt_global_to_local);
    }
    memcpy (global_to_local.trans, bt_global_to_local.trans_vec, 3*sizeof(double));
    memcpy (global_to_local.quat, bt_global_to_local.rot_quat, 4*sizeof(double));


    int64_t utime_now = bot_timestamp_now();
    if ((utime_now - self->last_global_to_local_utime) > 1e6/PUBLISH_GLOBAL_TO_LOCAL_HZ) {
        bot_core_rigid_transform_t_publish (self->lcm, "GLOBAL_TO_LOCAL", &global_to_local);
        self->last_global_to_local_utime = utime_now;
    }

    self->last_pose[0] = slam_pose.pos[0];
    self->last_pose[1] = slam_pose.pos[1];
    self->last_pose[2] = summary_mean_global.theta;
    self->last_pose_utime = slam_pose.utime;
}

int projectRangesToPoints(float * ranges, int numPoints, double thetaStart, double thetaStep, point2d_t * points,
                          double maxRange, int skip)
{

    double validRangeStart = -1000;
    double validRangeEnd = 1000;
    double * aveValidRange = NULL;
    double * stddevValidRange = NULL;

    int count = 0;
    double aveRange = 0;
    double aveRangeSq = 0;

    double theta = thetaStart;
    for (int i = 0; i < numPoints; i+=skip) {
        if (ranges[i] > .1 && ranges[i] < maxRange && theta > validRangeStart && theta < validRangeEnd) { //hokuyo driver seems to report maxRanges as .001 :-/
            //project to body centered coordinates
            points[count].x = ranges[i] * cos(theta);
            points[count].y = ranges[i] * sin(theta);
            count++;
            aveRange += ranges[i];
            aveRangeSq += ranges[i] * ranges[i];
        }

        theta += thetaStep * (skip);
    }

    aveRange /= (double) count;
    aveRangeSq /= (double) count;

    if (aveValidRange != NULL)
        *aveValidRange = aveRange;
    if (stddevValidRange != NULL)
        *stddevValidRange = sqrt(aveRangeSq - aveRange * aveRange);

    return count;
}

//drawing front robot laser message
static void draw_laser_lcmgl(carmen_point_p meanLoc, carmen_point_p maxParticleLoc, carmen_robot_laser_message *flaser,
			     double fl_offset, state_t *s)
{
    if(s->no_draw)
        return;

    point2d_t * points = (point2d_t *) calloc(flaser->num_readings, sizeof(point2d_t));
    int numValidPoints = projectRangesToPoints(flaser->range, flaser->num_readings, flaser->config.start_angle,
                                               flaser->config.fov / flaser->num_readings, points, flaser->config.maximum_range, 3);

    bot_lcmgl_point_size(s->lcmgl_laser, 3);
    bot_lcmgl_begin(s->lcmgl_laser, GL_POINTS);

    carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(*meanLoc, s->global_carmen3d_map);
    carmen_point_t max_part_global = carmen3d_map_map_to_global_coordinates(*maxParticleLoc, s->global_carmen3d_map);

    BotTrans mean_tranny;
    memset(&mean_tranny, 0, sizeof(mean_tranny));
    //change the value to be the point of the front laser

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

    double global_to_local[12];

    if (!bot_frames_get_trans_mat_3x4_with_utime (s->frames, "global",
                                                  "local", s->last_laser_time,
                                                  global_to_local)) {
        fprintf(stderr,"Frame Error\n");
        return;
    }

    for (int i = 0; i < numValidPoints; i++) {
        double p[3] = { points[i].x, points[i].y, 0 };
        double rp[3];
        bot_lcmgl_color3f(s->lcmgl_laser, 1, 0, 0);
        bot_trans_apply_vec(&mean_tranny, p, rp);

        double gl[3];
        bot_vector_affine_transform_3x4_3d (global_to_local, rp , gl);
        bot_lcmgl_vertex3f(s->lcmgl_laser, gl[0], gl[1], gl[2]);
    }
    bot_lcmgl_end(s->lcmgl_laser);
    bot_lcmgl_switch_buffer(s->lcmgl_laser);
    free(points);
}

/* publish a particle message */
static void draw_particles_lcmgl(carmen3d_localize_particle_filter_p filter, carmen3d_localize_summary_p summary, state_t *s)
{
    if(s->no_draw)
        return;
    //draw particles
    bot_lcmgl_color3f(s->lcmgl_particles, 0, 1, 1);
    bot_lcmgl_point_size(s->lcmgl_particles, 2);
    bot_lcmgl_begin(s->lcmgl_particles, GL_POINTS);

    double global_to_local[12];
    if (!bot_frames_get_trans_mat_3x4_with_utime (s->frames, "global",
                                                  "local", s->last_laser_time,
                                                  global_to_local)) {
        fprintf(stderr,"Frame Error\n");
        return;
    }

    for (int i = 0; i < filter->param->num_particles; i++) {
        carmen_point_t particle_map = { filter->particles[i].x, filter->particles[i].y, filter->particles[i].theta };
        carmen_point_t particle_global = carmen3d_map_map_to_global_coordinates(particle_map, s->global_carmen3d_map);

        double particle_gl[] = {particle_global.x, particle_global.y, 0};
        double particle_lc[3] = {0};

        bot_vector_affine_transform_3x4_3d (global_to_local, particle_gl , particle_lc);

        if(s->draw_info){
            double heading_length = 0.2;
            double particle_gl_1[3] = {particle_global.x + heading_length * cos(particle_global.theta),
                                       particle_global.y + heading_length * sin(particle_global.theta), 0};

	    bot_vector_affine_transform_3x4_3d (global_to_local, particle_gl, particle_lc);

	    bot_lcmgl_vertex3f(s->lcmgl, particle_lc[0], particle_lc[1], 0);

	    //these need to be converted to local frame

	    bot_lcmgl_vertex3f(s->lcmgl_particles, particle_lc[0], particle_lc[1], 0);
	}
    }

    bot_lcmgl_end(s->lcmgl_particles);
    bot_lcmgl_switch_buffer(s->lcmgl_particles);

    //draw summary
    //mean
    carmen_point_t summary_mean_global = carmen3d_map_map_to_global_coordinates(summary->mean, s->global_carmen3d_map);
    bot_lcmgl_color3f(s->lcmgl, 0, 1, 0);
    bot_lcmgl_point_size(s->lcmgl, 15);
    bot_lcmgl_begin(s->lcmgl, GL_POINTS);

    double mean_gl[3] = {summary_mean_global.x, summary_mean_global.y, 0};
    double mean_lc[3] = {0};
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);
    bot_lcmgl_end(s->lcmgl);

    //heading
    double headingLength = .8;
    bot_lcmgl_color3f(s->lcmgl, 1, 1, 0);
    bot_lcmgl_point_size(s->lcmgl, 8);
    bot_lcmgl_line_width(s->lcmgl, 8);
    bot_lcmgl_begin(s->lcmgl, GL_LINES);
    //bot_lcmgl_vertex3f(s->lcmgl, summary_mean_global.x, summary_mean_global.y, 0);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);

    double mean_gl_1[3] = {summary_mean_global.x + headingLength * cos(summary_mean_global.theta),
                           summary_mean_global.y + headingLength * sin(summary_mean_global.theta), 0};

    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl_1, mean_lc);

    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);

    //bot_lcmgl_vertex3f(s->lcmgl, summary_mean_global.x + headingLength * cos(summary_mean_global.theta),
    //                 summary_mean_global.y + headingLength * sin(summary_mean_global.theta), 0);


    bot_lcmgl_end(s->lcmgl);

    //cov
    double cov_factor = 10;
    bot_lcmgl_color3f(s->lcmgl, 1, 0, 0);
    bot_lcmgl_point_size(s->lcmgl, 5);
    bot_lcmgl_line_width(s->lcmgl, 5);
    bot_lcmgl_begin(s->lcmgl, GL_LINES);
    mean_gl[0] = summary_mean_global.x;
    mean_gl[1] = summary_mean_global.y;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);
    mean_gl[0] = summary_mean_global.x + cov_factor * summary->std.x;
    mean_gl[1] = summary_mean_global.y;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);
    bot_lcmgl_end(s->lcmgl);

    bot_lcmgl_begin(s->lcmgl, GL_LINES);
    mean_gl[0] = summary_mean_global.x;
    mean_gl[1] = summary_mean_global.y;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);

    mean_gl[1] = summary_mean_global.y + cov_factor * summary->std.y;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);

    bot_lcmgl_end(s->lcmgl);

    bot_lcmgl_begin(s->lcmgl, GL_LINES);
    mean_gl[0] = summary_mean_global.x;
    mean_gl[1] = summary_mean_global.y;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], 0);
    mean_gl[2] = cov_factor * summary->std.theta;
    bot_vector_affine_transform_3x4_3d (global_to_local, mean_gl, mean_lc);
    bot_lcmgl_vertex3f(s->lcmgl, mean_lc[0], mean_lc[1], mean_lc[2]);
    bot_lcmgl_end(s->lcmgl);

    bot_lcmgl_switch_buffer(s->lcmgl);

}


/* process initialization messages */

void carmen_localize_initialize_handler(carmen_localize_initialize_message *initialize_msg, state_t *s)
{
    if (initialize_msg->distribution == CARMEN_INITIALIZE_GAUSSIAN)
        carmen3d_localize_initialize_particles_gaussians(s->filter, initialize_msg->num_modes, initialize_msg->mean,
                                                         initialize_msg->std);
    else if (initialize_msg->distribution == CARMEN_INITIALIZE_UNIFORM) {
        carmen3d_localize_initialize_particles_uniform(s->filter, &(s->front_laser), &(s->map));
    }
    draw_particles_lcmgl(s->filter, &s->summary, s);
}

/*void carmen_localize_initialize_placename_handler(carmen_localize_initialize_placename_message *init_place)
{
    carmen3d_localize_initialize_particles_placename(filter, &placelist, init_place->placename);
    //draw_particles_lcmgl(filter, &summary);
    }*/

void robot_frontlaser_handler(carmen_robot_laser_message *flaser, state_t *s)
{
    //logic to stop processing if the robot stops - prevents divergence of the filter
    static double last_b_pose[2] = {.0,.0};
    double dist_travelled;

    dist_travelled = hypot(flaser->robot_pose.x - last_b_pose[0],flaser->robot_pose.y - last_b_pose[1]);
    last_b_pose[0] = flaser->robot_pose.x;
    last_b_pose[1] = flaser->robot_pose.y;

    /*if(dist_travelled==0){
      fprintf(stdout,"-");
      return;
      }*/

    if(!s->filter->initialized){
        //call initialize
        carmen_localize_initialize_message initialize_msg;
        memset(&initialize_msg, 0, sizeof(initialize_msg));
        initialize_msg.host = "self";
        initialize_msg.timestamp = carmen_get_time();

        initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

        initialize_msg.num_modes = 1;
        carmen_point_t mean = { 2, 4, M_PI / 4 };

        carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, s->global_carmen3d_map);

        //  carmen_point_t std={1,1,M_PI/2};
        carmen_point_t std = { .1, .1, .1 };
        initialize_msg.mean = &mean_map;
        initialize_msg.std = &std;


        //this has a slight issue - uses the flaser - which is not initiaized
        carmen_localize_initialize_handler(&initialize_msg, s);
        fprintf(stderr,"Initializing Filter\n");
        return;
    }

    static carmen_point_t prev_robot_position =  {.0,.0,.0};
    carmen_point_t robot_position;
    robot_position.x = flaser->robot_pose.x;
    robot_position.y = flaser->robot_pose.y;
    robot_position.theta = flaser->robot_pose.theta;
    carmen_point_t delta = carmen3d_body2D_difference(&robot_position,&prev_robot_position);

    prev_robot_position = robot_position;

    int stationary = 0;
    if(sqrt(delta.x * delta.x + delta.y * delta.y) == 0)
        stationary = 1;

    if (!stationary || s->new_map) {
        carmen3d_localize_run(s->filter, &(s->map), flaser, s->filter->param->front_laser_offset, 0);
        if (s->filter->initialized)
            carmen3d_localize_summarize(s->filter, &(s->summary), &(s->map), flaser->num_readings, flaser->range,
                                        s->filter->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);

        if(s->new_map)
            s->new_map = 0;
    }

    if (s->filter->initialized) {
        double maxWeight = -1e9;
        carmen_point_t maxParticle;
        memset(&maxParticle, 0, sizeof(maxParticle));
        for (int i = 0; i < s->filter->param->num_particles; i++) {
            if (s->filter->particles->weight > maxWeight) {
                maxWeight = s->filter->particles->weight;
                maxParticle.x = s->filter->particles->x;
                maxParticle.y = s->filter->particles->y;
                maxParticle.theta = s->filter->particles->theta;
            }
        }

        if (s->summary.converged) {
            //publish_slam_pose(&(s->summary.mean),flaser->timestamp, s);
            publish_localizer_pose(&(s->summary.mean),flaser->timestamp, s);
        }
        //draw_particles_lcmgl(filter, &summary);
        draw_particles_lcmgl(s->filter, &s->summary, s);
        draw_laser_lcmgl(&(s->summary.mean), &maxParticle, flaser, s->filter->param->front_laser_offset, s);
    }
}

void robot_rearlaser_handler(carmen_robot_laser_message *flaser, state_t *s)
{
    //the rear offset is reflected in the robot laser start angle
    //logic to stop processing if the robot stops - prevents divergence of the filter
    static double last_b_pose[2] = {.0,.0};
    double dist_travelled;
    dist_travelled = hypot(flaser->robot_pose.x - last_b_pose[0],flaser->robot_pose.y - last_b_pose[1]);
    last_b_pose[0] = flaser->robot_pose.x;
    last_b_pose[1] = flaser->robot_pose.y;
    //this is bad
    /*if(dist_travelled==0){
      fprintf(stdout,"-");
      return;
      }*/
    if(!s->filter->initialized){
        //call initialize
        carmen_localize_initialize_message initialize_msg;
        memset(&initialize_msg, 0, sizeof(initialize_msg));
        initialize_msg.host = "self";
        initialize_msg.timestamp = carmen_get_time();

        initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

        initialize_msg.num_modes = 1;
        carmen_point_t mean = { 2, 4, M_PI / 4 };

        carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, s->global_carmen3d_map);

        //  carmen_point_t std={1,1,M_PI/2};
        carmen_point_t std = { .1, .1, .1 };
        initialize_msg.mean = &mean_map;
        initialize_msg.std = &std;


        //this has a slight issue - uses the flaser - which is not initiaized
        carmen_localize_initialize_handler(&initialize_msg, s);
        fprintf(stderr,"Initializing Filter\n");
        return;
    }

    carmen3d_localize_run(s->filter, &(s->map), flaser, s->filter->param->rear_laser_offset, 0);
    if (s->filter->initialized) {
        carmen3d_localize_summarize(s->filter, &(s->summary), &(s->map), flaser->num_readings, flaser->range,
                                    s->filter->param->front_laser_offset, flaser->config.angular_resolution, flaser->config.start_angle, 0);

        double maxWeight = -1e9;
        carmen_point_t maxParticle;
        memset(&maxParticle, 0, sizeof(maxParticle));
        for (int i = 0; i < s->filter->param->num_particles; i++) {
            if (s->filter->particles->weight > maxWeight) {
                maxWeight = s->filter->particles->weight;
                maxParticle.x = s->filter->particles->x;
                maxParticle.y = s->filter->particles->y;
                maxParticle.theta = s->filter->particles->theta;
            }
        }

        if (s->summary.converged) {
            //publish_slam_pose(&(s->summary.mean),flaser->timestamp, s);
            publish_localizer_pose(&(s->summary.mean),flaser->timestamp, s);
        }
        //draw_particles_lcmgl(filter, &summary);
        draw_particles_lcmgl(s->filter, &s->summary, s);
        draw_laser_lcmgl(&(s->summary.mean), &maxParticle, flaser, s->filter->param->rear_laser_offset, s);
    }
}

static void on_pose(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                    const char * channel __attribute__((unused)),
                    const bot_core_pose_t * msg,
                    void * user  __attribute__((unused))) {

    state_t *s = (state_t *) user;

    if(s->last_bot_pose != NULL){
        bot_core_pose_t_destroy(s->last_bot_pose);
    }
    s->last_bot_pose = bot_core_pose_t_copy(msg);

    //if (s->solve_for_pose == FALSE && !strcmp (channel, "POSE_DEADRECKON"))
    //    bot_core_pose_t_publish (s->lcm, "POSE", msg);
}

void on_planar_lidar(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                     const char * channel __attribute__((unused)),
                     const bot_core_planar_lidar_t * msg,
                     void * user  __attribute__((unused)))
{

    state_t *self = (state_t *) user;

    static bot_core_planar_lidar_t *static_msg = NULL;
    if (static_msg)
        bot_core_planar_lidar_t_destroy (static_msg);
    static_msg = bot_core_planar_lidar_t_copy (msg);

    if (self->flip_front_laser && !strcmp(channel, "SKIRT_FRONT")) {
        for (int i=0; i<static_msg->nranges; i++)
            static_msg->ranges[i] = msg->ranges[msg->nranges-1-i];
    }

    self->last_laser_time = static_msg->utime;

    //set invalid regions to maxrange
    for (int i=0; i<static_msg->nranges;i++){
        if (static_msg->ranges[i]< self->laser_proj->min_range ||
            static_msg->ranges[i]> self->laser_proj->max_range ||
            i<self->laser_proj->surroundRegion[0] ||
            i>self->laser_proj->surroundRegion[1]){
            static_msg->ranges[i]=self->laser_proj->max_range+1;
        }
    }

    // This is still a hack as we pack the lidar and pose into a robot_laser_message
    // ToDo: Remove any notion of robot_laser message
    if (!self->last_bot_pose) {
        fprintf (stdout, "on_planar_lidar: No pose\n");
        return;
    }

    carmen_robot_laser_message *carmen_msg = &(self->front_laser);
    memset(carmen_msg, 0, sizeof(carmen_robot_laser_message));
    carmen_msg->laser_pose.x = self->last_bot_pose->pos[0];
    carmen_msg->laser_pose.y = self->last_bot_pose->pos[1];
    double rpy[3];
    bot_quat_to_roll_pitch_yaw(self->last_bot_pose->orientation, rpy);
    carmen_msg->laser_pose.theta = rpy[2];
    carmen_msg->robot_pose = carmen_msg->laser_pose;

    carmen_msg->config.accuracy = .1;
    carmen_msg->config.laser_type = HOKUYO_UTM;
    carmen_msg->config.remission_mode = REMISSION_NONE;
    carmen_msg->config.angular_resolution = static_msg->radstep;
    carmen_msg->config.fov = static_msg->nranges * static_msg->radstep;
    carmen_msg->config.maximum_range = self->laser_proj->max_range;
    if(!self->use_rear_laser){
        carmen_msg->config.start_angle = static_msg->rad0;
    }
    else{
        carmen_msg->config.start_angle = static_msg->rad0;
    }


    carmen_msg->id = 0;
    carmen_msg->num_readings = static_msg->nranges;
    carmen_msg->range = static_msg->ranges;
    carmen_msg->num_remissions = static_msg->nintensities;
    carmen_msg->remission = static_msg->intensities;
    carmen_msg->timestamp = common_bot_timestamp_to_double(static_msg->utime);
    carmen_msg->host = (char *) "lcm";

    //TODO:there are some other fields like tooclose and turn_axis etc that I don't think are used...
    if(!self->use_rear_laser)
        robot_frontlaser_handler(carmen_msg, self);
    else
        robot_rearlaser_handler(carmen_msg, self);
}

void lcm_localize_reinitialize_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)),
                                       const char *channel __attribute__((unused)),
                                       const localizer_reinitialize_cmd_t *msg,
                                       void *user)
{
    state_t *s = (state_t *) user;

    if(s->global_map_msg ==NULL){
        return;
    }
    fprintf(stderr,"\ngot new reinitialize msg (%f, %f, %f) (%f, %f, %f)\n\n",
           msg->mean[0], msg->mean[1], msg->mean[2],
           sqrt(msg->variance[0]), sqrt(msg->variance[1]), sqrt(msg->variance[2]));

    carmen_localize_initialize_message initialize_msg;
    memset(&initialize_msg, 0, sizeof(initialize_msg));
    initialize_msg.host = "self";
    initialize_msg.timestamp = carmen_get_time();

    initialize_msg.distribution = CARMEN_INITIALIZE_GAUSSIAN;
    initialize_msg.num_modes = 1;

    // Convert the points to the global frame (transformation may be identity)
    double mean_global[3];
    double mean_local[] = {msg->mean[0], msg->mean[1], 0};
    double quat_local[4], quat_global[4];
    double rpy_global[3];
    double rpy_local[] = {0, 0, msg->mean[2]};
    bot_roll_pitch_yaw_to_quat (rpy_local, quat_local);
    BotTrans local_to_global;

    if (!bot_frames_get_trans (s->frames, "local", "global", &local_to_global))
        fprintf (stderr, "lcm_localize_reinitialize_handler: Error getting transformation from local to global frame\n");

    bot_quat_mult (quat_global, local_to_global.rot_quat, quat_local);
    bot_quat_to_roll_pitch_yaw (quat_global, rpy_global);
    bot_frames_transform_vec (s->frames, "local", "global", mean_local, mean_global);
    carmen_point_t mean = { mean_global[0], mean_global[1], rpy_global[2] };

    carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, s->global_carmen3d_map);

    carmen_point_t std = {
        sqrt(msg->variance[0]),
        sqrt(msg->variance[1]),
        sqrt(msg->variance[2])
    };
    initialize_msg.mean = &mean_map;
    initialize_msg.std = &std;

    carmen_localize_initialize_handler(&initialize_msg, s);
}

//get params from server

void read_parameters_from_conf(int argc, char **argv, carmen3d_localize_param_p param,
                               lcm_t *lcm, BotParam * b_param, state_t *self )
{
    BotFrames *frames = bot_frames_get_global (lcm, b_param);

    double integrate_angle_deg;
    integrate_angle_deg = 1.0;


    // Find the position of the forward-facing LIDAR
    char *coord_frame;
    coord_frame = bot_param_get_planar_lidar_coord_frame (b_param, "SKIRT_FRONT");
    if (!coord_frame)
        fprintf (stderr, "\tError determining front laser coordinate frame\n");

    BotTrans front_laser_to_body;
    if (!bot_frames_get_trans (frames, coord_frame, "body", &front_laser_to_body))
        fprintf (stderr, "\tError determining front laser coordinate frame\n");
    else
        fprintf(stderr,"\tFront Laser Pos : (%f,%f,%f)\n",
                front_laser_to_body.trans_vec[0], front_laser_to_body.trans_vec[1], front_laser_to_body.trans_vec[2]);

    param->front_laser_offset = front_laser_to_body.trans_vec[0];
    param->front_laser_side_offset = front_laser_to_body.trans_vec[1];

    double front_rpy[3];
    bot_quat_to_roll_pitch_yaw (front_laser_to_body.rot_quat, front_rpy);
    param->front_laser_angle_offset = front_rpy[2];


    // Now for the rear laser
    if (self->use_rear_laser) {
        coord_frame = NULL;
        coord_frame = bot_param_get_planar_lidar_coord_frame (b_param, "SKIRT_REAR");

        if (!coord_frame)
            fprintf (stderr, "\tError determining rear laser coordinate frame\n");

        BotTrans rear_laser_to_body;
        if (!bot_frames_get_trans (frames, coord_frame, "body", &rear_laser_to_body))
            fprintf (stderr, "\tError determining LIDAR coordinate frame\n");
        else
            fprintf(stderr,"\tRear Laser Pos : (%f,%f,%f)\n",
                    rear_laser_to_body.trans_vec[0], rear_laser_to_body.trans_vec[1], rear_laser_to_body.trans_vec[2]);

        param->rear_laser_offset = rear_laser_to_body.trans_vec[0];
        param->rear_laser_side_offset = rear_laser_to_body.trans_vec[1];

        double rear_rpy[3];
        bot_quat_to_roll_pitch_yaw (rear_laser_to_body.rot_quat, rear_rpy);
        param->rear_laser_angle_offset = rear_rpy[2];
    }

    /*
     * double position[3];
     * char key[2048];
     *
     * sprintf(key, "%s.%s.%s.position", "calibration", "planar_lidars", "SKIRT_FRONT");
     *
     * if(3 != bot_param_get_double_array(b_param, key, position, 3)){
     *     fprintf(stderr,"\tError Reading Params\n");
     * }
     * else{
     *     fprintf(stderr,"\tFront Laser Pos : (%f,%f,%f)\n",position[0], position[1], position[2]);
     * }
     *
     *
     * param->front_laser_offset = position[0]; //bot_param_get_double_or_fail(b_param, "robot.frontlaser_offset");
     * param->front_laser_side_offset =  position[1];
     *
     * fprintf(stderr, "%f\n", param->front_laser_offset);
     *
     * double rpy[3];
     *
     * sprintf(key, "%s.%s.%s.rpy", "calibration", "planar_lidars", "SKIRT_FRONT");
     *
     *
     * if(3 != bot_param_get_double_array(b_param, key, rpy, 3)){
     *     fprintf(stderr,"\tError Reading Params\n");
     * }else{
     *     fprintf(stderr,"\tFront Laser RPY : (%f,%f,%f)\n",rpy[0], rpy[1], rpy[2]);
     * }
     *
     * param->front_laser_angle_offset = bot_to_radians(rpy[2]);//bot_param_get_double_or_fail(b_param,"robot.frontlaser_angular_offset");
     *
     * sprintf(key, "%s.%s.%s.position", "calibration", "planar_lidars", "SKIRT_REAR");
     * if(3 != bot_param_get_double_array(b_param, key, position, 3)){
     *     fprintf(stderr,"\tError Reading Params\n");
     * }
     * else{
     *     fprintf(stderr,"\tRear Laser Pos : (%f,%f,%f)\n",position[0], position[1], position[2]);
     * }
     *
     * param->rear_laser_offset = position[0]; //bot_param_get_double_or_fail(b_param, "robot.rearlaser_offset");
     *
     * param->rear_laser_side_offset = position[1]; //bot_param_get_double_or_fail(b_param,"robot.rearlaser_side_offset");
     *
     * sprintf(key, "%s.%s.%s.rpy", "calibration", "planar_lidars", "SKIRT_REAR");
     *
     * if(3 != bot_param_get_double_array(b_param, key, rpy, 3)){
     *     fprintf(stderr,"\tError Reading Params\n");
     * }else{
     *     fprintf(stderr,"\tRear Laser RPY : (%f,%f,%f)\n",rpy[0], rpy[1], rpy[2]);
     * }
     *
     * param->rear_laser_angle_offset = bot_to_radians(rpy[2]);
     */

    param->use_rear_laser = bot_param_get_boolean_or_fail(b_param,"localizer.use_rear_laser");
    param->num_particles =bot_param_get_int_or_fail(b_param,"localizer.num_particles");
    param->max_range = bot_param_get_double_or_fail(b_param,"localizer.laser_max_range");
    param->min_wall_prob = bot_param_get_double_or_fail(b_param,"localizer.min_wall_prob");
    param->outlier_fraction = bot_param_get_double_or_fail(b_param,"localizer.outlier_fraction");
    param->update_distance = bot_param_get_double_or_fail(b_param,"localizer.update_distance");
    param->update_heading = bot_param_get_double_or_fail(b_param,"localizer.update_heading");
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


static void gridmap_handler(const lcm_recv_buf_t *rbuf, const char *channel, const gmlcm_gridmap_t *msg, void *user)
{
    state_t *s = (state_t *) user;

    fprintf(stderr,"Gridmap Received\n");
    if (s->global_map_msg != NULL) {
        gmlcm_gridmap_t_destroy(s->global_map_msg);
    }
    s->global_map_msg = gmlcm_gridmap_t_copy(msg);

    fprintf(stderr,"New map recieved\n");

    carmen3d_map_uncompress_lcm_map(s->global_carmen3d_map, s->global_map_msg);

    if(s->global_carmen_map !=NULL){
        free(s->global_carmen_map);
    }

    s->global_carmen_map = carmen3d_map_map3d_map_copy(s->global_carmen3d_map);

    /* create a localize map */
    carmen_warn("Creating likelihood maps... ");
    carmen3d_to_localize_map(s->global_carmen_map, &(s->map), &(s->param));

    if(s->last_pose_utime > 0){//for now use the last pose - prob should check if we have a good pose
        //reinitialized the filters
        fprintf(stderr,"Reinitializing Filter\n");
        carmen_localize_initialize_message initialize_msg;
        memset(&initialize_msg, 0, sizeof(initialize_msg));
        initialize_msg.host = "self";
        initialize_msg.timestamp = carmen_get_time();

        initialize_msg.distribution = CARMEN_INITIALIZE_GAUSSIAN;
        initialize_msg.num_modes = 1;
        carmen_point_t mean = {s->last_pose[0],s->last_pose[1], s->last_pose[2] };
        //this should have a floor attched - or use the current floor
        carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, s->global_carmen3d_map);

        //fprintf(stderr,"P: Last Pose : %f,%f,%f => Map Pose : %f,%f,%f\n", last_pose[0],
        //      last_pose[1], last_pose[2], mean_map.x, mean_map.y, mean_map.theta);

        carmen_point_t std = {
            sqrt(0.01),
            sqrt(0.01),
            sqrt(carmen_degrees_to_radians(0.1))
        };
        initialize_msg.mean = &mean_map;
        initialize_msg.std = &std;

        carmen_localize_initialize_handler(&initialize_msg, s);

        s->new_map = 1;
    }
    fprintf(stderr,"Done");
    //this just coppies the message - doesnt do anything else with it - need to actually have the localize listen to and switch maps
    //on the fly
}

/* create localize specific maps */

void create_localize_map(carmen3d_localize_map_t *map, carmen3d_localize_param_p param, state_t *s)
{
    /* subscripe to map, and wait for it to come in... */
    maplcm_map_request_msg_t msg;
    msg.utime =  carmen_get_time()*1e6;
    msg.requesting_prog = "LOCALIZE";



    gmlcm_gridmap_t_subscription_t * sub = gmlcm_gridmap_t_subscribe(s->lcm, "MAP_SERVER", gridmap_handler,
                                                                     s);

    gmlcm_gridmap_t_subscription_t * sub_1 = gmlcm_gridmap_t_subscribe(s->lcm, "FINAL_SLAM", gridmap_handler,
                                                                       s);
    fprintf(stderr,"Waiting for a map");
    int sent_map_req = 0;

    while (s->global_map_msg == NULL) {
        if(!sent_map_req){
            sent_map_req = 1;
            maplcm_map_request_msg_t_publish(s->lcm,"MAP_REQUEST_CHANNEL",&msg);
        }
        common_sleep(s->lcm, .25);

        carmen_warn(".");
    }
    carmen_warn("Got a map\n");

    //copies and uncompress the carmen3d map to global map
    carmen3d_map_uncompress_lcm_map(s->global_carmen3d_map, s->global_map_msg);
    s->global_carmen_map = carmen3d_map_map3d_map_copy(s->global_carmen3d_map);

    // create a localize map
    carmen_warn("Creating likelihood maps... ");
    carmen3d_to_localize_map(s->global_carmen_map, map, param);

    if(s->last_pose_utime > 0){//for now use the last pose - prob should check if we have a good pose
        //reinitialized the filters
        fprintf(stderr,"Reinitializing Filter\n");
        carmen_localize_initialize_message initialize_msg;
        memset(&initialize_msg, 0, sizeof(initialize_msg));
        initialize_msg.host = "self";
        initialize_msg.timestamp = carmen_get_time();

        initialize_msg.distribution = CARMEN_INITIALIZE_GAUSSIAN;
        initialize_msg.num_modes = 1;
        carmen_point_t mean = {s->last_pose[0],s->last_pose[1], s->last_pose[2] };
        //this should have a floor attched - or use the current floor
        carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, s->global_carmen3d_map);

        carmen_point_t std = {
            sqrt(0.01),
            sqrt(0.01),
            sqrt(carmen_degrees_to_radians(0.1))
        };
        initialize_msg.mean = &mean_map;
        initialize_msg.std = &std;

        carmen_localize_initialize_handler(&initialize_msg, s);
    }

    carmen_warn("done.\n");
}

void tagging_handler(const lcm_recv_buf_t *rbuf __attribute__((unused)), const char * channel __attribute__((unused)),
                     const maplcm_tagged_node_t * msg,
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

static void usage(char * funcName)
{
    printf("Usage: %s [options]\n"
           "options are:\n"
           "--help,             -h    display this message \n"
           "--verbose           -v    verbose  \n"
           "--mode              -m    mode (live/playback) default - live\n"
           "--reverse,          -r    use rear laser\n"
           "--no-draw           -n    don't draw laser and pose\n"
           "--flip              -f    flip front laser (when upside down)\n"
           "--solve_pose        -s    solve for and publish pose\n", funcName);
    exit(1);

}

int main(int argc, char **argv)
{
    g_thread_init(NULL);
    setlinebuf (stdout);

    state_t *self = (state_t*) calloc(1, sizeof(state_t));

    const char *optstring = "hfrm:vsn";
    struct option long_opts[] = { { "help", no_argument, 0, 'h' },
                                  { "verbose", no_argument, 0, 'v' },
                                  { "reverse", no_argument, 0, 'r' },
                                  { "solve_pose", no_argument, 0, 's' },
                                  { "no-draw", no_argument, 0, 'n' },
                                  { "flip", no_argument, 0, 'f' },
                                  { "mode", required_argument , 0, 'm' },
                                  { 0, 0, 0, 0 } };

    self->solve_for_pose = FALSE;

    int c;
    while ((c = getopt_long(argc, argv, optstring, long_opts, 0)) >= 0) {
        switch (c) {
        case 'h':
            usage(argv[0]);
            break;
        case 'n':
            self->no_draw = 0;
            break;
        case 'v':
            self->verbose = TRUE;
            break;
        case 's':
            self->solve_for_pose = TRUE;
            break;
        case 'f':
            self->flip_front_laser = TRUE;
            break;
        case 'r':
            {
                fprintf(stderr,"Using rear laser\n");
                self->use_rear_laser = 1;
                break;
            }
        case 'm':
            {
                if(!strcmp(optarg, "playback")){
                    self->playback = TRUE;
                    fprintf(stderr, "In Playback Mode - no timeout errors\n");
                }
                break;
            }
        default:
            {
                usage(argv[0]);
                break;
            }
        }
    }

    carmen_randomize(&argc, &argv);

    self->lcm = bot_lcm_get_global(NULL);
    self->lcmgl = bot_lcmgl_init(self->lcm, "localize3d");
    self->lcmgl_particles = bot_lcmgl_init(self->lcm,"localize3d_particles");
    self->lcmgl_laser = bot_lcmgl_init(self->lcm,"localize3d_laser");

    BotParam * b_param = bot_param_new_from_server(self->lcm, 1);
    self->frames = bot_frames_get_global (self->lcm, b_param);

    if(!self->use_rear_laser){
        self->laser_proj = laser_projector_new(b_param, NULL, "SKIRT_FRONT",0);
    }
    else{
        self->laser_proj = laser_projector_new(b_param, NULL, "SKIRT_REAR",0);
    }

    /* Setup exit handler */
    signal(SIGINT, shutdown_localize);

    /* Initialize all the relevant parameters */
    //read_parameters(argc, argv, &param);

    read_parameters_from_conf(argc, argv, &(self->param), self->lcm, b_param, self);


    self->global_carmen_map = (carmen_map_p) calloc(1, sizeof(carmen_map_t));
    carmen_test_alloc(self->global_carmen_map);

    self->global_carmen3d_map = (ripl_map_p) calloc(1, sizeof(ripl_map_t));
    carmen_test_alloc(self->global_carmen3d_map);

    maplcm_tagged_node_t_subscribe(self->lcm, "WHEELCHAIR_MODE", tagging_handler, self);

    localizer_reinitialize_cmd_t_subscribe(self->lcm, "LOCALIZE_REINITIALIZE", lcm_localize_reinitialize_handler, self);


    /* get a map */
    create_localize_map(&(self->map), &(self->param), self);

#ifndef OLD_MOTION_MODEL
    //param.motion_model = carmen_localize_motion_initialize(argc, argv);
    self->param.motion_model = carmen_localize_motion_initialize_from_conf(b_param);
#endif

    /* Allocate memory for the particle filter */
    self->filter = carmen3d_localize_particle_filter_new(&(self->param));

    //we're done with ipc, so disconnect
    //carmen_ipc_disconnect();

    memset(&(self->front_laser), 0, sizeof(carmen_robot_laser_message));

    //call initialize
    carmen_localize_initialize_message initialize_msg;
    memset(&initialize_msg, 0, sizeof(initialize_msg));
    initialize_msg.host = "self";
    initialize_msg.timestamp = carmen_get_time();

    initialize_msg.distribution = CARMEN_INITIALIZE_UNIFORM;

    initialize_msg.num_modes = 1;
    carmen_point_t mean = { 2, 4, M_PI / 4 };

    carmen_point_t mean_map = carmen3d_map_global_to_map_coordinates(mean, self->global_carmen3d_map);

    //  carmen_point_t std={1,1,M_PI/2};
    carmen_point_t std = { .1, .1, .1 };
    initialize_msg.mean = &mean_map;
    initialize_msg.std = &std;

    //this has a slight issue - uses the flaser - which is not initiaized
    //carmen_localize_initialize_handler(&initialize_msg);

    // Two operating modes wrt pose:
    //   1) Publish pose and LOCAL_TO_GLOBAL: Takes in LIDAR and POSE_DEADRECKON
    //                                        and publishes POSE and LOCAL_TO_GLOBAL
    //   2) Publish LOCAL_TO_GLOBAL: Takes in LIDAR and POSE and publishes LOCAL_TO_GLOBAL
    if (self->solve_for_pose == TRUE)
        bot_core_pose_t_subscribe (self->lcm, "POSE_DEADRECKON", on_pose, self);
    else
        bot_core_pose_t_subscribe (self->lcm, "POSE", on_pose, self);

    if(self->use_rear_laser)
        bot_core_planar_lidar_t_subscribe(self->lcm, "SKIRT_REAR" , on_planar_lidar, self);
    else
        bot_core_planar_lidar_t_subscribe(self->lcm, "SKIRT_FRONT" , on_planar_lidar, self);

    self->mainloop = g_main_loop_new( NULL, FALSE );

    bot_glib_mainloop_attach_lcm (self->lcm);

    /* heart beat*/
    //called every 100 ms (more or less)
    g_timeout_add(100, heartbeat_cb, self);

    //adding proper exiting
    bot_signal_pipe_glib_quit_on_kill (self->mainloop);

    ///////////////////////////////////////////////
    g_main_loop_run(self->mainloop);

    bot_glib_mainloop_detach_lcm(self->lcm);

    //    common_lcm_dispatch(lcm);

    return 0;
}
