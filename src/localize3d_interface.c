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

#include <carmen_utils/carmen.h>
#include "localize3d_messages.h"

void
carmen_localize_correct_odometry(carmen_base_odometry_message *odometry,
				 carmen_localize_globalpos_message *globalpos)
{
  int backwards;
  double dr1, dt, dr2;
  double dx, dy;

  dx = odometry->x - globalpos->odometrypos.x;
  dy = odometry->y - globalpos->odometrypos.y;
  dt = sqrt(dx * dx + dy * dy);
  backwards = (dx * cos(odometry->theta) + dy * sin(odometry->theta) < 0);

  /* The dr1/dr2 code becomes unstable if dt is too small. */
  if(dt < 0.05)
    {
      dr1 = carmen_normalize_theta(odometry->theta -
				   globalpos->odometrypos.theta) / 2.0;
      dr2 = dr1;
    }
  else
    {
      if(backwards)
	dr1 = carmen_normalize_theta(atan2(globalpos->odometrypos.y -
					   odometry->y,
					   globalpos->odometrypos.x -
					   odometry->x) -
				     globalpos->odometrypos.theta);
      else
	dr1 = carmen_normalize_theta(atan2(odometry->y -
					   globalpos->odometrypos.y,
					   odometry->x -
					   globalpos->odometrypos.x) -
				     globalpos->odometrypos.theta);
      dr2 = carmen_normalize_theta(odometry->theta -
				   globalpos->odometrypos.theta - dr1);
    }
  if(backwards)
    dt = -dt;
  odometry->x = globalpos->globalpos.x + dt *
    cos(globalpos->globalpos.theta + dr1);
  odometry->y = globalpos->globalpos.y + dt *
    sin(globalpos->globalpos.theta + dr1);
  odometry->theta = carmen_normalize_theta(globalpos->globalpos.theta +
					   dr1 + dr2);
}

void
carmen_localize3d_correct_laser(carmen_robot_laser_message *laser,
			      carmen_localize_globalpos_message *globalpos)
{
  int backwards;
  double dr1, dt, dr2;
  double dx, dy;
  double dtheta;

  fprintf(stderr,"Laser Pose (%f,%f)\n", laser->laser_pose.x, laser->laser_pose.y);

  dx = laser->laser_pose.x - globalpos->odometrypos.x;
  dy = laser->laser_pose.y - globalpos->odometrypos.y;
  dtheta = laser->laser_pose.theta - globalpos->odometrypos.theta;

  dt = sqrt(dx * dx + dy * dy);
  backwards = (dx * cos(laser->laser_pose.theta) +
	       dy * sin(laser->laser_pose.theta) < 0);

  /* The dr1/dr2 code becomes unstable if dt is too small. */
  if(dt < 0.05) {
    dr1 = carmen_normalize_theta(laser->laser_pose.theta -
				 globalpos->odometrypos.theta) / 2.0;
    dr2 = dr1;
  } else {
    if(backwards)
      dr1 = carmen_normalize_theta(atan2(-dy, -dx)-
				   globalpos->odometrypos.theta);
    else
      dr1 = carmen_normalize_theta(atan2(dy, dx)-
				   globalpos->odometrypos.theta);
    dr2 = carmen_normalize_theta(dtheta - dr1);
    }
  if(backwards)
    dt = -dt;
  laser->laser_pose.x = globalpos->globalpos.x + dt *
    cos(globalpos->globalpos.theta + dr1);
  laser->laser_pose.y = globalpos->globalpos.y + dt *
    sin(globalpos->globalpos.theta + dr1);
  laser->laser_pose.theta =
    carmen_normalize_theta(globalpos->globalpos.theta + dr1 + dr2);
}

