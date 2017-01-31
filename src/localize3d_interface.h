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


/** @addtogroup localize liblocalize_interface **/
// @{

/** \file localize_interface.h
 * \brief Definition of the interface of the module localize.
 *
 * This file specifies the interface to subscribe the messages of
 * that module and to receive its data via ipc.
 **/

 
#ifndef CARMEN3D_LOCALIZE_INTERFACE_H
#define CARMEN3D_LOCALIZE_INTERFACE_H

#include "localize3d_messages.h"

#ifdef __cplusplus
extern "C" {
#endif



void
carmen_localize_correct_odometry(carmen_base_odometry_message *odometry, 
				 carmen_localize_globalpos_message *globalpos);

void
carmen_localize3d_correct_laser(carmen_robot_laser_message *laser, 
			      carmen_localize_globalpos_message *globalpos);

#ifdef __cplusplus
}
#endif

#endif
// @}
