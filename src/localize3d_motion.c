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

//#include <carmen/carmen.h>
#include <carmen_utils/carmen.h>
#include "localize3d_motion.h"

static void install_params(carmen3d_localize_motion_model_t *model,BotParam * b_param)
{
  model->mean_c_d = bot_param_get_double_or_fail(b_param, "localizer.mean_c_d");
  model->mean_c_t = bot_param_get_double_or_fail(b_param, "localizer.mean_c_t");
  model->std_dev_c_d = bot_param_get_double_or_fail(b_param, "localizer.std_dev_c_d");
  model->std_dev_c_t = bot_param_get_double_or_fail(b_param, "localizer.std_dev_c_t");
  model->mean_d_d = bot_param_get_double_or_fail(b_param, "localizer.mean_d_d");
  model->mean_d_t = bot_param_get_double_or_fail(b_param, "localizer.mean_d_t");
  model->std_dev_d_d = bot_param_get_double_or_fail(b_param, "localizer.std_dev_d_d");
  model->std_dev_d_t = bot_param_get_double_or_fail(b_param, "localizer.std_dev_d_t");
  model->mean_t_d =  bot_param_get_double_or_fail(b_param, "localizer.mean_t_d");
  model->mean_t_t =  bot_param_get_double_or_fail(b_param, "localizer.mean_t_t");
  model->std_dev_t_d = bot_param_get_double_or_fail(b_param, "localizer.std_dev_t_d");
  model->std_dev_t_t = bot_param_get_double_or_fail(b_param, "localizer.std_dev_t_t");
}

carmen3d_localize_motion_model_t *carmen_localize_motion_initialize(int argc, char *argv[])
{
  /*carmen3d_localize_motion_model_t *model;

  model = (carmen3d_localize_motion_model_t *)
    calloc(1, sizeof(carmen3d_localize_motion_model_t));
  carmen_test_alloc(model);

  install_params(model, argc, argv);

  return model;*/
  return NULL;
}

carmen3d_localize_motion_model_t *carmen_localize_motion_initialize_from_conf(BotParam * b_param)
{
  carmen3d_localize_motion_model_t *model;

  model = (carmen3d_localize_motion_model_t *)
    calloc(1, sizeof(carmen3d_localize_motion_model_t));
  carmen_test_alloc(model);

  install_params(model, b_param);

  return model;
}

double carmen3d_localize_sample_noisy_downrange(double delta_t, 
					      double delta_theta,
					      carmen3d_localize_motion_model_t 
					      *model)
{
  double downrange_mean, downrange_std_dev;
  double sample;

  downrange_mean = delta_t*model->mean_d_d+delta_theta*model->mean_d_t;
  downrange_std_dev =fabs(delta_t)*model->std_dev_d_d+fabs(delta_theta)*model->std_dev_d_t;

  if (downrange_std_dev < 1e-6)
    return downrange_mean;

  do {
    sample = carmen_gaussian_random(downrange_mean, downrange_std_dev);
  } while (fabs(sample - downrange_mean) > 2*downrange_std_dev);

  return sample; 
}

double carmen3d_localize_sample_noisy_crossrange(double delta_t, 
					       double delta_theta,
					       carmen3d_localize_motion_model_t 
					       *model)
{
  double crossrange_mean, crossrange_std_dev;
  double sample;

  crossrange_mean = delta_t*model->mean_c_d+delta_theta*model->mean_c_t;
  crossrange_std_dev = fabs(delta_t)*model->std_dev_c_d+
    fabs(delta_theta)*model->std_dev_c_t;

  if (crossrange_std_dev < 1e-6)
    return crossrange_mean;

  do {
    sample = carmen_gaussian_random(crossrange_mean, crossrange_std_dev);
  } while (fabs(sample - crossrange_mean) > 2*crossrange_std_dev);

  return sample; 
}

double carmen3d_localize_sample_noisy_turn(double delta_t, double delta_theta,
					 carmen3d_localize_motion_model_t *model)
{
  double turn_mean, turn_std_dev;
  double sample;

  turn_mean = delta_t*model->mean_t_d+delta_theta*model->mean_t_t;
  turn_std_dev = fabs(delta_t)*model->std_dev_t_d+fabs(delta_theta)*model->std_dev_t_t;

  if (turn_std_dev < 1e-6)
    return turn_mean;

  do {
    sample = carmen_gaussian_random(turn_mean, turn_std_dev);
  } while (fabs(sample - turn_mean) > 2*turn_std_dev);

  return sample;
}
