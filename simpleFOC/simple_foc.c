#include <string.h>

#include "foc_util.h"
#include "simple_foc.h"

static int foc_operate(foc_t *self, float mechanic_angle, const phase_data_t *current, float time_diff, phase_data_t *output)
{	
	float mechanic_angle_diff = mechanic_angle - self->mechanic_angle_prev;
	if(mechanic_angle_diff > _PI){
		mechanic_angle_diff = mechanic_angle_diff - _2PI;
	}else if(mechanic_angle_diff < -_PI){
		mechanic_angle_diff = mechanic_angle_diff + _2PI;
	}

	float speed = mechanic_angle_diff / time_diff;
	speed = self->speed_filter.operate(&self->speed_filter, speed, time_diff);
	
	self->electic_angle = (mechanic_angle-self->zero_angle) * self->direction * self->pole_pair_num; 
	self->sine = foc_sin(self->electic_angle);
	self->cosine = foc_cos(self->electic_angle);
	
	float target = self->target;
	switch(self->control_type){
		case FOC_CONTROL_ANGLE:{
			target = self->speed_pid.operate(&self->angle_pid, target-mechanic_angle, time_diff);
		}
		case FOC_CONTROL_SPEED:{
			target = self->speed_pid.operate(&self->speed_pid, target-speed, time_diff);
		}
		case FOC_CONTROL_TORQUE:{
			phase_to_clarke(current, &self->clarke_input);
			clarke_to_park(&self->clarke_input, &self->park_input, self->sine, self->cosine);
			self->park_input.d = self->d_filter.operate(&self->d_filter, self->park_input.d, time_diff);
			self->park_input.q = self->q_filter.operate(&self->q_filter, self->park_input.q, time_diff);
			self->park_output.d = self->d_pid.operate(&self->d_pid, -self->park_input.d, time_diff);
			self->park_output.q = self->q_pid.operate(&self->q_pid, target-self->park_input.q, time_diff);
			park_to_clarke(&self->park_output, &self->clarke_output, self->sine, self->cosine);
			clarke_to_phase(&self->clarke_output, output);
		}break;
		default:{
			output->u = 0.0f;
			output->v = 0.0f;
			output->w = 0.0f;
		}break;
	}
	
	self->mechanic_angle_prev 	= mechanic_angle;
	self->speed_prev			= speed;
	return pdTRUE;
}

int foc_init(foc_t *self, const foc_config_t *cfg)
{
	memset(self, 0, sizeof(*self));
	
    self->direction     = cfg->direction;
    self->pole_pair_num = cfg->pole_pair_num;
    self->zero_angle    = cfg->zero_angle;
    self->control_type  = cfg->control_type;
    self->target        = cfg->target;

    low_pass_filter_init(&self->angle_filter, cfg->angle_filter_time_const);
    low_pass_filter_init(&self->speed_filter, cfg->speed_filter_time_const);
    low_pass_filter_init(&self->d_filter, cfg->dq_filter_time_const);
    low_pass_filter_init(&self->q_filter, cfg->dq_filter_time_const);

    pid_control_init(&self->angle_pid, &cfg->angle_pid_cfg);
    pid_control_init(&self->speed_pid, &cfg->speed_pid_cfg);
    pid_control_init(&self->d_pid, &cfg->d_pid_cfg);
    pid_control_init(&self->q_pid, &cfg->q_pid_cfg);

    self->operate = foc_operate;

	return pdTRUE;
}
