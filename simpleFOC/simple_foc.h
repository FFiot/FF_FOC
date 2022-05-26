#ifndef SIMPLE_FOC_H
#define SIMPLE_FOC_H

#include "foc_util.h"

typedef enum{
	FOC_CONTROL_TORQUE 	= 0,
	FOC_CONTROL_SPEED	= 1,
	FOC_CONTROL_ANGLE	= 2,
}FOC_CONTROL_TYPE_T;

typedef struct{
	// mechanic
	float				direction;
	float 				zero_angle;
	float 				pole_pair_num;
	// control
	FOC_CONTROL_TYPE_T	control_type;
    float 				target;
	// filter & PID
	float 				angle_filter_time_const;
	float 				speed_filter_time_const;
	float 				dq_filter_time_const;
	pid_config_t		angle_pid_cfg;
	pid_config_t		speed_pid_cfg;
	pid_config_t		d_pid_cfg;
	pid_config_t		q_pid_cfg;
}foc_config_t;

typedef struct foc_str foc_t;
struct foc_str{
	// mechanic
	float				direction;
	float 				pole_pair_num;
	float 				zero_angle;
	float				full_rotations;
	// control
	FOC_CONTROL_TYPE_T	control_type;
	float 				target;
	// status
	float 				mechanic_angle_prev;
	float				speed_prev;
	// cache
	float				electic_angle;
	float				sine;
	float				cosine;
	// transform
	clarke_t			clarke_input;
	park_t				park_input;
	park_t				park_output;
	clarke_t			clarke_output;
	// filter & PID
	low_pass_filter_t	angle_filter;
	low_pass_filter_t	speed_filter;
	low_pass_filter_t 	d_filter;
	low_pass_filter_t 	q_filter;
	pid_control_t		angle_pid;
	pid_control_t 		speed_pid;
	pid_control_t 		d_pid;
	pid_control_t		q_pid;
	// function
	int (*operate)(foc_t *self, float mechanic_angle, const phase_data_t *current, float time_diff, phase_data_t *output);
};
int foc_init(foc_t *self, const foc_config_t *cfg);

#endif
