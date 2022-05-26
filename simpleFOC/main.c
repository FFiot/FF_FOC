#include "foc_util.h"
#include "simple_foc.h"

static const foc_config_t foc_config = {
	.direction 					= 1.0f,
	.zero_angle 				= _2PI * 25.93f / 360.0f,
	.pole_pair_num 				= 5.0f,
	.control_type				= FOC_CONTROL_SPEED,
	.target						= 100.0f,
	.angle_filter_time_const	= 0.01f,
	.speed_filter_time_const	= 0.01f,
	.dq_filter_time_const		= 0.002f,
	.angle_pid_cfg 				= {0.01f, 0.01f, 0.0f, 0.5f, 0.1f},
	.speed_pid_cfg 				= {0.01f, 0.1f,  0.0f, 0.5f, 0.1f},
	.dq_pid_cfg 				= {0.01f, 0.1f,  0.0f, 0.5f, 0.1f},
};

int main(void){
    foc_t foc = {0};
    foc_init(&foc, &foc_config);
    while(1){
        // read sensor
        float angle = 0.0f;
        phase_data_t current = {0.1f, 0.05f, 0.05f};
        float time_diff = 1e-3;

        phase_data_t output = {0};
        foc.operate(&foc, angle, &current, time_diff, &output);

        // run pwm by output
    }
}