#include "foc_util.h"

#include "arm_math.h"
#define _SIN(x) arm_sin_f32(x)
#define _COS(x) arm_cos_f32(x)

inline float foc_sin(float x)
{
	return _SIN(x);
}

inline float foc_cos(float x)
{
	return _COS(x);
}

inline float value_limit(float x, float low, float high)
{
	if(x > high){
		return high;
	}else if(x < low){
		return low;
	}else{
		return x;
	}
}

inline float change_limit(float x, float prev, float limit)
{
	float change = x - prev;
	if(change > limit){
		return prev + limit;
	}else if(change < -limit){
		return prev-limit;
	}else{
		return x;
	}
}

inline void phase_to_clarke(const phase_data_t *input, clarke_t *clarke)
{
    clarke->alpha = (input->u - input->v/2.0f - input->w/2.0f) * _2_3;
	clarke->beta  = (input->v * _SQRT3_2 - input->w * _SQRT3_2) * _2_3;
}

inline void clarke_to_phase(const clarke_t *clarke, phase_data_t *output)
{
/* Arduino-FOC\src\BLDCMotor.cpp Line 507. */
	output->u = clarke->alpha;
	output->v = -0.5f * clarke->alpha + _SQRT3_2 * clarke->beta;
	output->w = -0.5f * clarke->alpha - _SQRT3_2 * clarke->beta;
}

void svpwm_output(float x, float electic_angle, phase_data_t *output)
{
/*
	注：没有跳过程，首先计算各个向量的占空比，然后计算 PWM 输出。
	1、输出力矩不均匀，需自行调整。
	   在60°(6个向量的位置)可以输出最大力矩，在30°(6个向量的中间位置，输出最小)。
	2、使用V0，不使用V7，保证上桥自举电容充电，同时节能。
	3、3 步：V0 -> [V1, V2, V4] -> [V3, V6, V5]
	   6 步：V0 -> [V1, V2, V4] -> [V3, V6, V5] -> [V3, V6, V5] -> [V1, V2, V4] -> V0。
	4、UPandDOWN模式，可以简单地由 6 步变为 3 步。
*/
	svpwm_sequence_t sequence;
	if(x <= 0.0f){
		sequence.svpwm[0].vector 		= SVPWM_V0;
		sequence.svpwm[0].duty			= 1.0f;
		sequence.svpwm[1].vector 		= SVPWM_V0;
		sequence.svpwm[1].duty			= 0.0f;
		sequence.svpwm[2].vector 		= SVPWM_V0;
		sequence.svpwm[2].duty			= 0.0f;
		return;
	}
	if(x > 1.0f){
		x = 1.0f;
	}
	
	float angle = electic_angle / _PI * 180.0f;
	while(angle >= 360.0f){
		angle -= 360.0f;
	}
	while(angle < 0.0f){
		angle += 360.0f;
	}

	sequence.svpwm[0].vector 		= SVPWM_V0;
	sequence.svpwm[1].duty			= 0.0f;
	if(angle < 60.0f){ // sector1
		angle = (angle - 0.0f) / 180.0f * _PI;
		sequence.svpwm[1].vector 	= SVPWM_V1;
		sequence.svpwm[1].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[2].vector 	= SVPWM_V3;
		sequence.svpwm[2].duty 		= foc_sin(angle) * _2_SQRT3;
	}else if(angle < 120.0f){ // sector2
		angle = (angle - 60.0f) / 180.0f * _PI;
		sequence.svpwm[2].vector 	= SVPWM_V3;
		sequence.svpwm[2].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[1].vector 	= SVPWM_V2;
		sequence.svpwm[1].duty 		= foc_sin(angle) * _2_SQRT3;
	}else if(angle < 180.0f){ // sector3
		angle = (angle - 120.0f) / 180.0f * _PI;
		sequence.svpwm[1].vector 	= SVPWM_V2;
		sequence.svpwm[1].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[2].vector 	= SVPWM_V6;
		sequence.svpwm[2].duty 		= foc_sin(angle) * _2_SQRT3;
	}else if(angle < 240.0f){	// sector4
		angle = (angle - 180.0f) / 180.0f * _PI;
		sequence.svpwm[2].vector 	= SVPWM_V6;
		sequence.svpwm[2].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[1].vector 	= SVPWM_V4;
		sequence.svpwm[1].duty 		= foc_sin(angle) * _2_SQRT3;
	}else if(angle < 300.0f){	// sector5
		angle = (angle - 240.0f) / 180.0f * _PI;
		sequence.svpwm[1].vector 	= SVPWM_V4;
		sequence.svpwm[1].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[2].vector 	= SVPWM_V5;
		sequence.svpwm[2].duty 		= foc_sin(angle) * _2_SQRT3;
	}else if(angle < 360.0f){	// sector6
		angle = (angle - 300.0f) / 180.0f * _PI;
		sequence.svpwm[2].vector 	= SVPWM_V5;
		sequence.svpwm[2].duty 		= foc_cos(angle) - foc_sin(angle) * _1_SQRT3;
		sequence.svpwm[1].vector 	= SVPWM_V1;
		sequence.svpwm[1].duty 		= foc_sin(angle) * _2_SQRT3;
	}
	sequence.svpwm[1].duty 		= x * sequence.svpwm[1].duty;
	sequence.svpwm[2].duty 		= x * sequence.svpwm[2].duty;
	
	float sum = sequence.svpwm[1].duty + sequence.svpwm[2].duty;
	if(sum > 1.0f){
		sequence.svpwm[0].duty 	= 0.0f;
		sequence.svpwm[1].duty 	= sequence.svpwm[1].duty / sum;
		sequence.svpwm[2].duty 	= sequence.svpwm[2].duty / sum;
	}else{
		sequence.svpwm[0].duty 	= 1.0f - sum;
	}

	float T1 = 0.0f, T2 = 0.0f;
	T1 = sequence.svpwm[1].duty + sequence.svpwm[2].duty;
	T2 = sequence.svpwm[2].duty;
	if(0X01 == sequence.svpwm[1].vector){
		output->u = T1;
		if(0X02 & sequence.svpwm[2].vector){
			output->v = T2;
			output->w = 0.0f;
		}else if(0X04 & sequence.svpwm[2].vector){
			output->v = 0.0f;
			output->w = T2;
		}
	}else if(0X02 == sequence.svpwm[1].vector){
		output->v = T1;
		if(0X01 & sequence.svpwm[2].vector){
			output->u = T2;
			output->w = 0.0f;
		}else if(0X04 & sequence.svpwm[2].vector){
			output->u = 0.0f;
			output->w = T2;
		}
	}else if(0X04 == sequence.svpwm[1].vector){
		output->w = T1;
		if(0X01 & sequence.svpwm[2].vector){
			output->u = T2;
			output->v = 0.0f;
		}else if(0X02 & sequence.svpwm[2].vector){
			output->u = 0.0f;
			output->v = T2;
		}
	}
}

inline void clarke_to_park(const clarke_t *clarke, park_t *pack, float sine, float cosine)
{
/* Arduino-FOC\src\common\base_classes\CurrentSense.cpp Line 72. */
	pack->d = +clarke->alpha * cosine + clarke->beta * sine;
	pack->q = -clarke->alpha * sine + clarke->beta * cosine;
}

inline void park_to_clarke(const park_t *park, clarke_t *clarke, float sine, float cosine)
{
/* Arduino-FOC\src\BLDCMotor.cpp Line 500. */
	clarke->alpha = park->d * cosine - park->q * sine;
	clarke->beta  = park->d * sine + park->q * cosine;
}

static float pid_control_operate(pid_control_t *self, float error, float time_diff)
{
/* Arduino-FOC\src\common\pid.cpp Line 17. */
	if(time_diff <= 0 || time_diff > 0.5f){
		time_diff = 1e-3f;
	}
	self->proportional = self->P * error;
	self->integral = self->integral + self->I * (error + self->error_prev) * time_diff;
	self->integral = value_limit(self->integral, -self->limit, self->limit);
	self->derivative = self->D * (error - self->error_prev) / time_diff;
	float y = self->proportional + self->integral + self->derivative;
	y = value_limit(y, -self->limit, self->limit);
	if(self->change_ramp > 0){
		y = change_limit(y, self->y_prev, self->change_ramp);
	}
	self->error_prev = error;
	self->y_prev = y;
	return y;
}

int pid_control_init(pid_control_t *self, const pid_config_t *cfg)
{
    self->P = cfg->P;
    self->I = cfg->I;
    self->D = cfg->D;
    self->limit = cfg->limit;
    self->change_ramp = cfg->change_ramp;
    self->operate = pid_control_operate;
    return pdTRUE;
}

static float low_pass_filter_operate(low_pass_filter_t *self, float x, float time_diff)
{
	/* Arduino-FOC\src\common\lowpass_filter.cpp Line 10. */
	if (time_diff <= 0.0f){
		time_diff = 1e-3f;
	}else if(time_diff > 0.3f){
		self->y_prev = x;
		return x;
	}
	
	float alpha = self->time_const/(self->time_const + time_diff);
	self->y_prev = self->y_prev * alpha + x * (1 - alpha);
	return self->y_prev;
}

int low_pass_filter_init(low_pass_filter_t *self, float time_const)
{
	self->time_const  = time_const;
	self->y_prev = 0.0f;
	self->operate = low_pass_filter_operate;
	return pdTRUE;
}
