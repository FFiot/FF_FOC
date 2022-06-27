#ifndef FOC_UTIL_H
#define FOC_UTIL_H

#ifndef pdFALSE
#define pdFALSE (0)
#endif

#ifndef pdTRUE
#define pdTRUE  (1)
#endif

#define _2_3	    (0.66666666667f)
#define _1_SQRT3    (0.57735026919f)
#define _2_SQRT3    (1.15470053838f)
#define _SQRT3_2    (0.86602540378f)
#define _2PI 	    (6.28318530718f)
#define _PI			(3.14159265359f)
#define _PI_2 	    (1.57079632679f)

float foc_sin(float x);
float foc_cos(float x);
float value_limit(float x, float low, float high);

typedef struct{
	float u;
	float v;
	float w;
}phase_data_t;

typedef enum{
	SVPWM_V0 = 0,
	SVPWM_V1 = 1,
	SVPWM_V2 = 2,
	SVPWM_V3 = 3,
	SVPWM_V4 = 4,
	SVPWM_V5 = 5,
	SVPWM_V6 = 6,
	SVPWM_V7 = 7,
}SVPWM_VECTOR_T;

typedef struct{
	SVPWM_VECTOR_T 	vector;
	float 			duty;
}svpwm_t;

typedef struct{
	svpwm_t svpwm[3];
}svpwm_sequence_t;

typedef struct{
	float alpha;
	float beta;
}clarke_t;

typedef struct{
	float d;
	float q;
}park_t;

void phase_to_clarke(const phase_data_t *input, clarke_t *clarke);
void clarke_to_phase(const clarke_t *clarke, phase_data_t *output);
void clarke_to_park(const clarke_t *clack, park_t *park, float sine, float cosine);
void park_to_clarke(const park_t *pack, clarke_t *clack, float sine, float cosine);
void park_to_svpwm(const park_t *park, float electic_angle, svpwm_sequence_t *sequence);

typedef struct{
	float P;
	float I;
	float D;
	float limit;
	float change_ramp;
}pid_config_t;

typedef struct pid_control_str pid_control_t;
struct pid_control_str{
	// config
	float P;
	float I;
	float D;
	float limit;
    float change_ramp;
	// status
	float proportional;
	float integral;
	float derivative;
	float error_prev;
	float y_prev;
	// function
	float (*operate)(pid_control_t *self, float x, float time_diff);
};
int pid_control_init(pid_control_t *self, const pid_config_t *cfg);

typedef struct low_pass_filter_str low_pass_filter_t;
struct low_pass_filter_str{
	// config
	float time_const;
	// status
	float y_prev;
	// function
	float (*operate)(low_pass_filter_t *self, float x, float time_diff);
};
int low_pass_filter_init(low_pass_filter_t *self, float time_const);

#endif
