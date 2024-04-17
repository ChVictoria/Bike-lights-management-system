#include "Sensortile.h"
#include "motion_fx.h"
#include "TargetFeatures.h"
/*
 * first direction in ax's name = positive direction
 */
#define x 0
#define y 1
#define z 2

#define FOR_BACK_AX y
#define UP_DOWN_AX z
#define RIGHT_LEFT_AX x

#define BALANCE_THRESH 0
#define ACCELR_THRESH 0
#define TURN_THRESH 0

typedef enum {
	STOP,
	BOOST,
	LEFT,
	RIGHT,
	EMERGENCY,
	NO_SIGNAL
} light_signal;

light_signal Check_acceleration(MFX_output_t *fused_motion_data);
light_signal Check_turn(MFX_output_t *fused_motion_data);
light_signal Check_emergency(MFX_output_t *fused_motion_data);

#define CHECK_FUNCS_NUMBER 3
light_signal (* check_funcs_list[CHECK_FUNCS_NUMBER])(MFX_output_t *fused_motion_data) = {&Check_emergency, &Check_turn, &Check_acceleration};



light_signal Check_acceleration(MFX_output_t *fused_motion_data){
	if (fused_motion_data->linear_acceleration_9X[FOR_BACK_AX] < -ACCELR_THRESH){
		return STOP;
	} else if(fused_motion_data->linear_acceleration_9X[FOR_BACK_AX] > ACCELR_THRESH){
		return BOOST;
	} else {
		return NO_SIGNAL;
	}
}

light_signal Check_turn(MFX_output_t *fused_motion_data){
	if (fused_motion_data->rotation_9X[UP_DOWN_AX] > TURN_THRESH){
		return RIGHT;
	}else if (fused_motion_data->rotation_9X[UP_DOWN_AX] < -TURN_THRESH){
		return LEFT;
	} else {
		return NO_SIGNAL;
	}

}

light_signal Check_emergency(MFX_output_t *fused_motion_data){
	if ((fused_motion_data->rotation_9X[FOR_BACK_AX] < BALANCE_THRESH) ||
		(fused_motion_data->rotation_9X[FOR_BACK_AX] > -BALANCE_THRESH)){
			return EMERGENCY;
	} else {
		return NO_SIGNAL;
	}
}

void Generate_light_signal(light_signal signal){
	switch (signal){
		case STOP:
			LedOnTargetPlatform();
			break;
		case BOOST:
			LedOffTargetPlatform();
			break;
		case LEFT:
			break;
		case RIGHT:
			break;
		case EMERGENCY:
			LedOnTargetPlatform();
			break;
		default: break;
	}
}

void Manage_light_signals(MFX_output_t *fused_motion_data){
	light_signal signal;
	for (int i=0; i<CHECK_FUNCS_NUMBER; i++){
		signal = check_funcs_list[i](fused_motion_data);
		if (signal != NO_SIGNAL){
			Generate_light_signal(signal);
			return;
		}
	}
}



