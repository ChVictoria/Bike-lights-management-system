#include "Sensortile.h"
#include "motion_fx.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
#include "pcf857x.h"
/*
 * first direction in ax's name = positive direction
 */
#define x 0
#define y 1
#define z 2

#define FOR_BACK_AX y
#define UP_DOWN_AX z
#define ROLL_AX z

#define BALANCE_THRESH 30
#define ACC_STOP_THRESH 0.08
#define ACC_BOOST_THRESH 0.05

#define MAX_TERMINAL_MESSAGE_LEN 30

#define LED_BLINK_PERIOD 1000 //in milliseconds


uint16_t initial_pcf857x_pins_value = 0x00FF;

#define MOTION_CHECKERS_NUMBER 2
#define BUTTON_CHECKERS_NUMBER 5

typedef enum {
	DARK,
	LIGHT
}illumination;

illumination cur_illumination = LIGHT;

typedef enum {
	NO_SIGNAL = 0,
	STOP,
	BOOST,
	LEFT_ON,
	LEFT_OFF,
	RIGHT_ON,
	RIGHT_OFF,
	EMERGENCY_ON,
	EMERGENCY_OFF,
	HEAD_LIGHT_ON,
	HEAD_LIGHT_OFF,
	DAY_LIGHT_ON,
	DAY_LIGHT_OFF
} light_signal;

typedef enum {
	HEAD_LIGHT_PIN = 15,
	DAY_LIGHT_PIN = 13,
	EMERGENCY_STOP_PIN = 14,
	TURN_LEFT_PIN = 12,
	TURN_RIGHT_PIN = 11,
	HEAD_LIGHT_BUTTON_PIN = 3,
	DAY_LIGHT_BUTTON_PIN = 4,
	EMERG_LEFT_BUTTON_PIN = 1,
	EMERG_RIGHT_BUTTON_PIN = 0,
	RIGHT_LEFT_BUTTON_PIN = 2,
} port_ext_pin;



typedef struct  {
	void (*callback)(void);
	TimerHandle_t timer;
}timer_callback_arg;
void StopOffCallback(void);
void DaylightBlinkOffCallback(void);
void DaylightOffCallback(void);
void StopTimerCallback(void const *arg);
timer_callback_arg backlight_delay_arg = {NULL, NULL};
TimerHandle_t backlight_periodic_timer = NULL;
timer_callback_arg daylight_delay_arg = {NULL, NULL};
timer_callback_arg headlight_delay_arg = {NULL, NULL};

/* Motion signal checkers */
light_signal Check_acceleration(MFX_output_t *fused_motion_data);
//light_signal Check_turn(MFX_output_t *fused_motion_data);
light_signal Check_emergency_motion(MFX_output_t *fused_motion_data);


//the closer checker to the beginning of the array the bigger it's priority when multiple signals come
light_signal (* motion_checkers[MOTION_CHECKERS_NUMBER])(MFX_output_t *fused_motion_data) = {&Check_emergency_motion, &Check_acceleration};
light_signal cur_motion_signals[MOTION_CHECKERS_NUMBER] = {0}; //all are NO_SIGNAL

#define num_acc_values 30
float acc_for_back_values[num_acc_values] = {0};
//float acc_up_down_values[num_acc_values] = {0};
int acc_counter = 0;

/* Button signal checkers */
light_signal Check_emergency_button(void);
light_signal Check_head_light_button(void);
light_signal Check_day_light_button(void);
light_signal Check_turn_left_button(void);
light_signal Check_turn_right_button(void);

light_signal (* button_checkers[BUTTON_CHECKERS_NUMBER])(void) = {&Check_emergency_button, &Check_head_light_button, &Check_day_light_button,
                                                         &Check_turn_left_button, &Check_turn_right_button};
light_signal cur_button_signals[BUTTON_CHECKERS_NUMBER] = {0}; //all are NO_SIGNAL

/* Private functions prototypes */
void SignalDelay(int period_ms, timer_callback_arg *arg);
void PeriodicTimerStart(int period_ms, void (*callback)(TimerHandle_t), osTimerId *timer);
void PeriodicTimerStop(osTimerId *timer);

float sum(float arr[num_acc_values]){
	float s = 0;
	for(int i=0; i<num_acc_values; i++){
		s = s + arr[i];
	}
	return s;
}

float acc_avg = 0;
light_signal Check_acceleration(MFX_output_t *fused_motion_data){
	acc_for_back_values[acc_counter] = fused_motion_data->linear_acceleration_6X[FOR_BACK_AX];
	//acc_up_down_values[acc_counter] = fused_motion_data->linear_acceleration_6X[UP_DOWN_AX];
	acc_counter = (acc_counter+1)% num_acc_values;
	//float acc_avg = sum(acc_for_back_values)/(num_acc_values)-sum(acc_up_down_values)/(num_acc_values);
	acc_avg = sum(acc_for_back_values)/(num_acc_values);
	if (acc_avg < -ACC_STOP_THRESH){
		return STOP;
	} else if(acc_avg > ACC_BOOST_THRESH){
		return BOOST;
	} else {
		return NO_SIGNAL;
	}
}

/*light_signal Check_turn(MFX_output_t *fused_motion_data){
	if (fused_motion_data->rotation_9X[UP_DOWN_AX] > TURN_THRESH){
		return RIGHT;
	}else if (fused_motion_data->rotation_9X[UP_DOWN_AX] < -TURN_THRESH){
		return LEFT;
	} else {
		return NO_SIGNAL;
	}

}
this function can be used, but has no sense in current context
*/

light_signal Check_emergency_motion(MFX_output_t *fused_motion_data){
	light_signal sig = NO_SIGNAL;
	if ((fused_motion_data->rotation_9X[ROLL_AX] < -BALANCE_THRESH) ||
		(fused_motion_data->rotation_9X[ROLL_AX] > BALANCE_THRESH)){
				sig = EMERGENCY_ON;
	} else {
		sig = EMERGENCY_OFF;
	}
	if (cur_motion_signals[0]==sig)
		return NO_SIGNAL;
	return sig;
}

light_signal Check_emergency_button(void){
	light_signal signal= NO_SIGNAL;
	pcf857x_Write(EMERG_RIGHT_BUTTON_PIN, 0);
	if (!pcf857x_Read(EMERG_LEFT_BUTTON_PIN)){
		signal = EMERGENCY_ON;
	} else {
		signal = EMERGENCY_OFF;
	}
	pcf857x_Write(EMERG_RIGHT_BUTTON_PIN, 1);
	return signal;
}

light_signal Check_head_light_button(void){
	if (!pcf857x_Read(HEAD_LIGHT_BUTTON_PIN)){
		return HEAD_LIGHT_ON;
	} else {
		return HEAD_LIGHT_OFF;
	}
}

light_signal Check_day_light_button(void){
	 if (!pcf857x_Read(DAY_LIGHT_BUTTON_PIN)){
		 if (cur_button_signals[2] == DAY_LIGHT_ON){
			 return DAY_LIGHT_OFF;
		 }
		 return DAY_LIGHT_ON;
	 }
	 return NO_SIGNAL;
}

light_signal Check_turn_left_button(void){
	light_signal signal= NO_SIGNAL;
	pcf857x_Write(RIGHT_LEFT_BUTTON_PIN, 0);
	if (!pcf857x_Read(EMERG_LEFT_BUTTON_PIN)){
	  signal = LEFT_ON;
	} else {
	  signal = LEFT_OFF;
	}

	pcf857x_Write(RIGHT_LEFT_BUTTON_PIN, 1);
	return signal;
}

light_signal Check_turn_right_button(void){
	pcf857x_Write(RIGHT_LEFT_BUTTON_PIN, 0);
	light_signal signal= NO_SIGNAL;

	if (!pcf857x_Read(EMERG_RIGHT_BUTTON_PIN)){
	  signal = RIGHT_ON;
	} else {
	  signal = RIGHT_OFF;
	}
	pcf857x_Write(RIGHT_LEFT_BUTTON_PIN, 1);
	return signal;
}

void StopOffCallback(void){
	pcf857x_Write(EMERGENCY_STOP_PIN, 0);

}

void DaylightBlinkCallback(TimerHandle_t xTimer){
	pcf857x_Toggle(DAY_LIGHT_PIN);
}

void DaylightBlinkOffCallback(void){
	PeriodicTimerStop(&backlight_periodic_timer);
	pcf857x_Write(DAY_LIGHT_PIN, 1);
}

void DaylightOffCallback(void){
	pcf857x_Write(DAY_LIGHT_PIN, 0);
}

void BacklightBlinkCallback(TimerHandle_t xTimer){
	pcf857x_Toggle(EMERGENCY_STOP_PIN);
}

int Generate_light_signal(light_signal signal){
	switch (signal){
		case STOP:
			if (!backlight_delay_arg.timer && !backlight_periodic_timer){
				pcf857x_Write(EMERGENCY_STOP_PIN, 1);
				backlight_delay_arg.callback = StopOffCallback;
				SignalDelay(1000, &backlight_delay_arg);
				return 1;
			}
			return 0;
		case BOOST:
			if (cur_button_signals[2] == DAY_LIGHT_ON && !backlight_delay_arg.timer){
				PeriodicTimerStart(1, DaylightBlinkCallback, &backlight_periodic_timer);
				backlight_delay_arg.callback = DaylightBlinkOffCallback;
				SignalDelay(1500, &backlight_delay_arg);
				return 1;
			}
			return 0;
		case HEAD_LIGHT_ON:
			if (!headlight_delay_arg.timer){
				pcf857x_Write(HEAD_LIGHT_PIN, 1);
				SignalDelay(1500, &headlight_delay_arg);
				return 1;
			}
			return 0;
		case HEAD_LIGHT_OFF:
			if (!headlight_delay_arg.timer){
				pcf857x_Write(HEAD_LIGHT_PIN, 0);
				SignalDelay(1000, &headlight_delay_arg);
				return 1;
			}
			return 0;
		case DAY_LIGHT_ON:
			if (!daylight_delay_arg.timer){
				pcf857x_Write(DAY_LIGHT_PIN, 1);
				SignalDelay(500, &daylight_delay_arg);
				return 1;
			}
			return 0;
		case DAY_LIGHT_OFF:
			if (!daylight_delay_arg.timer){
				pcf857x_Write(DAY_LIGHT_PIN, 0);
				SignalDelay(500, &daylight_delay_arg);
				return 1;
			}
			return 0;
		case LEFT_ON:
			pcf857x_Write(TURN_LEFT_PIN, 1);
			return 1;
		case RIGHT_ON:
			pcf857x_Write(TURN_RIGHT_PIN, 1);
			return 1;
		case LEFT_OFF:
			pcf857x_Write(TURN_LEFT_PIN, 0);
			return 1;
		case RIGHT_OFF:
			pcf857x_Write(TURN_RIGHT_PIN, 0);
			return 1;
		case EMERGENCY_ON:
			if (backlight_delay_arg.timer){
				StopTimerCallback(backlight_delay_arg.timer);
			}
			if (backlight_periodic_timer){

				PeriodicTimerStop(&backlight_periodic_timer);
			}

			PeriodicTimerStart(500, BacklightBlinkCallback, &backlight_periodic_timer);
			return 1;

		case EMERGENCY_OFF:
			PeriodicTimerStop(&backlight_periodic_timer);
			pcf857x_Write(EMERGENCY_STOP_PIN, 0);
			return 1;
		default:
			return 0;

	}
}


void Send_signal_to_terminal(light_signal signal){
	char message[MAX_TERMINAL_MESSAGE_LEN+1] = {0};
	switch(signal){
		case STOP:
			strncpy( message, "You're slowing down!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case BOOST:
			strncpy( message, "You're speeding up!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case HEAD_LIGHT_ON:
			strncpy( message, "Head light is ON!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case HEAD_LIGHT_OFF:
			strncpy( message, "Head light is OFF!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case DAY_LIGHT_ON:
			strncpy( message, "Day light is ON!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case DAY_LIGHT_OFF:
			strncpy( message, "Day light is OFF!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case LEFT_ON:
			strncpy( message, "Turn left is ON!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case RIGHT_ON:
			strncpy( message, "Turn right is ON!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case LEFT_OFF:
			strncpy( message, "Turn left is OFF!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case RIGHT_OFF:
			strncpy( message, "Turn right is OFF!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case EMERGENCY_ON:
			strncpy( message, "Emergency signal is ON!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case EMERGENCY_OFF:
			strncpy( message, "Emergency signal is OFF!\r\n", MAX_TERMINAL_MESSAGE_LEN);
			break;
		case NO_SIGNAL:
			break;
	}

	Send_msg_BT_terminal(message);
}

void Manage_motion_light_signals(MFX_output_t *fused_motion_data){
	light_signal signal;
	for (int i=0; i<MOTION_CHECKERS_NUMBER; i++){
		signal = motion_checkers[i](fused_motion_data);

		if (signal != NO_SIGNAL){
			int status = Generate_light_signal(signal);
			if (status){
				cur_motion_signals[i] = signal;
				Send_signal_to_terminal(signal);
			}
		}
	}

}

void Manage_button_light_signals(void){
	light_signal signal;
	for (int i=0; i<BUTTON_CHECKERS_NUMBER; i++){
		signal = button_checkers[i]();
		if (cur_button_signals[i] != signal && signal!=NO_SIGNAL){
			int status = Generate_light_signal(signal);
			if (status){
				Send_signal_to_terminal(signal);
				cur_button_signals[i] = signal;
			}
		}
	}
}


void StopTimerCallback(void const *arg) {
	timer_callback_arg *argument = (timer_callback_arg *)pvTimerGetTimerID((TimerHandle_t)arg);
	if (argument->callback){
		argument->callback();
	}
	if  (osTimerStop (argument->timer) != osOK){
	  ALLMEMS2_PRINTF("could not stop timer\n\r");
	}
	if (osTimerDelete (argument->timer) != osOK)  {
	  ALLMEMS2_PRINTF("could not delete timer\n\r");
	}
	argument->timer = NULL;
}

void SignalDelay(int period_ms, timer_callback_arg *arg)
{
	if (!arg->timer) {
		arg->timer = xTimerCreate(
							"delay timer",
							1,
							pdFALSE,
							(void *)arg,
							(TimerCallbackFunction_t)StopTimerCallback);

	}

	if (arg->timer){
	  osTimerStart(arg->timer,  pdMS_TO_TICKS(period_ms));
	}
	else {
		ALLMEMS2_PRINTF("could not create timer\n\r");
	}
}



void PeriodicTimerStart(int period_ms, void (*callback)(TimerHandle_t), osTimerId *timer)
{
  if (!*timer) {
	   *timer = xTimerCreate(
	                        "",         // Name of timer
	                        1,  		// Period of timer (in ticks)
	                        pdTRUE,     // Auto-reload
	                        (void *)1,  // Timer ID
							callback);  // Callback function

  }
  if (*timer){
	  osTimerStart(*timer,  pdMS_TO_TICKS(period_ms));
  }
}

void PeriodicTimerStop(osTimerId *timer)
{
  if (*timer) {
        if  (osTimerStop (*timer) != osOK){
          ALLMEMS2_PRINTF("could not stop timer\n\r");
        }
        if (osTimerDelete (*timer) != osOK)  {
          ALLMEMS2_PRINTF("could not delete timer\n\r");
        }
    *timer = NULL;
  }
}





