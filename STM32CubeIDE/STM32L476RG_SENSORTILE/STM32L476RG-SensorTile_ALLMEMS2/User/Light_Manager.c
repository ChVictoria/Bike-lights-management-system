#include "Sensortile.h"
#include "motion_fx.h"
#include "main.h"
#include "cmsis_os.h"
#include "timers.h"
/*
 * first direction in ax's name = positive direction
 */
#define x 0
#define y 1
#define z 2

#define FOR_BACK_AX y
#define ROLL_AX z


#define BALANCE_THRESH 50
#define ACCELR_THRESH 0.5
#define TURN_THRESH 0.1

#define MAX_TERMINAL_MESSAGE_LEN 30

#define LED_BLINK_PERIOD 1000 //in milliseconds
osTimerId emerg_timer;

typedef enum {
	DARK,
	LIGHT
}illumination;

illumination cur_illumination = LIGHT;

typedef enum {
	NO_SIGNAL,
	STOP,
	BOOST,
	LEFT_ON,
	LEFT_OFF,
	RIGHT_ON,
	RIGHT_OFF,
	EMERGENCY_ON,
	EMERGENCY_OFF,
	EVENLY
} light_signal;

light_signal cur_signal = NO_SIGNAL;

light_signal Check_acceleration(MFX_output_t *fused_motion_data);
//light_signal Check_turn(MFX_output_t *fused_motion_data);
light_signal Check_emergency(MFX_output_t *fused_motion_data);

#define CHECK_FUNCS_NUMBER 2
//the closer checker to the beginning of the array the bigger it's priority when multiple signals come
light_signal (* checkers_list[CHECK_FUNCS_NUMBER])(MFX_output_t *fused_motion_data) = {&Check_emergency, &Check_acceleration};



light_signal Check_acceleration(MFX_output_t *fused_motion_data){
	if (fused_motion_data->linear_acceleration_9X[FOR_BACK_AX] < -ACCELR_THRESH){
		return STOP;
	} else if(fused_motion_data->linear_acceleration_9X[FOR_BACK_AX] > ACCELR_THRESH){
		return BOOST;
	} else {
		return EVENLY;
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

light_signal Check_emergency(MFX_output_t *fused_motion_data){
	if ((fused_motion_data->rotation_9X[ROLL_AX] < -BALANCE_THRESH) ||
		(fused_motion_data->rotation_9X[ROLL_AX] > BALANCE_THRESH)){
			return EMERGENCY_ON;
	} else {
		if (cur_signal == EMERGENCY_ON){
			return EMERGENCY_OFF;
		} else {
			return NO_SIGNAL;
		}
	}
}

void Generate_light_signal(light_signal signal){
	switch (signal){
		case STOP:
			switch (cur_illumination){
				case LIGHT:
					BSP_LED_On(LED1);
					break;
				case DARK:
					//todo led blinking with higher then normal duty cycle
					break;
			}
			osDelay(1000);
			break;
		case BOOST:
			switch (cur_illumination){
				case LIGHT:
					BSP_LED_Off(LED1);
					break;
				case DARK:
					//todo led blinking with lower then normal duty cycle
					osDelay(1000);
					break;
			}
			break;
		case EVENLY:
			switch (cur_illumination){
				case LIGHT:
					BSP_LED_Off(LED1);
					break;
				case DARK:
					//todo led blinking with normal duty cycle
					break;
			}
			break;
		case LEFT_ON:
			break;
		case RIGHT_ON:
			break;
		case LEFT_OFF:
			break;
		case RIGHT_OFF:
			break;
		case EMERGENCY_ON:
			//BSP_LED_On(LED1);
			LedBlinkPeriodicStart();
			//osDelay(1000);
			//BSP_LED_Off(LED1);
			break;
		case EMERGENCY_OFF:
			//BSP_LED_Off(LED1);
			LedBlinkPeriodicStop();
			break;

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
		case EVENLY:
			strncpy( message, "Your velocity is even!\r\n", MAX_TERMINAL_MESSAGE_LEN);
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
	}

	Send_msg_BT_terminal(message);
}

void Manage_light_signals(MFX_output_t *fused_motion_data){
	light_signal signal;
	for (int i=0; i<CHECK_FUNCS_NUMBER; i++){
		signal = checkers_list[i](fused_motion_data);
		if (signal != NO_SIGNAL){
			if (cur_signal != signal){
				Generate_light_signal(signal);
				Send_signal_to_terminal(signal);
			}
			break;
		}
	}


	cur_signal = signal;
}

void EmergTimerCallback(TimerHandle_t xTimer) {
	BSP_LED_Toggle(LED1);
}

void LedBlinkPeriodicStart(void)
{
  LedOnTargetPlatform();
  if (!emerg_timer) {
	  emerg_timer = xTimerCreate(
	                        "Emergency signal timer",        // Name of timer
	                        1,  							// Period of timer (in ticks)
	                        pdTRUE,                        // Auto-reload
	                        (void *)1,                    // Timer ID
							EmergTimerCallback);         // Callback function

  }
  if (emerg_timer){
	  osTimerStart(emerg_timer,  pdMS_TO_TICKS( 1000 ));
  }
}

void LedBlinkPeriodicStop(void)
{
  LedOffTargetPlatform();
  if (emerg_timer) {
        if  (osTimerStop (emerg_timer) != osOK){
          ALLMEMS2_PRINTF("could not stop led emergency timer\n\r");
        }
        if (osTimerDelete (emerg_timer) != osOK)  {
          ALLMEMS2_PRINTF("could not delete led emergency timer\n\r");
        }
    emerg_timer = NULL;
  }
}





