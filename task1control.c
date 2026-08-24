/*
 * task1control.c
 *
 *  Created on: Jan 27, 2026
 *      Author: justinlesko
 */
#include "task1control.h"
#include "task2interface.h"


void task_1_control(int task_1_state){
	switch(task_1_state) {
		case 0:
			uartprint(task_1_state);
			initiate_encoder();
			task_1_state = state_1_idle;
			break;
		case 1:
			break;
		case 2:

			break;
		case 3:

			break;
	}
}

