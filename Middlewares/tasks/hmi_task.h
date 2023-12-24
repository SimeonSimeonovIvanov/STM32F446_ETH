/*
 * hmi_task.h
 *
 *  Created on: Dec 27, 2020
 *      Author: Simo
 */

#ifndef TASKS_HMI_TASK_H_
#define TASKS_HMI_TASK_H_

typedef struct
{
	uint8_t *lpRS485PortLed[2];
	uint8_t *arrInput, *arrOutput;
	uint16_t arrInputLen, arrOutputLen;
} HMI_TASK_ARG, *LP_HMI_TASK_ARG;

void HmiTask( void *argument );
void hmiInitLeds( void );
void hmiSetLeds( uint8_t *lpLeds, uint8_t index, uint8_t size );
void hmiSetAddress( uint8_t address );
void hmiUpdateLeds( void );

#endif /* TASKS_HMI_TASK_H_ */
