#ifndef THREADS_H
#define THREADS_H

#include <cmsis_os2.h>
#include "LCD_driver.h"
#include <MKL25Z4.H>

#define THREAD_READ_TS_PERIOD_MS (50)  // 1 tick/ms
#define THREAD_UPDATE_SCREEN_PERIOD_MS (50)
#define THREAD_BUS_PERIOD_MS (1)

#define USE_LCD_MUTEX (1)

// Custom stack sizes for larger threads

void Init_Debug_Signals(void);

// Events for sound generation and control
#define EV_PLAYSOUND (1) 
#define EV_SOUND_ON (2)
#define EV_SOUND_OFF (4)

#define EV_REFILL_SOUND_BUFFER  (1)

void Create_OS_Objects(void);


typedef struct {
	int channelNum;
	osMessageQueueId_t * request;
} request_type;

typedef struct {
	int channelNum;
	uint32_t result;
} result_type;

extern result_type res_msg;
extern request_type req_msg;
extern osMessageQueueId_t request_queue, result_queue;
extern osSemaphoreId_t Semaphore_start,Semaphore_print, Semaphore_print_complete;
 
extern osThreadId_t t_Read_TS, t_Read_Accelerometer, t_Sound_Manager, t_US, t_Refill_Sound_Buffer;
extern osMutexId_t LCD_mutex;

#endif // THREADS_H

