#include <MKL25Z4.H>
#include <stdio.h>
#include <stdint.h>

#include "gpio_defs.h"
#include "debug.h"
#include <cmsis_os2.h>
#include "control.h"

#include "timers.h"
#include "delay.h"
#include "LEDs.h"
#include "UI.h"
#include "threads.h"

#include "FX.h"

volatile int16_t g_duty_cycle=5;  // global to give debugger access

volatile int g_enable_flash=1;
volatile int g_peak_set_current=FLASH_CURRENT_MA; // Peak flash current
volatile int g_flash_duration=FLASH_DURATION_MS;
volatile int g_flash_period=FLASH_PERIOD_MS; 

volatile int g_enable_control=1;
volatile int g_set_current=0; // Default starting LED current

volatile int g_measured_current;
volatile int error;
volatile 	uint32_t count1,count2;
	static int i=1;
	static result_type result_message;
	static request_type request_message;
	PREV_CONV previous_conversion = priority;
int32_t pGain_8 = PGAIN_8; // proportional gain numerator scaled by 2^8
  volatile int py[960];
	volatile int i_set[960];
SPid plantPID = {0, // dState
	0, // iState
	LIM_DUTY_CYCLE, // iMax
	-LIM_DUTY_CYCLE, // iMin
	P_GAIN_FL, // pGain
	I_GAIN_FL, // iGain
	D_GAIN_FL  // dGain
};

SPidFX plantPID_FX = {FL_TO_FX(0), // dState
	FL_TO_FX(0), // iState
	FL_TO_FX(LIM_DUTY_CYCLE), // iMax
	FL_TO_FX(-LIM_DUTY_CYCLE), // iMin
	P_GAIN_FX, // pGain
	I_GAIN_FX, // iGain
	D_GAIN_FX  // dGain
};

void Init_ADC_HBLED(void) {
#if USE_ADC_FOR_BUCK
	// Configure ADC to read Ch 8 (FPTB 0)
	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
	ADC0->CFG1 = 0x0C; // 16 bit
	//	ADC0->CFG2 = ADC_CFG2_ADLSTS(3);
	ADC0->SC2 = ADC_SC2_REFSEL(0);

#if USE_ADC_HW_TRIGGER
	// Enable hardware triggering of ADC
	ADC0->SC2 |= ADC_SC2_ADTRG(1);
	// Select triggering by TPM0 Overflow
	SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8) | SIM_SOPT7_ADC0ALTTRGEN_MASK;
	// Select input channel 
	ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
	ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
#endif // USE_ADC_HW_TRIGGER

#if USE_ADC_INTERRUPT 
	// enable ADC interrupt
	ADC0->SC1[0] |= ADC_SC1_AIEN(1);

	// Configure NVIC for ADC interrupt
	NVIC_SetPriority(ADC0_IRQn, 128); // 0, 64, 128 or 192
	NVIC_ClearPendingIRQ(ADC0_IRQn); 
	NVIC_EnableIRQ(ADC0_IRQn);	
#endif // USE_ADC_INTERRUPT
#endif // USE_ADC_FOR_BUCK
}

float UpdatePID(SPid * pid, float error, float position){
	float pTerm, dTerm, iTerm;

	// calculate the proportional term
	pTerm = pid->pGain * error;
	// calculate the integral state with appropriate limiting
	pid->iState += error;
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	iTerm = pid->iGain * pid->iState; // calculate the integral term
	dTerm = pid->dGain * (position - pid->dState);
	pid->dState = position;

	return pTerm + iTerm - dTerm;
}

FX16_16 UpdatePID_FX(SPidFX * pid, FX16_16 error_FX, FX16_16 position_FX){
	FX16_16 pTerm, dTerm, iTerm, diff, ret_val;

	// calculate the proportional term
	pTerm = Multiply_FX(pid->pGain, error_FX);

	// calculate the integral state with appropriate limiting
	pid->iState = Add_FX(pid->iState, error_FX);
	if (pid->iState > pid->iMax) 
		pid->iState = pid->iMax;
	else if (pid->iState < pid->iMin) 
		pid->iState = pid->iMin;
	
	iTerm = Multiply_FX(pid->iGain, pid->iState); // calculate the integral term
	diff = Subtract_FX(position_FX, pid->dState);
	dTerm = Multiply_FX(pid->dGain, diff);
	pid->dState = position_FX;

	ret_val = Add_FX(pTerm, iTerm);
	ret_val = Subtract_FX(ret_val, dTerm);
	return ret_val;
}

void Control_HBLED(void) {
	uint16_t res;
	FX16_16 change_FX, error_FX;
	osStatus_t sem;
	static int samples=0;
	static int position = 0;
	FPTB->PSOR = MASK(DBG_CONTROLLER);
	static int strt_sampl =0;
#if USE_ADC_INTERRUPT
	// already completed conversion, so don't wait
#else
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
		; // wait until end of conversion
#endif
	res = ADC0->R[0];

	g_measured_current = (res*1500)>>16; // Extra Credit: Make this code work: V_REF_MV*MA_SCALING_FACTOR)/(ADC_FULL_SCALE*R_SENSE)

	if (g_enable_control) {
		switch (control_mode) {
			case OpenLoop:
					// don't do anything!
				break;
			case BangBang:
				if (g_measured_current < g_set_current)
					g_duty_cycle = LIM_DUTY_CYCLE;
				else
					g_duty_cycle = 0;
				break;
			case Incremental:
				if (g_measured_current < g_set_current)
					g_duty_cycle += INC_STEP;
				else
					g_duty_cycle -= INC_STEP;
				break;
			case Proportional:
				g_duty_cycle += (pGain_8*(g_set_current - g_measured_current))/256; //  - 1;
			break;
			case PID:
				g_duty_cycle += UpdatePID(&plantPID, g_set_current - g_measured_current, g_measured_current);
				break;
			case PID_FX:
				error_FX = INT_TO_FX(g_set_current - g_measured_current);
				change_FX = UpdatePID_FX(&plantPID_FX, error_FX, INT_TO_FX(g_measured_current));
				g_duty_cycle += FX_TO_INT(change_FX);
			break;
			default:
				break;
		}
	
		// Update PWM controller with duty cycle
		if (g_duty_cycle < 0)
			g_duty_cycle = 0;
		else if (g_duty_cycle > LIM_DUTY_CYCLE)
			g_duty_cycle = LIM_DUTY_CYCLE;
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	}
	//Storing current values
		sem= osSemaphoreAcquire(Semaphore_start, 0);//semaphore to store
		if(sem == osOK && strt_sampl == 0){
			strt_sampl++;
			}
		if (strt_sampl==1){
					position ++;
			i_set[position-1]=g_set_current;
			py[position-1]= g_measured_current;
			if (position == 960){
					position = 0;
					osSemaphoreRelease(Semaphore_print); //semaphore to print
					FPTB->PCOR = MASK(DBG_IRQTPM);
				strt_sampl++;
			}
			}
		sem= osSemaphoreAcquire(Semaphore_print_complete, 0);//semaphore signifying plotting complete
		if(sem == osOK){
			strt_sampl =0;
			}		
	FPTB->PCOR = MASK(DBG_CONTROLLER);
}

#if USE_ADC_INTERRUPT
void ADC0_IRQHandler() {
	osStatus_t result,sem;
	
	FPTB->PSOR = MASK(DBG_IRQ_ADC);

	static int next_state=0;
	static int counter_avg = 0;
	static int division = 0;
	uint16_t resi=0;
	uint16_t time_left=0;
	int current =0;
	switch (next_state) {
		case 0:
			//run the HBLed
			Control_HBLED();
			if (osMessageQueueGetCount(request_queue)!=0) {
				//check if there is enough time left to perform a queued request
				count1=TPM0->CNT;
							count2=TPM0->CNT;
				if (count1>count2){
									time_left=(500+count2)*21;
								}
								else {
									time_left=(500-count2)*21;
								}
				result = osMessageQueueGet(request_queue,&request_message, NULL,0);
				if (result == osOK && request_message.channelNum==LCD_TS_YU_CHANNEL && time_left>2800){
					next_state = 1;
				}
				if (result == osOK && request_message.channelNum==LCD_TS_XL_CHANNEL && time_left>2800){
					next_state = 2;
				}
			} else {
				// stay in this state, awaiting a start command
			}
			break;
		case 1:
						//set to software trigger and perform YU conversion
									ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK; 
									ADC0->SC2 |= ADC_SC2_ADTRG(0);
									
									#if USE_ADC_FOR_TOUCHSCREEN
								  // Read inputs
									ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
									ADC0->SC1[0] |= LCD_TS_YU_CHANNEL; // start conversion on channel Y
									#endif
									
									next_state = 3;
									
       break;
		case 2:
		//set to software trigger and perform XL converison
									ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK; 
									ADC0->SC2 |= ADC_SC2_ADTRG(0);
									
									#if USE_ADC_FOR_TOUCHSCREEN
									// Read inputs
									ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
									ADC0->SC1[0] |= LCD_TS_XL_CHANNEL; // start conversion on channel YU
									#endif
									next_state = 3;
								
			break;
		case 3:
			//store result and set to hardware trigger
						result_message.result = ADC0->R[0];
						result_message.channelNum=request_message.channelNum;
			
						ADC0->SC1[0] &= ~ADC_SC1_ADCH_MASK;
						ADC0->SC1[0] |= ADC_SC1_ADCH(ADC_SENSE_CHANNEL);
						ADC0->SC2 &= ~ADC_SC2_ADTRG_MASK;
						ADC0->SC2 |= ADC_SC2_ADTRG(1);
						next_state = 4;
			break;
				case 4:
						//put message onto queue
						osMessageQueuePut(*request_message.request,&result_message, NULL,0);
						next_state = 0;
			break;
		default: next_state = 0;
			break;
	}
FPTB->PCOR = MASK(DBG_IRQ_ADC);
}
#endif

void Set_DAC(unsigned int code) {
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Set_DAC_mA(unsigned int current) {
	unsigned int code = MA_TO_DAC_CODE(current);
	// Force 16-bit write to DAC
	uint16_t * dac0dat = (uint16_t *)&(DAC0->DAT[0].DATL);
	*dac0dat = (uint16_t) code;
}

void Init_DAC_HBLED(void) {
  // Enable clock to DAC and Port E
	SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	
	// Select analog for pin
	PORTE->PCR[DAC_POS] &= ~PORT_PCR_MUX_MASK;
	PORTE->PCR[DAC_POS] |= PORT_PCR_MUX(0);	
		
	// Disable buffer mode
	DAC0->C1 = 0;
	DAC0->C2 = 0;
	
	// Enable DAC, select VDDA as reference voltage
	DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
	Set_DAC(0);
}

void Update_Set_Current(void) {
	static int delay=0;

	if (delay == 0)					// Just for initialization from global
		delay = g_flash_period;
	
	if (g_enable_flash){
		delay--;
		if (delay == 15){
			osSemaphoreRelease(Semaphore_start);
			FPTB->PSOR = MASK(DBG_IRQTPM);
		}
		if (delay == g_flash_duration) { // assumes runs every 1 ms
			g_set_current = g_peak_set_current;
			Set_DAC_mA(g_set_current);
		} else  if (delay == 0) {
			delay = g_flash_period;
			g_set_current = 0;
			Set_DAC_mA(g_set_current);
		}
	}
}

void Init_Buck_HBLED(void) {
	Init_DAC_HBLED();
	Init_ADC_HBLED();
	
	// Configure driver for buck converter
	// Set up PTE31 to use for SMPS with TPM0 Ch 4
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
	PORTE->PCR[31]  &= PORT_PCR_MUX(7);
	PORTE->PCR[31]  |= PORT_PCR_MUX(3);
	PWM_Init(TPM0, PWM_HBLED_CHANNEL, PWM_PERIOD, g_duty_cycle, 0, 0);
	
}

// Handler functions (callbacks)
// Default handlers
void Control_OnOff_Handler (UI_FIELD_T * fld, int v) {
	if (fld->Val != NULL) {
		if (v > 0) {
			*fld->Val = 1;
		} else {
			*fld->Val = 0;
		}
	}
}

void Control_IntNonNegative_Handler (UI_FIELD_T * fld, int v) {
	int n;
	if (fld->Val != NULL) {
		n = *fld->Val + v/16;
		if (n < 0) {
			n = 0;
		}
		*fld->Val = n;
	}
}

void Control_DutyCycle_Handler(UI_FIELD_T * fld, int v) {
	int dc;
	if (fld->Val != NULL) {
		dc = g_duty_cycle + v/16;
		if (dc < 0)
			dc = 0;
		else if (dc > LIM_DUTY_CYCLE)
			dc = LIM_DUTY_CYCLE;
		*(fld->Val) = dc;
		PWM_Set_Value(TPM0, PWM_HBLED_CHANNEL, g_duty_cycle);
	}
}

void Control_Sample_Handler (UI_FIELD_T * fld, int v) {
	if (fld->Val != NULL) {
		if (*fld->Val == 0) {
		if (v > 0) {
			*fld->Val = 1;
		} else if (v < 0) {
			*fld->Val = 0;
		}
	}
		if (*fld->Val == 1) {
		if (v > 60) {
			*fld->Val = 2;
		} else if (v < 0) {
			*fld->Val = 0;
		}
	}
		if (*fld->Val == 2) {
		if (v > 60) {
			*fld->Val = 2;
		} else if (v < 60) {
			*fld->Val = 1;
		}
	}
}
}