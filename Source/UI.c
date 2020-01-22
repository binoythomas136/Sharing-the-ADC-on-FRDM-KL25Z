#include "UI.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "LCD.h"
#include "colors.h"
#include "ST7789.h"
#include "T6963.h"
#include "font.h"
#include "control.h"
#include "FX.h"
#include "timers.h"
#include "threads.h"
#include "debug.h"

volatile int g_sample_type=0;
// Green fields = read/write
// Orange fields = read-only
UI_FIELD_T Fields[] = {
	
	{"Duty Cycle  ", "ct", "", (volatile int *)&g_duty_cycle, NULL, {0,8}, 
	&green, &black, 1, 0, 0, 0,Control_DutyCycle_Handler},
	{"Enable Ctlr ", "", "", (volatile int *)&g_enable_control, NULL, {0,9}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"Enable Flash", "", "", (volatile int *)&g_enable_flash, NULL, {0,10}, 
	&green, &black, 1, 0, 0, 0, Control_OnOff_Handler},	
	{"T_flash_prd ", "ms", "", (volatile int *)&g_flash_period, NULL, {0,11}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},		
	{"T_flash_on  ", "ms", "", (volatile int *)&g_flash_duration, NULL, {0,12}, 
	&green, &black,  1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set       ", "mA", "", (volatile int *)&g_set_current, NULL, {0,13}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_set_peak  ", "mA", "", (volatile int *)&g_peak_set_current, NULL, {0,14}, 
	&green, &black, 1, 0, 0, 0, Control_IntNonNegative_Handler},
	{"I_measured  ", "mA", "", (volatile int *)&g_measured_current, NULL, {0,15}, 
	&orange, &black, 1, 0, 1, 1, NULL},
};

UI_SLIDER_T Slider = {
	0, {0,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {UI_SLIDER_WIDTH-1,LCD_HEIGHT-1}, 
	{119,LCD_HEIGHT-UI_SLIDER_HEIGHT}, {119,LCD_HEIGHT-1}, &white, &dark_gray, &light_gray
};

int UI_sel_field = -1;
osStatus_t sem;

void UI_Update_Field_Values (UI_FIELD_T * f, int num) {
	int i;
	for (i=0; i < num; i++) {
		snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
		f[i].Updated = 1;
	}	
}

void UI_Update_Volatile_Field_Values(UI_FIELD_T * f) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		if (f[i].Volatile) {
			snprintf(f[i].Buffer, sizeof(f[i].Buffer), "%s%4d %s", f[i].Label, f[i].Val? *(f[i].Val) : 0, f[i].Units);
			f[i].Updated = 1;
		}
	}
}

void UI_Draw_Fields(UI_FIELD_T * f, int num){
	int i;
	COLOR_T * bg_color;
	for (i=0; i < num; i++) {
		if ((f[i].Updated) || (f[i].Volatile)) { // redraw updated or volatile fields
			f[i].Updated = 0;
			if ((f[i].Selected) && (!f[i].ReadOnly)) {
				bg_color = &white;
			} else {
				bg_color = f[i].ColorBG;
			}
			LCD_Text_Set_Colors(f[i].ColorFG, bg_color);
			LCD_Text_PrintStr_RC(f[i].RC.Y, f[i].RC.X, f[i].Buffer);
		}
	}
}

void UI_Draw_Slider(UI_SLIDER_T * s) {
	static int initialized=0;
	
	if (!initialized) {
		LCD_Fill_Rectangle(&s->UL, &s->LR, s->ColorBG);
		initialized = 1;
	}
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorBG); // Erase old bar
	
	s->BarUL.Y = s->UL.Y;
	s->BarLR.Y = s->LR.Y;
	s->BarUL.X = (s->LR.X - s->UL.X)/2 + s->Val;
	s->BarLR.X = s->BarUL.X + UI_SLIDER_BAR_WIDTH/2;
	s->BarUL.X -= UI_SLIDER_BAR_WIDTH/2;
	LCD_Fill_Rectangle(&s->BarUL, &s->BarLR, s->ColorFG); // Draw new bar
}

int UI_Identify_Field(PT_T * p) {
	int i, t, b, l, r;

	if ((p->X >= LCD_WIDTH) || (p->Y >= LCD_HEIGHT)) {
		return -1;
	}

	if ((p->X >= Slider.UL.X) && (p->X <= Slider.LR.X) 
		&& (p->Y >= Slider.UL.Y) && (p->Y <= Slider.LR.Y)) {
		return UI_SLIDER;
	}
  for (i=0; i<UI_NUM_FIELDS; i++) {
		l = COL_TO_X(Fields[i].RC.X);
		r = l + strlen(Fields[i].Buffer)*CHAR_WIDTH;
		t = ROW_TO_Y(Fields[i].RC.Y);
		b = t + CHAR_HEIGHT-1;
		
		if ((p->X >= l) && (p->X <= r) 
			&& (p->Y >= t) && (p->Y <= b) ) {
			return i;
		}
	}
	return -1;
}

void UI_Update_Field_Selects(int sel) {
	int i;
	for (i=0; i < UI_NUM_FIELDS; i++) {
		Fields[i].Selected = (i == sel)? 1 : 0;
	}
}

void UI_Process_Touch(PT_T * p) {  // Called by Thread_Read_TS
	int i;
	
	i = UI_Identify_Field(p);
	if (i == UI_SLIDER) {
		Slider.Val = p->X - (Slider.LR.X - Slider.UL.X)/2; // Determine slider position (value)
		if (UI_sel_field >= 0) {  // If a field is selected...
			if (Fields[UI_sel_field].Val != NULL) {
				if (Fields[UI_sel_field].Handler != NULL) {
					(*Fields[UI_sel_field].Handler)(&Fields[UI_sel_field], Slider.Val); // Have the field handle the new slider value
				}
				UI_Update_Field_Values(&Fields[UI_sel_field], 1);
			}
		}
	} else if (i>=0) {
		if (!Fields[i].ReadOnly) { // Can't select (and modify) a ReadOnly field
			UI_sel_field = i;
			UI_Update_Field_Selects(UI_sel_field);
			UI_Update_Field_Values(Fields, UI_NUM_FIELDS);
			Slider.Val = 0; // return to slider to zero if a different field is selected
		}
	} 
}

void UI_Draw_Screen(int first_time) { // Called by Thread_Update_Screen
	
	int counter_avg = 0;
	int current_samples = 0;
	int current_samples1=0;
	
	char buffer[32];
	uint32_t i;
	PT_T p,pp,wavep,wavepp,setp[6],msk1,msk2,p1,pp1;
	if (first_time) {
		UI_Update_Field_Values(Fields, UI_NUM_FIELDS);	
	}
	//coordinates for plotting 
		wavep.X=0;
		wavep.Y=140;
		wavepp.X=240;
		wavepp.Y=20;
	UI_Update_Volatile_Field_Values(Fields);
	UI_Draw_Fields(Fields, UI_NUM_FIELDS);
	UI_Draw_Slider(&Slider);
	sem = osSemaphoreAcquire (Semaphore_print,0); //semaphore for plotting
	if (sem == osOK){
	LCD_Fill_Rectangle(&wavep,&wavepp,&yellow); //area for plotting
		
		LCD_Text_Set_Colors(&white, &black);

		if (g_sample_type==0){
		LCD_Text_PrintStr_RC(0, 3, " Average Sam ");
		LCD_Text_PrintStr_RC(1, 12, " Il-blue ");
			LCD_Text_PrintStr_RC(2, 12, " Im-red ");
		}
			switch (g_sample_type){
				case 0:
				for(i=0;i<960;i++){
					//averaging out 4 values to fit 960 samples onto 240 pixel screen
				counter_avg ++;
				current_samples= current_samples + py[i];
					current_samples1= current_samples1 + i_set[i];
				if (counter_avg == 4){
				p.X = pp.X;
				p.Y = pp.Y;
				pp.X = (uint32_t)((i+1)/4);
				pp.Y = (uint32_t)(140-(3*(current_samples/10)));
				p1.X = pp1.X;
				p1.Y = pp1.Y;
				pp1.X = (uint32_t)((i+1)/4);
				pp1.Y = (uint32_t)(140-(3*(current_samples1/10)));
				LCD_Draw_Line(&p,&pp,&dark_magenta);//printing the measured current
					LCD_Draw_Line(&p1,&pp1,&dark_blue);//printing the set current
				counter_avg =0;
				current_samples=0;
					current_samples1=0;
				}
				}
				break;
				
			}
			osSemaphoreRelease(Semaphore_print_complete);
}
	
	
}
