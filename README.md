#Sharing the ADC on FRDM_KL25Z

In this project, the ADC on the FRDM KL25Z was shared between the touchscreen and the buck converter. The current flowing throught the buck converter was also displayed on the touchscreen

The main task of the project was to share the ADC and plot the current on the LCD display. The hardware used for the purpose of demonstration was:
a.	NXP FRDM-KL25Z development board
b.	320x240 pixel color TFT LCD with touchscreen
c.	NXP FRDM-KL25Z Shield
d.	Analog Discovery 2

The code was developed on CMSIS debugger of the Keil uvision software and signals were observed using waveforms.

The project was majorly divided into two parts
1.	Modifying an IRQ_Handler which has the ability to share the ADC between the high brightness LED and the LCD touchscreen
2.	Writing code which will be able to display the current of the high brightness LED on the LCD display

The task of sharing the ADC was completed by modifying the Interrupt handler such that it services the time critical ControlHBLed() function and queues the LCD touchscreen requests in a Message Queue. 

Video for the project:-https://youtu.be/jPraNhVdCk4
