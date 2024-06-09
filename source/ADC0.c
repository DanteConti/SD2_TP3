/*
 * ADC0.c
 *
 *  Created on: 16 abr. 2022
 *      Author: dante
 */

#include <common.h>
#include "ADC0.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_port.h"

#define DEMO_ADC16_CHANNEL_GROUP 	0U
#define DEMO_ADC16_USER_CHANNEL 	3U /* PTE22*/

#define SENSOR_LUZ_PORT         	PORTE
#define SENSOR_LUZ_PIN          	22

adc16_channel_config_t adc16_channel = {
	.channelNumber = 0,
	.enableInterruptOnConversionCompleted = false,
	.enableDifferentialConversion = false,
};

uint32_t g_Adc16ConversionValue = 15;
uint16_t brightnessScale;

void ADC0_init(){
		adc16_config_t adc16_config;

		PORT_SetPinMux(SENSOR_LUZ_PORT, SENSOR_LUZ_PIN, kPORT_PinDisabledOrAnalog);

		/*
		 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
		 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
		 * adc16ConfigStruct.enableAsynchronousClock = true;
		 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
		 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
		 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
		 * adc16ConfigStruct.enableHighSpeed = false;
		 * adc16ConfigStruct.enableLowPower = false;
		 * adc16ConfigStruct.enableContinuousConversion = false;
		 */

		ADC16_GetDefaultConfig(&adc16_config);
		adc16_config.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
		ADC16_Init(ADC0, &adc16_config);
		ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */

		adc16_channel.channelNumber = DEMO_ADC16_USER_CHANNEL;
		adc16_channel.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
		adc16_channel.enableDifferentialConversion = false;


		NVIC_EnableIRQ(ADC0_IRQn);
}

void ADC0_iniciarConv(){
	/*
		 When in software trigger mode, each conversion would be launched once calling the "ADC16_ChannelConfigure()"
		 function, which works like writing a conversion command and executing it. For another channel's conversion,
		 just to change the "channelNumber" field in channel configuration structure, and call the function
		 "ADC16_ChannelConfigure()"" again.
		 Also, the "enableInterruptOnConversionCompleted" inside the channel configuration structure is a parameter for
		 the conversion command. It takes affect just for the current conversion. If the interrupt is still required
		 for the following conversion, it is necessary to assert the "enableInterruptOnConversionCompleted" every time
		 for each command.
		 */
		ADC16_SetChannelConfig(ADC0, DEMO_ADC16_CHANNEL_GROUP, &adc16_channel);
}

void ADC0_IRQHandler(void) {
	g_Adc16ConversionDoneFlag = true;
	/* Read conversion result to clear the conversion completed flag. */
	g_Adc16ConversionValue = ADC16_GetChannelConversionValue(ADC0, DEMO_ADC16_CHANNEL_GROUP);
}

uint16_t ADC0_getBrightness(){
	brightnessScale = 4095 - g_Adc16ConversionValue;
	brightnessScale = brightnessScale * 100 / 4095;
	return brightnessScale;
}
