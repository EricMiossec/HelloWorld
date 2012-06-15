#include "ch.h"
#include "hal.h"
#include "shell.h"
#include <string.h>
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

#define DEV_BRD 1
#if DEV_BRD
#define ADC_SAMPLING_LEFT_CHANNEL ADC_Channel_14
#define ADC_SAMPLING_LEFT_CHANNEL_GPIO_PORT GPIOC
#define ADC_SAMPLING_LEFT_CHANNEL_PIN GPIO_Pin_4
#define ADC_SAMPLING_RIGHT_CHANNEL ADC_Channel_15
#define ADC_SAMPLING_RIGHT_CHANNEL_GPIO_PORT GPIOC
#define ADC_SAMPLING_RIGHT_CHANNEL_PIN GPIO_Pin_5
#else
#define ADC_SAMPLING_LEFT_CHANNEL ADC_Channel_10
#define ADC_SAMPLING_LEFT_CHANNEL_GPIO_PORT GPIOC
#define ADC_SAMPLING_LEFT_CHANNEL_PIN GPIO_Pin_0
#define ADC_SAMPLING_RIGHT_CHANNEL ADC_Channel_11
#define ADC_SAMPLING_RIGHT_CHANNEL_GPIO_PORT GPIOC
#define ADC_SAMPLING_RIGHT_CHANNEL_PIN GPIO_Pin_1
#endif

#define LUT_SIZE 4096
#define MAX_DISTANCE 50
#define NUM_CHANNELS 2

// Calibration points (in cm)
#define MINIMUM_DISTANCE 	0
#define MAXIMUM_DISTANCE 	50
#define MIDDLE_DISTANCE 	MAXIMUM_DISTANCE/2
// Calibration points indices (in cm)
#define MINIMUM_DISTANCE_INDEX 	0
#define MAXIMUM_DISTANCE_INDEX 	LUT_SIZE - 1
#define MIDDLE_DISTANCE_INDEX 	LUT_SIZE/2

// Index into the array of flags which intdiacted if a certain distance was calibrated
#define MIN_DISTANCE_CALIB_FLAG_INDEX 		0
#define MID_DISTANCE_CALIB_FLAG_INDEX 		1
#define MAX_DISTANCE_CALIB_FLAG_INDEX 		2

uint16_t getLatest();

void update_volt_measures(void);

void ADC_Configuration(int channel, 
						GPIO_TypeDef * port,
						 uint16_t pin);

void cmd_calib(BaseChannel *chp, int argc, char *argv[]) ;
