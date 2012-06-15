#include "calib.h"
#include "stm32f10x_gpio.h"

uint16_t LookupTable[LUT_SIZE];
uint16_t LatestMeasures[NUM_CHANNELS];
uint16_t CalibFlags[3];

uint16_t getLatest()
{
	return LatestMeasures[0];
}

void ADC_Configuration(int channel, GPIO_TypeDef * port, uint16_t pin)
{
	uint16_t result = 0;
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

  	// Configure PC0 (ADC Channel10) as analog input
  	GPIO_InitStructure.GPIO_Pin = pin;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  	GPIO_Init(port, &GPIO_InitStructure);
    // Enable ADC1 clock
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );
	
    // ADC1 Configuration ------------------------------------------------------
	ADC_InitStructure.ADC_Mode                = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode        = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode  = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv    = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign           = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel        = 1; 
	ADC_Init( ADC1, &ADC_InitStructure );
	
	// ADC1 Regular Channels
	ADC_RegularChannelConfig( ADC1, channel,  1, ADC_SampleTime_239Cycles5);
		
	// Enable ADC1 external trigger conversion
	ADC_ExternalTrigConvCmd( ADC1, ENABLE );
	
	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	
	// Enable ADC1 reset calibration register 
	ADC_ResetCalibration(ADC1);
	
	// Check the end of ADC1 reset calibration register
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	// Start ADC1 calibration
	ADC_StartCalibration(ADC1);
	
	// Check the end of ADC1 calibration
	while(ADC_GetCalibrationStatus(ADC1));
	
}

void GenerateLookupTable()
{
	
}

void update_volt_measures()
{
	// Configure the right ADC channel
	ADC_Configuration(ADC_SAMPLING_LEFT_CHANNEL, 
						ADC_SAMPLING_LEFT_CHANNEL_GPIO_PORT, 
						ADC_SAMPLING_LEFT_CHANNEL_PIN);
	// Start ADC1 Software Conversion
    ADC_SoftwareStartConvCmd( ADC1, ENABLE );
	
     // Wait for end of conversion
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	
    // Return the conversion value
    LatestMeasures[0] = ADC_GetConversionValue(ADC1);
	
		// Configure the right ADC channel
	ADC_Configuration(ADC_SAMPLING_RIGHT_CHANNEL, 
						ADC_SAMPLING_RIGHT_CHANNEL_GPIO_PORT, 
						ADC_SAMPLING_RIGHT_CHANNEL_PIN);
	// Start ADC1 Software Conversion
    ADC_SoftwareStartConvCmd( ADC1, ENABLE );
	
     // Wait for end of conversion
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	
	LatestMeasures[1] = ADC_GetConversionValue(ADC1);
}

void cmd_calib(BaseChannel *chp, int argc, char *argv[]) 
{
	char* calibrating_distance = argv[1];
	int LUT_index = 0;
	float distance_fraction;
	int ratio = 0;
	
	uint16_t volt_sample;
	if(argc > 2)
	{
		chprintf(chp, "Invalid Calibration Command\n");
		return;
	}
	else if(strcmp(argv[1], "MINDISTANCE") )
	{
		update_volt_measures();
		chprintf(chp, "Minimum Distance: Values are: CH1: %d and CH2: %d\n", LatestMeasures[0], LatestMeasures[1]);
		chprintf(chp, "Ratio is %5f\n" , LatestMeasures[0]/LatestMeasures[1]);
	}
	else if(strcmp(argv[1], "MIDDISTANCE") )
	{
		update_volt_measures();
		chprintf(chp, "Middle Distance: Values are: CH1: %d and CH2: %d\n", LatestMeasures[0], LatestMeasures[1]);
		chprintf(chp, "Ratio is %5f\n" , LatestMeasures[0]/LatestMeasures[1]);
	}
	else if(strcmp(argv[1], "MAXDISTANCE") )
	{
		update_volt_measures();
		chprintf(chp, "Maximum Distance: Values are: CH1: %d and CH2: %d\n", LatestMeasures[0], LatestMeasures[1]);
		chprintf(chp, "Ratio is %5f\n" , LatestMeasures[0]/LatestMeasures[1]);
	}
	// Invalid distances
	else
	{
		chprintf(chp, "Invalid Calibration Command\n");
		return;
	}
}


