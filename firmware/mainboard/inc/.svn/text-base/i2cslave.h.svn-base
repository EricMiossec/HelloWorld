#include "chconf.h"
#define I2C_DUTYCYCLE I2C_DutyCycle_2

// definitions for I2C slave
#define SLAVE_ADDRESS 0x02		// 7 bits address
//#define SLAVE_ADDRESS 0x0330 		// 10 bits address
#define I2C_SPEED 9600
#define I2C_DUTYCYCLE I2C_DutyCycle_2

// interrupt handlers
#define I2C1_EV_IRQHandler VectorBC
#define I2C1_ER_IRQHandler VectorC0

#define I2Ca 					I2C1
#define I2Cx_CLK 				RCC_APB1Periph_I2C1
#define I2Cx_EV_IRQn 			I2C1_EV_IRQHandler
#define I2Cx_ER_IRQn 			I2C1_ER_IRQHandler

#define I2Cx_SDA_GPIO_CLK 		RCC_AHBPeriph_GPIOB
#define I2Cx_SDA_PIN 			GPIO_Pin_7
#define I2Cx_SDA_GPIO_PORT 		GPIOB
#define I2Cx_SDA_SOURCE 		GPIO_PinSource11
#define I2Cx_SDA_AF 			GPIO_AF_I2C1

#define I2Cx_SCL_GPIO_CLK 		RCC_AHBPeriph_GPIOB
#define I2Cx_SCL_PIN 			GPIO_Pin_6
#define I2Cx_SCL_GPIO_PORT 		GPIOB
#define I2Cx_SCL_SOURCE 		GPIO_PinSource10
#define I2Cx_SCL_AF 			GPIO_AF_I2C1

// Register Addresses
// Constants
#define REG_ADDR_FIRMWAREVERSION_REVISION 0x00
#define REG_ADDR_PRODUCT_ID 0x08
#define REG_ADDR_SENSOR_TYPE 0x10
// Variables
#define REG_ADDR_DISTANCE_MEASUREMENT 0x40
// Commands
#define REG_ADDR_LASER_ON_OR_OFF 0x40
#define REG_ADDR_SAMP_FREQ 0x41

// configurations
void GPIO_Configuration(void);
void I2C_Configuration(void);


