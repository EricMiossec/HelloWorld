#include "ch.h"
#include "i2cslave.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "calib.h"


#define FIRMWARE_VERSION 0x11
#define IDENT 0x5A

static int testvalue=0;

static struct {
		enum {
				// Nothing
				ST_IDLE,
				// The chip has been addressed
				ST_ADDRESSED,
				// The read/write address has been received
				ST_ADDR_RXED,
				// Currently reading bytes
				ST_READING,
				// Currently writing bytes
				ST_WRITING,
		} state;
		// An incrementing ID for housekeeping tasks to kill a stalled transfer
		uint8_t id;
		// The current address being read or writting
		uint8_t addr;
		uint8_t testbuffer[256];
} i2c_state;


void I2C_Configuration(void)
{
	int i;
	
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	/*!< I2C Periph clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	
	//!< Configure I2C SCL pin
	GPIO_InitStructure.GPIO_Pin = I2Cx_SCL_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStructure);

	//!< Configure I2C SDA pin
	GPIO_InitStructure.GPIO_Pin = I2Cx_SDA_PIN;
	GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStructure);
	
	// Configure the Priority Group to 1 bit
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
    // Configure the I2C event priority 
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	// Configure I2C error interrupt to have the higher priority 
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	// deinitialize i2c first
	I2C_DeInit(I2Ca);
	
	//!< I2C Struct Initialize 
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DUTYCYCLE;
	I2C_InitStructure.I2C_OwnAddress1 = SLAVE_ADDRESS;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	
	// initialize structure for 7bit or 10bit
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	
	I2C_Init(I2Ca, &I2C_InitStructure);
	
	// Populate test buffer with predictable values
	for(i = 0; i < 256; i++)
		i2c_state.testbuffer[i] = i;
	
	// Initialize i2c_state
	i2c_state.id = 0;
	i2c_state.state = ST_IDLE;
	
	// Enable Error Interrupt
	I2C_ITConfig(I2Ca, I2C_IT_ERR , ENABLE);
	
	// Enable I2C event interrupt
	I2C_ITConfig(I2Ca, I2C_IT_EVT, ENABLE);
	// I2C ENABLE
	I2C_Cmd(I2Ca, ENABLE);
}

uint8_t vreg_read(uint8_t vaddr){
	uint8_t val;
	//return i2c_state.testbuffer[vaddr];
	switch(vaddr){
		case REG_ADDR_FIRMWAREVERSION_REVISION:
				return FIRMWARE_VERSION;
		case REG_ADDR_PRODUCT_ID:
				return IDENT;
		default:
				return i2c_state.testbuffer[testvalue++];
	}
}

void vreg_write(uint8_t vaddr, uint8_t value){
	i2c_state.testbuffer[vaddr] = value;
	switch(vaddr){
		case REG_ADDR_FIRMWAREVERSION_REVISION:
		case REG_ADDR_PRODUCT_ID:
			break;
	}
}

void I2C1_ER_IRQHandler(){

	switch(i2c_state.state){
	case ST_READING:
	case ST_WRITING:
			// Read SR1 register to get I2C error
			if ((I2C_ReadRegister(I2Ca, I2C_Register_SR1) & 0xFF00) != 0x00){
					// Clears error flags
					I2Ca->SR1 &= 0x00FF;
					// Reset
					i2c_state.state = ST_IDLE;
					// Indicate next transfer
		 			i2c_state.id += 1;
					//i2c_state.addr = 0;
					
					// Reset the I2C so that it doesn't try to go into idle anymore
					I2C_Cmd(I2Ca, DISABLE);
					I2C_Cmd(I2Ca, ENABLE);
			}
			break;
	default:
		// This is funked
		//while(1);
	}
}

void I2C1_EV_IRQHandler(){
		static uint32_t event = 0;
		
		event = I2C_GetLastEvent(I2Ca);

		switch(i2c_state.state){
		case ST_IDLE:
				// Expecting to be addressed as receiver to receive a register address
				switch(event){
				case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
						// Advance state
						i2c_state.state = ST_ADDRESSED;
						I2C_ITConfig(I2Ca, I2C_IT_BUF, ENABLE);
						break;
				default:
						//while(1);
						break;
				}
				break;
		case ST_ADDRESSED:
				// Expecting to receive a register start address
				switch(event){
				case I2C_EVENT_SLAVE_BYTE_RECEIVED:
				case (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_SR1_BTF):
						i2c_state.addr = I2C_ReceiveData(I2Ca);
						i2c_state.state = ST_ADDR_RXED;
						break;
				default:
						//while(1);
				}
				break;
		case ST_ADDR_RXED:
				// Expecting either a new stop and start and transmitter address
				// OR the first byte to write
				switch(event){
				case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED | I2C_EVENT_SLAVE_STOP_DETECTED:
						// Now ready to send data back
						i2c_state.state = ST_READING;
						I2C_SendData(I2Ca, vreg_read(i2c_state.addr++));
						I2C_ITConfig(I2Ca, I2C_IT_BUF , ENABLE);
						break;
				case I2C_EVENT_SLAVE_BYTE_RECEIVED:
						// This is the first byte to write
						i2c_state.state = ST_WRITING;
						vreg_write(i2c_state.addr++, I2C_ReceiveData(I2Ca));
						break;
				default:
						//while(1);
				}
				break;
		case ST_WRITING:
				// Expecting either a stop bit or another byte
				switch(event){
				case I2C_EVENT_SLAVE_BYTE_RECEIVED:
						vreg_write(i2c_state.addr++, I2C_ReceiveData(I2Ca));
						break;
				case I2C_EVENT_SLAVE_STOP_DETECTED:
						// Done
						I2C_GetFlagStatus(I2Ca, I2C_FLAG_STOPF);
  						I2C_Cmd(I2Ca, ENABLE);
						i2c_state.state = ST_IDLE;
						break;
				default:
						break;
						//while(1);
				}
				break;
		case ST_READING:
				// Expecting
				switch(event){
				case I2C_EVENT_SLAVE_BYTE_TRANSMITTING:
				case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
						I2C_SendData(I2Ca, vreg_read(i2c_state.addr++));
						break;
				case I2C_EVENT_SLAVE_STOP_DETECTED:
						// Read done
						i2c_state.id += 1;
						I2C_GetFlagStatus(I2Ca, I2C_FLAG_STOPF);
  						I2C_Cmd(I2Ca, ENABLE);
						i2c_state.state = ST_IDLE;
						break;
				default:
						//while(1);
						break;
				}
				break;
		default:
				//while(1);
		}
}

void NMIVector(void)
{
	while(1);
}
void HardFaultVector(void)
{
	while(1);
}
MemManageVector(void)
{
	while(1);
}
BusFaultVector(void)
{
	while(1);
}
UsageFaultVector(void)
{
	while(1);
}

















