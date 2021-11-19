
#include <stdio.h>
#include "M0564.h"

#define ENABLE_I2C_POLLING_DISCRETE
//#define ENABLE_I2C_POLLING_API
//#define ENABLE_I2C_IRQ

//#define DEBUG_LOG_MASTER_LV1


#define MASTER_I2C						  		(I2C0)
#define MASTER_I2C_IRQn						  	(I2C0_IRQn)
#define I2Cx_Master_IRQHandler					(I2C0_IRQHandler)

// #define MASTER_I2C						  		(I2C1)
// #define MASTER_I2C_IRQn						  	(I2C1_IRQn)
// #define I2Cx_Master_IRQHandler					(I2C1_IRQHandler)

#define I2C_WR                  					(0x00)
#define I2C_RD                  					(0x01)

#define MASTER_START_TRANSMIT			  		(0x08)
#define MASTER_REPEAT_START               		(0x10)
#define MASTER_TRANSMIT_ADDRESS_ACK       	(0x18)
#define MASTER_TRANSMIT_ADDRESS_NACK      	(0x20)
#define MASTER_TRANSMIT_DATA_ACK          	(0x28)
#define MASTER_TRANSMIT_DATA_NACK         	(0x30)
#define MASTER_ARBITRATION_LOST           		(0x38)
#define MASTER_RECEIVE_ADDRESS_ACK        		(0x40)
#define MASTER_RECEIVE_ADDRESS_NACK       		(0x48)
#define MASTER_RECEIVE_DATA_ACK           		(0x50)
#define MASTER_RECEIVE_DATA_NACK          		(0x58)
#define BUS_ERROR                         		(0x00)


#define ADDRESS_TRANSMIT_ARBITRATION_LOST    	(0xB0)

void I2Cx_WriteSingleToSlaveIRQ(uint8_t address,uint8_t reg, uint8_t *data);
void I2Cx_ReadSingleToSlaveIRQ(uint8_t address, uint8_t reg,uint8_t *data);

void I2Cx_WriteMultiToSlaveIRQ(uint8_t address,uint8_t reg,uint8_t *data,uint16_t len);
void I2Cx_ReadMultiFromSlaveIRQ(uint8_t address,uint8_t reg,uint8_t *data,uint16_t len);


