
/* Define to prevent recursive inclusion -------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#ifndef __I2C_MS5637_H
#define __I2C_MS5637_H

#ifdef __cplusplus
extern "C" {
#endif

#include "M0564.h"

#define MS5637_ADDRESS_7BIT                   				(0x76)
#define MS5637_ADDRESS_8BIT                 				(MS5637_ADDRESS_7BIT << 1)
#define MS5637_I2C_SPEED      								(100000)


typedef enum
{
	MS5637_OK = 0,
	MS5637_FAIL
}MS5637_Status_TypDef;

#define ADDR_W      										0xEE       	// Module address write mode 
#define ADDR_R  											0xEF       	// Module address read mode 
 
#define CMD_RESET    										0x1E    		// ADC reset command 
#define CMD_ADC_READ 										0x00    		// ADC read command 
#define CMD_ADC_CONV 										0x40    		// ADC conversion command 
#define CMD_ADC_D1   										0x00     	// ADC D1 conversion 
#define CMD_ADC_D2   										0x10     	// ADC D2 conversion 
#define CMD_ADC_256 	 									0x00     	// ADC OSR=256 
#define CMD_ADC_512  										0x02     	// ADC OSR=512 
#define CMD_ADC_1024 										0x04     	// ADC OSR=1024 
#define CMD_ADC_2048 										0x06     	// ADC OSR=2048 
#define CMD_ADC_4096 										0x08     	// ADC OSR=4096 
#define CMD_PROM_RD  										0xA0   		// Prom read command


/* Includes ------------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void appMS5637_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer);
void appMS5637_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer);

void appMS5637_Setup(void);
void appMS5637_GetData(void);


#ifdef __cplusplus
}
#endif

#endif /* __MS5637_H */

