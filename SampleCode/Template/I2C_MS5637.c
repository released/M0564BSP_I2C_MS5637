
/* Includes ------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "I2C_MS5637.h"
#include <stdio.h>
#include <math.h>

#include "i2c_master.h"

unsigned int C[8] = {0};    // calibration coefficients   

const unsigned int nprom[] = {0x3132,0x3334,0x3536,0x3738,0x3940,0x4142,0x4344,0x4500};

void MS5637_Delay_ms(uint16_t nCount)
{
	TIMER_Delay(TIMER0, 1000*nCount);
}

void MS5637_Delay_us(uint16_t nCount)
{
	TIMER_Delay(TIMER0, nCount);
}

void appMS5637_Read(uint8_t DeviceAddr, uint8_t RegisterAddr,
                              uint16_t NumByteToRead,
                              uint8_t* pBuffer)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i, tmp;
	I2C_T *i2c = MASTER_I2C;
	
	I2C_START(i2c);                         			//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_WR );             		//send slave address+W
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, RegisterAddr);             		//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_CONTROL_REG(i2c, I2C_CTL_STA_SI);		//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_RD );    			//send slave address+R
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	for (i=0; i<NumByteToRead; i++)
	{
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);
		tmp = I2C_GET_DATA(i2c);           			//read data
		pBuffer[i]=tmp;
	}
	I2C_STOP(i2c);									//Stop

	#elif defined (ENABLE_I2C_POLLING_API)
	
	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
//	uint8_t i = 0;
	
	I2C_ReadMultiBytesOneReg(MASTER_I2C, u8SlaveAddr, RegisterAddr, pBuffer, NumByteToRead);	
//	i = I2C_ReadByteOneReg(MASTER_I2C, u8SlaveAddr, RegisterAddr);
//	*pBuffer =  i;

	#elif defined (ENABLE_I2C_IRQ)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
	
	I2Cx_ReadMultiFromSlaveIRQ(u8SlaveAddr , RegisterAddr, pBuffer, NumByteToRead);
	
	#endif
}

void appMS5637_Write(uint8_t DeviceAddr, uint8_t RegisterAddr,
                               uint16_t NumByteToWrite,
                               uint8_t* pBuffer)
{
	#if defined (ENABLE_I2C_POLLING_DISCRETE)
	uint8_t i;
	uint32_t tmp;
	
	I2C_T *i2c = MASTER_I2C;	
	I2C_START(i2c);                    			//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_WR );        			//send slave address
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, RegisterAddr);        			//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	for (i=0; i<NumByteToWrite; i++)
	{
		tmp = pBuffer[i];
		I2C_SET_DATA(i2c, tmp);            		//send Data
		I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
		I2C_WAIT_READY(i2c);
	}

	I2C_STOP(i2c);								//Stop

	#elif defined (ENABLE_I2C_POLLING_API)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;

	I2C_WriteMultiBytesOneReg(MASTER_I2C, u8SlaveAddr, RegisterAddr, pBuffer, NumByteToWrite);
//	I2C_WriteByteOneReg(MASTER_I2C, u8SlaveAddr , RegisterAddr, *pBuffer);	

	#elif defined (ENABLE_I2C_IRQ)

	/* u8SlaveAddr     Access Slave address(7-bit) */
	uint8_t u8SlaveAddr = DeviceAddr >>1;
	
	I2Cx_WriteMultiToSlaveIRQ(u8SlaveAddr , RegisterAddr, pBuffer, NumByteToWrite);

	#endif
}

							   
void appMS5637_WriteCmd(uint8_t DeviceAddr, uint8_t RegisterAddr)
{

//	uint8_t i;
//	uint32_t tmp;
	
	I2C_T *i2c = MASTER_I2C;	
	I2C_START(i2c);                    						//Start
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, DeviceAddr | I2C_WR );        			//send slave address
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_SET_DATA(i2c, RegisterAddr);        					//send index
	I2C_SET_CONTROL_REG(i2c, I2C_CTL_SI);
	I2C_WAIT_READY(i2c);

	I2C_STOP(i2c);											//Stop

}

unsigned char crc4(unsigned int n_prom[]) 
{ 
	int cnt;          // simple counter   
	unsigned int n_rem;       // crc reminder 
	unsigned int crc_read;      // original value of the crc 
	unsigned char  n_bit; 
	n_rem = 0x00; 
	crc_read=n_prom[7];       //save read CRC 
	n_prom[7]=(0xFF00 & (n_prom[7]));   //CRC byte is replaced by 0 
	for (cnt = 0; cnt < 16; cnt++)        // operation is performed on bytes 
	{// choose LSB or MSB 
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF); 
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8); 
		for (n_bit = 8; n_bit > 0; n_bit--) 
		{ 
			if (n_rem & (0x8000)) 
			{ 
				n_rem = (n_rem << 1) ^ 0x3000; 

			} 
			else 
			{ 
				n_rem = (n_rem << 1); 
			} 
		} 
	} 
	n_rem=  (0x000F & (n_rem >> 12));   // final 4-bit reminder is CRC code 
	n_prom[7]=crc_read;     // restore the crc_read to its original place 
	return (n_rem ^ 0x0); 
				   
}

unsigned int cmd_prom(char coef_num) 
{ 
//	unsigned int ret; 
	unsigned int rC=0; 
	I2C_T *i2c = MASTER_I2C;
	uint8_t ReadData[2] = {0};	
	uint8_t ReadLen = 0;
	
	appMS5637_WriteCmd(MS5637_ADDRESS_8BIT | I2C_WR , CMD_PROM_RD+coef_num*2);   			// send PROM READ command 

	ReadLen = I2C_ReadMultiBytes(i2c,MS5637_ADDRESS_7BIT , ReadData , 2);

	printf("ReadLen = 0x%2X\r\n" , ReadLen);

	rC = 256*ReadData[0]; 
	rC = rC + ReadData[1]; 
		 

	return rC; 
}

unsigned long cmd_adc(char cmd) 
{ 
//	unsigned int ret; 
	unsigned long temp=0;               
//	uint8_t i;
	I2C_T *i2c = MASTER_I2C;
	uint8_t ReadData[3] = {0};
	uint8_t ReadLen = 0;
	
	appMS5637_WriteCmd(MS5637_ADDRESS_8BIT | I2C_WR ,CMD_ADC_CONV+cmd);   // send conversion command 
	switch (cmd & 0x0f)         													// wait necessary conversion time 
	{ 
		case CMD_ADC_256 : MS5637_Delay_us(900); 	break; 
		case CMD_ADC_512 : MS5637_Delay_ms(3);   	break; 
		case CMD_ADC_1024: MS5637_Delay_ms(4);   	break; 
		case CMD_ADC_2048: MS5637_Delay_ms(6);   	break; 
		case CMD_ADC_4096: MS5637_Delay_ms(10);  	break; 
	} 
	appMS5637_WriteCmd(MS5637_ADDRESS_8BIT | I2C_WR ,CMD_ADC_READ); 

	ReadLen = I2C_ReadMultiBytes(i2c,MS5637_ADDRESS_7BIT , ReadData , 3);

//	printf("ReadLen = 0x%2X\r\n" , ReadLen);
	
	temp = 65536*ReadData[0];
	temp = temp + 256*ReadData[1];
	temp = temp + ReadData[2];  

	return temp; 
	
}

void cmd_reset(void) 
{   
	appMS5637_WriteCmd(MS5637_ADDRESS_8BIT | I2C_WR ,CMD_RESET);      		// send reset sequence 
	MS5637_Delay_ms(3);         														// wait for the reset sequence timing 
} 

void appMS5637_Setup(void)
{
	int i; 
	unsigned char n_crc;  // crc value of the prom 

	
	cmd_reset();    
	for (i=0;i<8;i++){ C[i]=cmd_prom(i);}  // read coefficients 
	n_crc=crc4(C);        // calculate the CRC 
	
}	

void appMS5637_GetData(void)
{
	unsigned long D1;     // ADC value of the pressure conversion 
	unsigned long D2;     // ADC value of the temperature conversion  
	double P;      // compensated pressure value 
	double T;      // compensated temperature value 
	double dT;      // difference between actual and measured temperature 
	double OFF;      // offset at actual temperature 
	double SENS;      // sensitivity at actual temperature 

  
	D1=cmd_adc(CMD_ADC_D1+CMD_ADC_256);  // read uncompensated pressure 
	D2=cmd_adc(CMD_ADC_D2+CMD_ADC_4096);  // read uncompensated temperature 


    // calcualte 1st order pressure and temperature (MS5607 1st order algorithm) 
    dT=D2-C[5]*pow(2,8); 
    OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);         
    SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7); 
   
    T=(2000+(dT*C[6])/pow(2,23))/100; 
    P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15))/100; 

	printf("-Pressure altitude: P:%.3fmbar  T:%.2fC , D1: %lu, D2: %lu\n\r",P,T , D1, D2);	
	
}	


