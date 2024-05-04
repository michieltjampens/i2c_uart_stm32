/*
 * i2c1.h
 */

#ifndef INC_I2C1_SLAVE_H_
#define INC_I2C1_SLAVE_H_
/* Includes ------------------------------------------------------------------*/
#include "./stm32c0xx.h"
/* Private includes ----------------------------------------------------------*/

/* Shared Variables ------------------------------------------------------------*/
extern __IO uint32_t Tick;
extern __IO uint16_t error;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void I2C1_Configure_Slave(void);

/* Other */
void I2C1_IRQHandler(void);

/* Private defines -----------------------------------------------------------*/
#define I2C_OK 0x01
#define I2C1_OWN_ADDRESS1 0x1B
#define I2C1_OWN_ADDRESS2 0x2B

// Errors
#define ERROR_I2C 		   		 0x10
#define ERROR_I2C_BERR			 0x11
#define ERROR_I2C_PECERR		 0x12
#define ERROR_I2C_OVR			 0x13
#define ERROR_I2C_ARLO			 0x14
#define ERROR_I2C_TIMEOUT		 0x15
#define ERROR_I2C_NONACK   		 0x16

#define ERROR_I2C_NO_BUSY_FLAG     0x17
#define ERROR_I2C_NO_TC_DETECT     0x18
#define ERROR_I2C_NO_BUSY_FLAG2    0x19
#define ERROR_I2C_NO_STOP_DT  	   0x1A
#define ERROR_I2C_DATA_REC_DELAY   0x1B
#define ERROR_I2C_NO_TXE_EMTPY     0x1C
#define ERROR_I2C_OVERFLOW 		   0x1D
#define ERROR_I2C_NACK   		   0x1E


#define I2C_TIMEOUT_VALUE  0x05
#define I2C_INVALID_ADDRES 0x03
#define I2C_NOTICK 0x04

#endif /* INC_I2C1_SLAVE_H_ */
