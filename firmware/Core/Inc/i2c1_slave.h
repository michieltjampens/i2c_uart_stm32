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

/* Private defines -----------------------------------------------------------*/
#define I2C_OK 0x01
#define I2C1_OWN_ADDRESS1 0x1B
#define I2C1_OWN_ADDRESS2 0x2B



#endif /* INC_I2C1_SLAVE_H_ */
