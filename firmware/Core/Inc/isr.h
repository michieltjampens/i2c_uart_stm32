#ifndef INC_ISR_H_
#define INC_ISR_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "./stm32c0xx.h"

/* Private includes ----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void ISR_Init(void);
void USART_DMA_Init(void);
void Check_usart1_out(void);
void Check_usart2_out(void);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_USART_TRANSMIT 0x20
#define ERROR_USART_ISR_TC 	 0x21
#define ERROR_USART_TIMEOUT  0x22

#define CIRCULAR 500

#define IDLE 	0x01
#define BUSY 	0x02
#define WAITING 0x04

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


#define I2C_IDLE_STATE 	 	 0x00
// Status
#define I2C_REC_STATUS	     0x01
#define I2C_REC_STATUS1	     0x11
#define I2C_REC_STATUS2      0x21
// Size
#define I2C_TO_UART_SIZE     0x02
#define I2C_TO_UART1_SIZE    0x12
#define I2C_TO_UART2_SIZE    0x22
// i2c->uart Data
#define I2C_TO_UART_DATA     0x03
#define I2C_TO_UART1_DATA    0x13
#define I2C_TO_UART2_DATA    0x23
// Conf
#define I2C_CONF_STATE 	     0x04
#define I2C_CONF1_STATE 	 0x14
#define I2C_CONF2_STATE 	 0x24
// uart->i2c
#define UART_TO_I2C_DATA     0x05
#define UART1_TO_I2C_DATA    0x15
#define UART2_TO_I2C_DATA    0x25

#ifdef __cplusplus
}
#endif

#endif /* INC_USART1_H_ */
