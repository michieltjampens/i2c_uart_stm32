#ifndef INC_USART2_H_
#define INC_USART2_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "./stm32c0xx.h"

/* Private includes ----------------------------------------------------------*/
/* Shared Variables  ---------------------------------------------------------*/
extern __IO uint32_t Tick;
extern __IO uint16_t error;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void USART2_Configure(void);
uint8_t USART2_Buffer_Free(void);
void USART2_Configure_Setup(void);
void USART2_Configure_GPIO(void);

void USART2_writeByte( uint8_t data );
void USART2_Transfer_Buffer( void );

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ERROR_USART_TRANSMIT 0x20
#define ERROR_USART_ISR_TC 	 0x21
#define ERROR_USART_TIMEOUT  0x22

#define CIRCULAR 512

#define IDLE 	0x01
#define BUSY 	0x02
#define WAITING 0x04

extern uint32_t Tock;
#ifdef __cplusplus
}
#endif

#endif /* INC_USART2_H_ */
