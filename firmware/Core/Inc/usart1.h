#ifndef INC_USART1_H_
#define INC_USART1_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "./stm32c0xx.h"

/* Private includes ----------------------------------------------------------*/


/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void USART1_Configure(void);
void USART1_Configure_Setup(void);
void USART1_Configure_GPIO(void);

/* Private defines -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* INC_USART1_H_ */
