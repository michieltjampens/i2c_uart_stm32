#ifndef INC_USART2_H_
#define INC_USART2_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "./stm32c0xx.h"

/* Private includes ----------------------------------------------------------*/
/* Shared Variables  ---------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void USART2_Configure(void);
void USART2_Configure_Setup(void);
void USART2_Configure_GPIO(void);

/* Private defines -----------------------------------------------------------*/
#ifdef __cplusplus
}
#endif

#endif /* INC_USART2_H_ */
