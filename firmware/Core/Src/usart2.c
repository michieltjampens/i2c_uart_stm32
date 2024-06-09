/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/usart2.h"

/* Private variables ---------------------------------------------------------*/


void USART2_Configure(){
    USART2_Configure_GPIO();
    USART2_Configure_Setup();
}
void USART2_Configure_GPIO(void){
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* GPIO configuration for USART2 signals */
  /* (1) Select Alternate Function (AF) mode (b10) on PA0 and PA1 */
  /* Moder LSB nibble: 1111 -> 1010 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE2|GPIO_MODER_MODE3))\
                 | (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
  /* (2) AF1 for USART2 signals, the line is explained in detail below it */
  /* First clear the AF setting for pin 2 and 3
     * -> This is in AFR[0] because pin number below 8
     * -> Start with clearing the AF for those pins by doing a reverse and
     * -> Then set AF bits to AF1 (0001 or 0x01)
     */
    GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (GPIO_AFRL_AFSEL2 | GPIO_AFRL_AFSEL3)) | (GPIO_AFRL_AFSEL2_0|GPIO_AFRL_AFSEL3_0);

  /* Extra info                                                                         */
  /* For the alternate functions, check the datasheet 'Alternate functions'             */
  /* AFR[0]=pin 0 to 7, AFR[1]=pin 8 to 15, each pin has 4 bits for selection           */
  /* So AFR[0] &~(0x00000FF0) = AFR[0] & 0xFFFFF00F -> Reset the bits for PA0 & PA1     */
  /* 4 << (0 * 4) -> Value 4 shifted one nibble to get to PA0 position  -> 0x00000004   */
  /* 4 << (1 * 4) -> Value 4 shifted two nibbles to get to PA1 position -> 0x00000040   */
}
void USART2_Configure_Setup(void){

    /* Enable the peripheral clock USART2 */
    RCC->APBENR1 |= RCC_APBENR1_USART2EN;
    /* Configure USART2 */
    /* System clock is 12MHz, 38400 baud (both already divided by 100) */
    USART2->BRR = 120000/384;//simplified 12MHz/38400

    USART2->CR3 |= USART_CR3_DMAT;		// Enable DMA triggering for transmit

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    USART2->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE);
    /* Extra info                                 */
    /* USART1->CR1 = USART1 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_RXFFIE = Rx buffer full int      */
    /* USART_CR1_IDLEIE = Idle isr enabled        */

	/* Configure Interrupt */
	/* Set priority for USART2_IRQn */
	NVIC_SetPriority(USART2_IRQn, 0);
	/* Enable USART2_IRQn */
	NVIC_EnableIRQ(USART2_IRQn);

	USART2->CR1 |= USART_CR1_UE; 		// Enable the uart
}
