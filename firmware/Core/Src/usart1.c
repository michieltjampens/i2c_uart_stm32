/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/usart1.h"

/* Private variables ---------------------------------------------------------*/

void USART1_Configure(){
    USART1_Configure_GPIO();
    USART1_Configure_Setup();
}
void USART1_Configure_GPIO(void){
  /* Enable the peripheral clock of GPIOA */
  RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

  /* GPIO configuration for USART1 signals */
  /* (1) Select Alternate Function (AF) mode (b10) on PA0 and PA1 */
  /* Moder LSB nibble: 1111 -> 1010 */
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODE0|GPIO_MODER_MODE1))\
                 | (GPIO_MODER_MODE0_1 | GPIO_MODER_MODE1_1);
  /* First clear the AF setting for pin 0 and 1
     * -> This is in AFR[0] because pin number below 8
     * -> Start with clearing the AF for those pins by doing a reverse and
     * -> Then set AF bits to AF4 (0100 or 0x04) using AFSELx_2 which toggles the third bit
     */
    GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1)) | (GPIO_AFRL_AFSEL0_2|GPIO_AFRL_AFSEL1_2);
}
void USART1_Configure_Setup(void){

    /* Enable the peripheral clock USART1 */
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;
    /* Configure USART1 */
    /* System clock is 12MHz, 38400 baud (both already divided by 100) */
    USART1->BRR = 120000/384;//384 -> 38400Bd

    //USART1->CR2 |= USART_CR2_SWAP; /* Swap RX and TX this needs to be set before CR1_UE is set */
	USART1->CR3 |= USART_CR3_DMAT;		// Enable DMA triggering for transmit

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    USART1->CR1 = (USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE);
    /* Extra info                                 */
    /* USART1->CR1 = USART1 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_RXFFIE = Rx buffer full int      */
    /* USART_CR1_IDLEIE = Idle isr enabled        */

	USART1->CR1 |= USART_CR1_UE; // Enable the uart

	/* Configure Interrupt */
	/* Set priority for USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 0);
	/* Enable USART1_IRQn */
	NVIC_EnableIRQ(USART1_IRQn);
}
