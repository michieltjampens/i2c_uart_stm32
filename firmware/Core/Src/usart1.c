/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/usart1.h"
#include "../inc/main.h"

/* Shared variables ----------------------------------------------------------*/
__IO uint8_t cmdReady;
uint16_t freeSpace_USART1 = CIRCULAR-1;
uint8_t irqStatus_USART1=IDLE;

/* Private variables ---------------------------------------------------------*/

/* Circular buffer for UART OUT*/
uint8_t outputBuffer_USART1[CIRCULAR];
uint8_t *outTemp_USART1;
uint8_t *outStart_USART1;
uint8_t *outEnd_USART1;
uint8_t *outHead_USART1;
uint8_t *outTail_USART1;

/* Circular buffer for UART IN*/
uint8_t inputBuffer_USART1[CIRCULAR];
uint8_t *inputTemp_USART1;
uint8_t *inputStart_USART1;
uint8_t *inputEnd_USART1;
uint8_t *inputHead_USART1;
uint8_t *inputTail_USART1;


void USART1_Configure(){
    /* Initialise the circular buffers */
    inputStart_USART1 = &inputBuffer_USART1[0];
    inputEnd_USART1 =   &inputBuffer_USART1[0];
    inputHead_USART1 =  &inputBuffer_USART1[0];
    inputTail_USART1 =  &inputBuffer_USART1[CIRCULAR-1];

    outStart_USART1 = &outputBuffer_USART1[0];
    outEnd_USART1  =  &outputBuffer_USART1[0];
    outHead_USART1 =  &outputBuffer_USART1[0];
    outTail_USART1 =  &outputBuffer_USART1[CIRCULAR-1];

    USART1_Configure_GPIO();
    USART1_Configure_Setup();

    cmdReady=0;
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
    GPIOA->AFR[0] = (GPIOA->AFR[0] &~ (GPIO_AFRL_AFSEL0 | GPIO_AFRL_AFSEL1)) | (GPIO_AFRL_AFSEL1_2|GPIO_AFRL_AFSEL1_2);
}
void USART1_Configure_Setup(void){
	uint32_t tickstart;

    /* Enable the peripheral clock USART1 */
    RCC->APBENR2 |= RCC_APBENR2_USART1EN;
    /* Configure USART1 */
    /* System clock is 12MHz, 19200 baud (both already divided by 100) */
    USART1->BRR = (256*120000)/192;//19200??

    USART1->CR2 |= USART_CR2_SWAP; /* Swap RX and TX this needs to be set before CR1_UE is set */

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    /* Extra info                                 */
    /* USART1->CR1 = USART1 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_UE = USART Enable                */

    /* polling idle frame Transmission */
    tickstart = Tick;
    while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC){
    	if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			error = ERROR_USART_TIMEOUT;
			return;
		}
    }
	USART1->ICR |= USART_ICR_TCCF;  /* Clear TC flag  (no bit for receive) */
	USART1->CR1 |= USART_CR1_TCIE | USART_CR1_RXFFIE; /* Enable Transmission Complete and receive interrupt */

	/* Configure Interrupt */
	/* Set priority for USART1_IRQn */
	NVIC_SetPriority(USART1_IRQn, 0);
	/* Enable USART1_IRQn */
	NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_writeByte( uint8_t data ){
	if( irqStatus_USART1 & IDLE ){// Meaning buffer not in use
		irqStatus_USART1 = BUSY;  // Show that it's in use now
		if( outEnd_USART1==outStart_USART1 ){ // Buffer is empty
			USART1->TDR = data; // Write the byte
			return; // Finished here
		}else{ // Buffer isn't empty but the irq is in idle so TXE empty, get it to work
			USART1->TDR = *outStart_USART1++;
			if (outStart_USART1 == outTail_USART1) {  // So never write on the tail!
				outStart_USART1 = outHead_USART1;     // End reached, back to head
			}
		}
	}
	// At this point the irq is busy with 'something'
	if( outEnd_USART1+1 == outStart_USART1 || (outStart_USART1==outHead_USART1 && outEnd_USART1+1==outTail_USART1 )){
		// Getting in here means that adding a byte would overflow the buffer, so wait a bit
		irqStatus_USART1 |= WAITING; // Put up the flag that we are waiting for a byte to transfer
		uint32_t tickstart = Tick;
		while( irqStatus_USART1 & WAITING ){ // This can be infinite?
			if ((Tick - tickstart ) > 50){
				return; // Buffer doesn't empty?
			}
		}
	}
	*outEnd_USART1++ = data;
	if (outEnd_USART1 == outTail_USART1) // So never write on the tail!
		outEnd_USART1 = outHead_USART1;  // End reached, back to head
}

void USART1_Transfer_Buffer( void ){
    uint8_t rec[16];
    uint8_t a;

    if( inputStart_USART1 == inputEnd_USART1) // This shouldn't happen, but somehow does
    	return;

    /* Clean the buffer that will be used */
    for( a=0;a<16;a++)
        rec[a]=0x00;

    a = 0x00;
    /* Move the data from the circular buffer to the local one */
    inputTemp_USART1 = inputEnd_USART1;                             // Remember the current endstop for the circular buffer,because other might use it in ISR
    if (inputStart_USART1 > inputTemp_USART1) {                     // If the 'end' is closer to the beginning of the buffer than the 'start'
        do{
            rec[a++] = *inputStart_USART1++;
        }while( inputStart_USART1 != inputTail_USART1 && a < CIRCULAR+5);  // Repeat till the 'start' has reached the end of the buffer
        inputStart_USART1 = inputHead_USART1;                       // Because the end was reached, continue from the start of the buffer
    }
    do{
        rec[a++] = *inputStart_USART1++;
    }while( inputStart_USART1 < inputTemp_USART1 && a < CIRCULAR+5);      // Repeat the 'start' is the same as the 'end'
}
void USART1_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    // Check if interrupt is due to transmit complete
    if( USART1->ISR & USART_ISR_TXE_TXFNF ){
    	if( outStart_USART1 != outEnd_USART1 ){ // meaning still data to send
			 USART1->TDR = *outStart_USART1++; // This also clears the complete flag
			 freeSpace_USART1++;
			 irqStatus_USART1=BUSY; // Indicate that the irq is working on the buffer
			 if (outStart_USART1 == outTail_USART1) { // So never write on the tail!
				 outStart_USART1 = outHead_USART1;    // End reached, back to head
			 }
		 }else{ // No more data to send, just clear the flag
			 USART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
			 irqStatus_USART1=IDLE;
		 }
    }
    if( USART1->ISR & USART_ISR_TC ){
    	USART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART1->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
        recChar = (uint8_t)(USART1->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *inputEnd_USART1++ = recChar;
            if (inputEnd_USART1 == inputTail_USART1) { // So never write on the tail!
                inputEnd_USART1 = inputHead_USART1;
            }
        }else if(recChar==0x00){
        	ok=0x00;
        }else if( recChar == '\n'){
        }else if( recChar == '\r'){
        	cmdReady ++;
        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //USART1_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
    }
}
