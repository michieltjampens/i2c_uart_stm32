/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "../inc/usart2.h"
#include "../inc/main.h"

/* Shared variables ----------------------------------------------------------*/
uint16_t freeSpace_USART2 = CIRCULAR-1;
uint8_t irqStatus_USART2=IDLE;

/* Private variables ---------------------------------------------------------*/

/* Circular buffer for UART OUT*/
uint8_t outputBuffer_USART2[CIRCULAR];
uint8_t *outTemp_USART2;
uint8_t *outStart_USART2;
uint8_t *outEnd_USART2;
uint8_t *outHead_USART2;
uint8_t *outTail_USART2;

/* Circular buffer for UART IN*/
uint8_t inputBuffer_USART2[CIRCULAR];
uint8_t *inputTemp_USART2;
uint8_t *inputStart_USART2;
uint8_t *inputEnd_USART2;
uint8_t *inputHead_USART2;
uint8_t *inputTail_USART2;


void USART2_Configure(){
    /* Initialise the circular buffers */
    inputStart_USART2 = &inputBuffer_USART2[0];
    inputEnd_USART2 =   &inputBuffer_USART2[0];
    inputHead_USART2 =  &inputBuffer_USART2[0];
    inputTail_USART2 =  &inputBuffer_USART2[CIRCULAR-1];

    outStart_USART2 = &outputBuffer_USART2[0];
    outEnd_USART2  =   &outputBuffer_USART2[0];
    outHead_USART2 =  &outputBuffer_USART2[0];
    outTail_USART2 =  &outputBuffer_USART2[CIRCULAR-1];

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
	uint32_t tickstart;

    /* Enable the peripheral clock USART2 */
    RCC->APBENR1 |= RCC_APBENR2_USART1EN;
    /* Configure USART2 */
    /* System clock is 16MHz (1MHz oversampling by 16), 19200 baud (both already divided by 100) */
    USART2->BRR = (256*160000)/192;//19200??

    USART2->CR2 |= USART_CR2_SWAP; /* Swap RX and TX this needs to be set before CR1_UE is set */

    /* 8 data bit, 1 start bit, 1 stop bit, no parity */
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    /* Extra info                                 */
    /* USART2->CR1 = USART2 Control register      */
    /* 8/1/1 no parity is default                 */
    /* USART_CR1_RE = Receiver enable             */
    /* USART_CR1_TE = Transmitter Enable          */
    /* USART_CR1_UE = USART Enable                */

    /* polling idle frame Transmission */
    tickstart = Tick;
    while((USART2->ISR & USART_ISR_TC) != USART_ISR_TC){
    	if ((Tick - tickstart ) > I2C_TIMEOUT_VALUE){
			error = ERROR_USART_TIMEOUT;
			return;
		}
    }
	USART2->ICR |= USART_ICR_TCCF;  /* Clear TC flag  (no bit for receive) */
	USART2->CR1 |= USART_CR1_TCIE | USART_CR1_RXFFIE; /* Enable Transmission Complete and receive interrupt */

	/* Configure Interrupt */
	/* Set priority for USART2_IRQn */
	NVIC_SetPriority(USART2_IRQn, 0);
	/* Enable USART2_IRQn */
	NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_writeByte( uint8_t data ){
	if( irqStatus_USART2 & IDLE ){// Meaning buffer not in use
		irqStatus_USART2 = BUSY;  // Show that it's in use now
		if( outEnd_USART2==outStart_USART2 ){ // Buffer is empty
			USART2->TDR = data; // Write the byte
			return; // Finished here
		}else{ // Buffer isn't empty but the irq is in idle so TXE empty, get it to work
			USART2->TDR = *outStart_USART2++;
			if (outStart_USART2 == outTail_USART2) {  // So never write on the tail!
				outStart_USART2 = outHead_USART2;     // End reached, back to head
			}
		}
	}
	// At this point the irq is busy with 'something'
	if( outEnd_USART2+1 == outStart_USART2 || (outStart_USART2==outHead_USART2 && outEnd_USART2+1==outTail_USART2 )){
		// Getting in here means that adding a byte would overflow the buffer, so wait a bit
		irqStatus_USART2 |= WAITING; // Put up the flag that we are waiting for a byte to transfer
		uint32_t tickstart = Tick;
		while( irqStatus_USART2 & WAITING ){ // This can be infinite?
			if ((Tick - tickstart ) > 50){
				return; // Buffer doesn't empty?
			}
		}
	}
	*outEnd_USART2++ = data;
	if (outEnd_USART2 == outTail_USART2) // So never write on the tail!
		outEnd_USART2 = outHead_USART2;  // End reached, back to head
}

void USART2_Transfer_Buffer( void ){
    uint8_t rec[16];
    uint8_t a;

    if( inputStart_USART2 == inputEnd_USART2) // This shouldn't happen, but somehow does
    	return;

    /* Clean the buffer that will be used */
    for( a=0;a<16;a++)
        rec[a]=0x00;

    a = 0x00;
    /* Move the data from the circular buffer to the local one */
    inputTemp_USART2 = inputEnd_USART2;                             // Remember the current endstop for the circular buffer,because other might use it in ISR
    if (inputStart_USART2 > inputTemp_USART2) {                     // If the 'end' is closer to the beginning of the buffer than the 'start'
        do{
            rec[a++] = *inputStart_USART2++;
        }while( inputStart_USART2 != inputTail_USART2 && a < CIRCULAR+5);  // Repeat till the 'start' has reached the end of the buffer
        inputStart_USART2 = inputHead_USART2;                       // Because the end was reached, continue from the start of the buffer
    }
    do{
        rec[a++] = *inputStart_USART2++;
    }while( inputStart_USART2 < inputTemp_USART2 && a < CIRCULAR+5);      // Repeat the 'start' is the same as the 'end'
}
void USART2_IRQHandler(void){
    uint8_t recChar = 0;
    uint8_t ok = 0x01;

    // Check if interrupt is due to transmit complete
    if( USART2->ISR & USART_ISR_TXE_TXFNF ){
    	if( outStart_USART2 != outEnd_USART2 ){ // meaning still data to send
			 USART2->TDR = *outStart_USART2++; // This also clears the complete flag
			 freeSpace_USART2++;
			 irqStatus_USART2=BUSY; // Indicate that the irq is working on the buffer
			 if (outStart_USART2 == outTail_USART2) { // So never write on the tail!
				 outStart_USART2 = outHead_USART2;    // End reached, back to head
			 }
		 }else{ // No more data to send, just clear the flag
			 USART2->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
			 irqStatus_USART2=IDLE;
		 }
    }
    if( USART2->ISR & USART_ISR_TC ){
    	USART2->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    }
    if((USART2->ISR & USART_ISR_RXNE_RXFNE) == USART_ISR_RXNE_RXFNE){ // ISR for received data
        recChar = (uint8_t)(USART2->RDR); /* Receive data, clear flag */
        if( recChar >= 0x20 && recChar <= 0x7F){
            *inputEnd_USART2++ = recChar;
            if (inputEnd_USART2 == inputTail_USART2) { // So never write on the tail!
                inputEnd_USART2 = inputHead_USART2;
            }
        }else if(recChar==0x00){
        	ok=0x00;
        }else if( recChar == '\n'){
        }else if( recChar == '\r'){

        }
        // Can't echo inside the ISR! because the processor doesn't process fast enough?
        //USART2_SendByte(recChar); // This gives issues
    }
    if( ok==0x00 ){ // Meaning ISR was for unknown reason
        error = ERROR_USART_TRANSMIT; /* Report an error */
        NVIC_DisableIRQ(USART2_IRQn); /* Disable USART2_IRQn */
    }
}
