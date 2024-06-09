# I2C to USART with STM32C0
Firmware for STM32C0 with CORE M0+ to use the usarts via I²C. 
Because I wanted a cheaper alternative to the purpose built chips that start at 6€.

## Progress
- Testing with Rev0 hardware
- Designing rev1, which mainly adds led's for feedback.
- Firmware.
  - TX via DMA works
  - RX via isr works
  - todo: changing uart settings
- Writing the code for [dcafs](https://github.com/michieltjampens/dcafs) to use it, decided to rewrite the I²C part for it.
  - Sending data works.
  - todo: processing received data
  - todo: changing settings
    
## Info
### Hardware
- Starting with the [STM32C011F4U6TR](https://www.st.com/en/microcontrollers-microprocessors/stm32c011f4.html).
   - Program memory: 16kB (26% used)
   - Ram memory: 6kB (37% used, mainly for 512Byte circular buffers)
   - I²C: 1x FM+
   - USART x2
   - Costs about 1,2€ ex vat
### Firmware
- Bare metal C code (so no LL or HAL).
- I2C to uart is handled by DMA.
