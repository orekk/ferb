## <b>SAI_AudioRecord_PDM_LowPower Example Description</b>

  This project demonstrates how to configure SAI to perform PDM record in low power sleep mode.

  SAI1_BlockA is configured in master reception with PDM activated.

  Two digital microphones of X-NUCLEO-CCA02M2 (digital mems microphone expansion board) are used on this example.
  X-NUCLEO-CCA02M2 is connected but not plugged on NUCLEO-U385RG-Q board.
  Following wired connections are needed:
  - CN7 pin 37 of NUCLEO-U385RG-Q (SAI1_D1) needs to be connected to CN10 pin 26 of X-NUCLEO-CCA02M2 (MIC_PDM12).
  - CN10 pin 37 of NUCLEO-U385RG-Q (SAI1_CK1) needs to be connected to CN10 pin 29 of X-NUCLEO-CCA02M2 (MIC_CLK_NUCLEO).
  - CN6 pin 4 of NUCLEO-U385RG-Q (3V3) needs to be connected to CN6 pin 4 of X-NUCLEO-CCA02M2 (3V3).
  - CN6 pin 6 of NUCLEO-U385RG-Q (GND) needs to be connected to CN6 pin 6 of X-NUCLEO-CCA02M2 (GND).

  On this example, SAI1 kernel clock is set to MSIK and MSIK is configured at 48MHZ.
  In order to have a PDM frequency at arroung 1MHz, SAI master clock divider is set to 24 (MckDiv = Fsai/(Fpdmx2)).
  PDM samples will be stored on PdmBuffer with circular DMA.

  PDM to PCM library will be used to transform PDM samples to PCM samples.
  With a decimation of 64, we will obtain PCM samples at around 16KHz (1MHz/64).

  PCM samples will be stored in loop on PcmBuffer that contains 2 seconds of recording data.
  After 5 loops (10 seconds of record), record stops and green led of board turns on.

  Last 2 seconds of record (PcmBuffer) can be downloaded with STM32CubeProgrammer.

  If error occurs during previous steps, green led toggles.

  In order to reduce power consumption, the system clock frequency is set to 12Mhz.
  After each PDM data treatments, CPU enters in sleep mode until new DMA tranfert callback.
  In addition, SysTick interrupt is disabled to avoid wake-up each millisecond.

  With SAI_AudioRecord_PDM example, power consumption during audio record is around 6,1mA.
  With this SAI_AudioRecord_PDM_LowPower example, power consumption during audio record is around 1,2mA.

  This project is targeted to run on STM32U385RG device on NUCLEO-U385RG-Q board from STMicroelectronics.

#### <b>Notes</b>

 1. Care must be taken when using HAL_Delay(), this function provides accurate delay (in milliseconds)
    based on variable incremented in SysTick ISR. This implies that if HAL_Delay() is called from
    a peripheral ISR process, then the SysTick interrupt must have higher priority (numerically lower)
    than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
    To change the SysTick interrupt priority you have to use HAL_NVIC_SetPriority() function.
 2. The application needs to ensure that the SysTick time base is always set to 1 millisecond
    to have correct HAL operation.
 3. The instruction cache (ICACHE) must be enabled by software to get a 0 wait-state execution
    from Flash memory and external memories, and reach the maximum performance.

### <b>Keywords</b>

Audio, SAI, DMA, PDM, PCM, Low Power, Sleep mode

### <b>Directory contents</b>

	- SAI/SAI_AudioRecord_PDM_LowPower/Src/main.c                   Main program
	- SAI/SAI_AudioRecord_PDM_LowPower/Src/system_stm32u3xx.c       STM32U3xx system clock configuration file
	- SAI/SAI_AudioRecord_PDM_LowPower/Src/stm32u3xx_it.c           Interrupt handlers
	- SAI/SAI_AudioRecord_PDM_LowPower/Src/stm32u3xx_hal_msp.c      HAL MSP module
	- SAI/SAI_AudioRecord_PDM_LowPower/Inc/main.h                   Main program header file
	- SAI/SAI_AudioRecord_PDM_LowPower/Inc/stm32u3xx_nucleo_conf.h  BSP Configuration file
	- SAI/SAI_AudioRecord_PDM_LowPower/Inc/stm32u3xx_hal_conf.h     HAL Configuration file
	- SAI/SAI_AudioRecord_PDM_LowPower/Inc/stm32u3xx_it.h           Interrupt handlers header file

### <b>Hardware and Software environment</b>

  - This example runs on STM32U385RG devices without security enabled (TZEN=0).
  - This example has been tested with STMicroelectronics NUCLEO-U385RG-Q (MB1841)
    board and can be easily tailored to any other supported device
    and development board.

### <b>How to use it ?</b>

In order to make the program work, you must do the following :

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the example

