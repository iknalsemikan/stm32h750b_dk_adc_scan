# stm32h750b_dk_adc_scan
An example of how to run ADC scan mode on a STM32H750 processor.

This project uses the STM32H750B-DK development kit from STM microelectronics.

In this project the ADC3 is configuered in scan mode. It uses the Timer TIM2 to sample at 100 Hz.
The ADC3 scans 4 analogue signals and uses DMA to copy the data from the ADC3->DR register into a buffer.

The buffer is a uint16_t buffer with the double of analogue signals, this means 8.

When the DMA half buffer interrupt is triggered the signals are read for further processing.

Configuration of ADC
ADC3 is used for this example.
The ADC is configured to 12 bits resolution.
Scan conversion mode: Enabled
End Of Conversion Selection: End of sequence of conversion
Conversion Data Management Mode: DMA Circular Mode

Signals to be monitored:
1. Channel7
1. Channel Vbat
1. Channel Temperature Sensor
1. Channel Vrefint

