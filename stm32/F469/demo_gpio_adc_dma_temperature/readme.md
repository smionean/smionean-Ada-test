This program demonstrates reading the temperature from
an ADC unit, using DMA with LM35 sensor.

Note that you may need to enable optimization (see the scenario variable) for
proper execution.

This program is expected to run on the STM32F469 Discovery boards because
it uses the LCD to display results.  This display usage is entirely for
convenience -- some other means for showing the results could be used, such
as a serial port, or some other board with a different display.

Both raw (sensed) and scaled values are continuously displayed.

Note that you will very likely need to reset the board manually for proper
execution.