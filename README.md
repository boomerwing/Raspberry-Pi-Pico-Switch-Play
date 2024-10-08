# Raspberry-Pi-Pico-Switch-Play
This Raspberry-Pi Pico FreeRTOS sumulation measures a voltage and displays a result on a seven seg LED.  There is an alarm boundary which blinks the LED if the bountary is crossed.

The active components are the PICO, A 16 port PCF8575 port extender, a Seven Segment LED, two LEDs, eight switches and a potentiometer to create the input voltage to be read by the ADC and displayed.

Four versions of the experiment are included as I played with the tasks and refined how they would work (main_i2c-D.c is the final version.)

Four tasks provide the functionality: adc task, blink task, gpio led task and switch debounce task.  The blink task can be used to blink leds in any other task. The switch debounce task debounces eight switch inputs at once.

As an interesting use of one of the switch outputs, the speed of the blink is changed by the state of switch sw0.

