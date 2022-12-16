# MCE Project

## Pump

In this code, we import the RPi.GPIO and time libraries, which provide access to the GPIO pins and the ability to pause the program execution, respectively. We then set the GPIO mode to BCM and the relay pin as an output.

Next, we turn the water pump on by setting the relay pin to a high logic level using the GPIO.output() function. We then pause the program for 5 seconds using the time.sleep() function, before turning the water pump off by setting the relay pin to a low logic level.

Finally, we clean up the GPIO settings by calling the GPIO.cleanup() function. This ensures that the GPIO pins are reset and available for use in other programs.

## Flame Sensor

This modified version of the code uses three GPIO pins to connect to the LM393 3 pin flame sensor: one for the sensor's signal output, one for the sensor's ground, and one for the sensor's VCC (power). It then uses these pins to read the value of the flame sensor and determine if a flame is detected.

## Encoder

In this code, we import the RPi.GPIO library and set the GPIO mode to BCM. We then specify the GPIO pins that the encoder is connected to, and set them as inputs using the GPIO.setup() function.

Next, we create a variable to store the encoder value and define a function that will be called whenever the encoder value changes. This function checks the state of the A and B output pins on the encoder and updates the encoder_value variable accordingly.

We then set the S pin on the encoder as an interrupt that triggers the update_encoder_value() function whenever the encoder value changes. This allows us to automatically update the encoder_value variable as the encoder is turned.

Finally, we print the initial and final encoder values and clean up the GPIO settings by calling the GPIO.cleanup() function.
