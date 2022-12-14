# Import the necessary libraries
import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set the GPIO pin that the relay is connected to
relay_pin = 10

# Set the relay pin as an output
GPIO.setup(relay_pin, GPIO.OUT)

# Turn the water pump on
GPIO.output(relay_pin, GPIO.HIGH)

# Wait for 5 seconds
time.sleep(20)

# Turn the water pump off
GPIO.output(relay_pin, GPIO.LOW)

# Clean up the GPIO settings
GPIO.cleanup()


'''
n this code, we import the RPi.GPIO and time libraries, which provide access to the GPIO pins and the ability to pause the program execution, respectively. We then set the GPIO mode to BCM and the relay pin as an output.

Next, we turn the water pump on by setting the relay pin to a high logic level using the GPIO.output() function. We then pause the program for 5 seconds using the time.sleep() function, before turning the water pump off by setting the relay pin to a low logic level.

Finally, we clean up the GPIO settings by calling the GPIO.cleanup() function. This ensures that the GPIO pins are reset and available for use in other programs.
'''
