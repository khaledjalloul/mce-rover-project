# Import the necessary libraries
import RPi.GPIO as GPIO

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set the GPIO pins that the encoder is connected to
encoder_pin_a = 17
encoder_pin_b = 18
encoder_pin_s = 27

# Set the encoder pins as inputs
GPIO.setup(encoder_pin_a, GPIO.IN)
GPIO.setup(encoder_pin_b, GPIO.IN)
GPIO.setup(encoder_pin_s, GPIO.IN)

# Create a variable to store the encoder value
encoder_value = 0

# Define a function to be called whenever the encoder value changes
def update_encoder_value(channel):
  global encoder_value
  if GPIO.input(encoder_pin_a) == GPIO.HIGH:
    encoder_value += 1
  else:
    encoder_value -= 1

# Set the encoder_pin_s as an interrupt that triggers the update_encoder_value function
GPIO.add_event_detect(encoder_pin_s, GPIO.BOTH, callback=update_encoder_value)

# Print the initial encoder value
print(encoder_value)

# Do something else in your code...

# Print the final encoder value
print(encoder_value)

# Clean up the GPIO settings
GPIO.cleanup()


'''
 In this code, we import the RPi.GPIO library and set the GPIO mode to BCM. We then specify the GPIO pins that the encoder is connected to, and set them as inputs using the GPIO.setup() function.

Next, we create a variable to store the encoder value and define a function that will be called whenever the encoder value changes. This function checks the state of the A and B output pins on the encoder and updates the encoder_value variable accordingly.

We then set the S pin on the encoder as an interrupt that triggers the update_encoder_value() function whenever the encoder value changes. This allows us to automatically update the encoder_value variable as the encoder is turned.

Finally, we print the initial and final encoder values and clean up the GPIO settings by calling the GPIO.cleanup() function.
 '''