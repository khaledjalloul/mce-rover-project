import RPi.GPIO as GPIO

# Set up GPIO using BCM numbering
GPIO.setmode(GPIO.BCM)

# Set up pin 18 as an input with a pull-down resistor
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set up a counter to keep track of the number of pulses
counter = 0

# Define a function to be called when a rising edge is detected on pin 18
def pulse_count(channel):
    global counter
    counter += 1

# Set up an interrupt to watch for rising edges on pin 18
GPIO.add_event_detect(18, GPIO.RISING, callback=pulse_count)

# Print the counter value every second
while True:
    print(counter)
    sleep(1)


# This code sets up GPIO pin 18 on the Raspberry Pi as an input, with a pull-down resistor to ensure a stable low level when the encoder is not sending pulses. It then sets up a counter variable to keep track of the number of pulses received, and defines a function that will be called whenever a rising edge is detected on the pin. This function increments the counter by one.

# Finally, the code sets up an interrupt to watch for rising edges on pin 18 and calls the pulse_count function whenever one is detected. It then enters an infinite loop, printing the current value of the counter every second.

# Note that this is just an example, and you may need to modify the code depending on your specific setup and requirements. For example, you may need to use a different GPIO pin, or you may want to adjust the pulse detection threshold or debounce time. You may also need to adjust the code to handle different types of encoders (e.g. quadrature encoders) or to read data in a different format (e.g. pulse width or frequency).
