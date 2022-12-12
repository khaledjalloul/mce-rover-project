import RPi.GPIO as GPIO

# Set up the Raspberry Pi to use the GPIO pin numbering scheme
GPIO.setmode(GPIO.BCM)

# Set up the two input pins for the encoders
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Define a function to be called when an encoder produces an input
def encoder_callback(channel):
  # Print the channel number of the encoder that produced the input
  print("Encoder input on channel: ", channel)

# Set up the Raspberry Pi to call the encoder_callback function
# whenever an encoder produces an input
GPIO.add_event_detect(17, GPIO.RISING, callback=encoder_callback)
GPIO.add_event_detect(18, GPIO.RISING, callback=encoder_callback)

# Continue reading inputs from the encoders indefinitely
while True:
  # This loop will run continuously and the encoder_callback function
  # will be called whenever an encoder produces an input
  pass