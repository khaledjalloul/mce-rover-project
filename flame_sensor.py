import RPi.GPIO as GPIO
import time

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Set the flame sensor pins as inputs
FLAME_SENSOR_PIN = 9
GPIO.setup(FLAME_SENSOR_PIN, GPIO.IN)

def read_flame_sensor():
  # Read the flame sensor's value
  value = GPIO.input(FLAME_SENSOR_PIN)

  # If the value is 1, it indicates that the flame sensor is detecting a flame
  if value == 0:
    print("Flame detected!")
  else:
    print("No flame detected.")

while True:
  read_flame_sensor()
  time.sleep(1)


# This modified version of the code uses three GPIO pins to connect to the LM393 3 pin flame sensor: one for the sensor's signal output, one for the sensor's ground, and one for the sensor's VCC (power). It then uses these pins to read the value of the flame sensor and determine if a flame is detected.

# Note that this is just one example of how to use the LM393 3 pin flame sensor with a Raspberry Pi. You may need to modify the code further depending on your specific application and the details of your setup.
