# Import necessary libraries
import RPi.GPIO as GPIO
import time

# Set GPIO mode and disable warnings
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Set the pins for the motor driver
motor1_in1 = 5
motor1_in2 = 6
motor1_en = 13
motor2_in1 = 19
motor2_in2 = 26
motor2_en = 16

# Set the pins as outputs
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)
GPIO.setup(motor1_en, GPIO.OUT)
GPIO.setup(motor2_in1, GPIO.OUT)
GPIO.setup(motor2_in2, GPIO.OUT)
GPIO.setup(motor2_en, GPIO.OUT)

# Create PWM instances for each motor
motor1 = GPIO.PWM(motor1_en, 100)
motor2 = GPIO.PWM(motor2_en, 100)

# Start the PWM with a duty cycle of 0 (motors are off)
motor1.start(0)
motor2.start(0)

# Function to set the speed of the motors
def set_speed(speed):
    # Set the duty cycle to the desired speed
    motor1.ChangeDutyCycle(speed)
    motor2.ChangeDutyCycle(speed)

try:
    while True:
        # Get the desired speed from the user
        speed = float(input("Enter a speed (0-100): "))

        # Set the speed of the motors
        set_speed(speed)

except KeyboardInterrupt:
    # Stop the motors
    motor1.stop()
    motor2.stop()
    # Clean up the GPIO pins
    GPIO.cleanup()