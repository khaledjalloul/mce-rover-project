import RPi.GPIO as GPIO

# Set up the GPIO pins for the motor driver
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)

# Define the functions for the four motors
def motor1_forward(speed):
    pwm = GPIO.PWM(11, speed)
    pwm.start(0)
    pwm.ChangeDutyCycle(speed)

def motor1_reverse(speed):
    pwm = GPIO.PWM(12, speed)
    pwm.start(0)
    pwm.ChangeDutyCycle(speed)

def motor2_forward(speed):
    pwm = GPIO.PWM(15, speed)
    pwm.start(0)
    pwm.ChangeDutyCycle(speed)

def motor2_reverse(speed):
    pwm = GPIO.PWM(16, speed)
    pwm.start(0)
    pwm.ChangeDutyCycle(speed)

# Define the functions for the four directions
def forward(speed):
    motor1_forward(speed)
    motor2_forward(speed)

def reverse(speed):
    motor1_reverse(speed)
    motor2_reverse(speed)

def left(speed):
    motor1_reverse(speed)
    motor2_forward(speed)

def right(speed):
    motor1_forward(speed)
    motor2_reverse(speed)

# Move the rover in the desired direction at the desired speed
direction = input("Enter direction (forward, reverse, left, right): ")
speed = int(input("Enter speed (1-100): "))

if direction == "forward":
    forward(speed)
elif direction == "reverse":