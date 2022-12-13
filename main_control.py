import RPi.GPIO as GPIO
import math
import time

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

# Set the pins for the motor driver inputs
motor1_in1 = 17
motor1_in2 = 18
motor2_in1 = 27
motor2_in2 = 22

# Set the pins for the motor driver outputs
motor1_en = 23
motor2_en = 24

# Set the GPIO pins that the encoder is connected to
encoder_pin = 17

# Encoder parameters
encoder_val = 0
old_time = 0
pulses_per_rev = 20
wheel_circumference = 2 * math.pi * 6.5 # in cm
pwm_speed = 25

# Set the encoder pins as inputs
GPIO.setup(encoder_pin, GPIO.IN)

# Set the GPIO pins for the motor driver inputs as outputs
GPIO.setup(motor1_in1, GPIO.OUT)
GPIO.setup(motor1_in2, GPIO.OUT)
GPIO.setup(motor2_in1, GPIO.OUT)
GPIO.setup(motor2_in2, GPIO.OUT)

# Set the GPIO pins for the motor driver outputs as outputs
GPIO.setup(motor1_en, GPIO.OUT)
GPIO.setup(motor2_en, GPIO.OUT)

# Create PWM channels for the motor driver outputs with a frequency of 100 Hz
motor1_pwm = GPIO.PWM(motor1_en, 100)
motor2_pwm = GPIO.PWM(motor2_en, 100)

# Start the PWM channels with a duty cycle of 0
motor1_pwm.start(0)
motor2_pwm.start(0)

# Function to control the speed and direction of motor 1
def set_motor1(speed, direction):
  if direction == "forward":
    # Set the input pins to control the motor direction
    GPIO.output(motor1_in1, GPIO.HIGH)
    GPIO.output(motor1_in2, GPIO.LOW)
  elif direction == "backward":
    # Set the input pins to control the motor direction
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.HIGH)
  else:
    # Stop the motor
    GPIO.output(motor1_in1, GPIO.LOW)
    GPIO.output(motor1_in2, GPIO.LOW)
 
  # Set the duty cycle of the PWM channel
  motor1_pwm.ChangeDutyCycle(speed)

# Function to control the speed and direction of motor 2
def set_motor2(speed, direction):
  if direction == "forward":
    # Set the input pins to control the motor direction
    GPIO.output(motor2_in1, GPIO.HIGH)
    GPIO.output(motor2_in2, GPIO.LOW)
  elif direction == "backward":
    # Set the input pins to control the motor direction
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.HIGH)
  else:
    # Stop the motor
    GPIO.output(motor2_in1, GPIO.LOW)
    GPIO.output(motor2_in2, GPIO.LOW)
 
  # Set the duty cycle of the PWM channel
  motor2_pwm.ChangeDutyCycle(speed)

# Function to make the robot move forward
def move_forward(speed):
  # Set the speed and direction of both motors
  set_motor1(speed, "forward")
  set_motor2(speed, "forward")

# Function to make the robot move backward
def move_backward(speed):
  # Set the speed and direction of both motors
  set_motor1(speed, "backward")
  set_motor2(speed, "backward")

# Function to make the robot turn left
def turn_left(speed):
  # Set the speed and direction of motor 1
  set_motor1(speed, "backward")
 
  # Set the speed and direction of motor 2
  set_motor2(speed, "forward")

# Function to make the robot turn right
def turn_right(speed):
  # Set the speed and direction of motor 1
  set_motor1(speed, "forward")
 
  # Set the speed and direction of motor 2
  set_motor2(speed, "backward")

def stop_motors():
  set_motor1(0, 'stop')
  set_motor2(0, 'stop')

# Define a function to be called whenever the encoder value changes
def update_encoder_value():
  global encoder_val
  if GPIO.input(encoder_pin) == GPIO.HIGH:
    encoder_val += 1

def get_linear_vel():
  global current_time, old_time
  current_time = time.time()
  if current_time - old_time >= 3:
      pulses_per_sec = encoder_val / (current_time - old_time)
      rev_per_sec = pulses_per_sec / pulses_per_rev
      linear_vel = rev_per_sec * wheel_circumference
      old_time = time.time()
      return linear_vel

def adjust_speed(linear_vel):
  pass

if __name__ == '__main__':
  GPIO.add_event_detect(encoder_pin, GPIO.HIGH, callback=update_encoder_value)
  print("Moving forward.")
  move_forward(pwm_speed)
  main_time = time.time() + 10
  while time.time() < main_time:
    current_vel = get_linear_vel()
    print(current_vel)


