import RPi.GPIO as GPIO

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

set_motor1(50,"forward")
set_motor2(50,"forward")

# set_motor2(50,"backward")
# set_motor1(50,"backward")

# Function to make the robot move forward
def move_forward(speed, signal):
  
  if not signal:
    # Set the speed and direction of both motors
    set_motor1(speed, "forward")
    set_motor2(speed, "forward")
  else:
    # Stop both motors
    set_motor1(0, "stop")
    set_motor2(0, "stop")

 
# Function to make the robot move backward
def move_backward(speed, signal):
  # Set the speed and direction of both motors
  set_motor1(speed, "backward")
  set_motor2(speed, "backward")
 
  # Loop until the signal is received
  while not signal.is_set():
    continue
 
  # Stop both motors
  set_motor1(0, "stop")
  set_motor2(0, "stop")

# Function to make the robot turn left
def turn_left(speed, signal):
  # Set the speed and direction of motor 1
  set_motor1(speed, "backward")
 
  # Set the speed and direction of motor 2
  set_motor2(speed, "forward")
 
  # Loop until the signal is received
  while not signal.is_set():
    continue
 
  # Stop both motors
  set_motor1(0, "stop")
  set_motor2(0, "stop")

# Function to make the robot turn right
def turn_right(speed, signal):
  # Set the speed and direction of motor 1
  set_motor1(speed, "forward")
 
  # Set the speed and direction of motor 2
  set_motor2(speed, "backward")
 
  # Loop until the signal is received
  while not signal.is_set():
    continue
 
  # Stop both motors
  set_motor1(0, "stop")
  set_motor2(0, "stop")


