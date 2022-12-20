import RPi.GPIO as GPIO
import math
import time
from simple_pid import PID

GPIO.setmode(GPIO.BCM)

# Set the pins for the motor driver inputs
MOTOR1_IN1 = 17
MOTOR1_IN2 = 18
MOTOR2_IN1 = 27
MOTOR2_IN2 = 22

# Set the pins for the motor driver outputs
MOTOR1_EN = 23
MOTOR2_EN = 24

# Set the GPIO pins that the encoder is connected to
ENCODER_PIN = 3
ENCODER_PIN2 = 4

# Set the encoder pins as inputs
GPIO.setup(ENCODER_PIN, GPIO.IN)
GPIO.setup(ENCODER_PIN2, GPIO.IN)

# Set the GPIO pins for the motor driver inputs as outputs
GPIO.setup(MOTOR1_IN1, GPIO.OUT)
GPIO.setup(MOTOR1_IN2, GPIO.OUT)
GPIO.setup(MOTOR2_IN1, GPIO.OUT)
GPIO.setup(MOTOR2_IN2, GPIO.OUT)

# Set the GPIO pins for the motor driver outputs as outputs
GPIO.setup(MOTOR1_EN, GPIO.OUT)
GPIO.setup(MOTOR2_EN, GPIO.OUT)


class MotorController:

    def __init__(self):

        # Encoder parameters
        self.encoder_val = 0
        self.encoder_val2 = 0
        self.old_time = 0
        self.pulses_per_rev = 20
        self.wheel_circumference = math.pi * 6.5  # in cm
        self.pwm_speed = 40
        self.pwm_speed2 = 40
        self.linear_vel = 0
        self.linear_vel2 = 0
        self.update_speed = False

        # Create PWM channels for the motor driver outputs with a frequency of 100 Hz
        self.motor1_pwm = GPIO.PWM(MOTOR1_EN, 100)
        self.motor2_pwm = GPIO.PWM(MOTOR2_EN, 100)

        # Start the PWM channels with a duty cycle of 0
        self.motor1_pwm.start(self.pwm_speed)
        self.motor2_pwm.start(self.pwm_speed2)

        self.pid = PID(1, 0, 0, setpoint=25)  # Target cm/s
        self.pid.sample_time = 0.1

        # Attach interrupts to count encoder pulses
        GPIO.add_event_detect(ENCODER_PIN, GPIO.RISING,
                              callback=self.update_encoder_value)
        GPIO.add_event_detect(ENCODER_PIN2, GPIO.RISING,
                              callback=self.update_encoder_value)

    # Function to control the speed and direction of motor 1

    def set_motor1(self, direction):
        if direction == "forward":
            GPIO.output(MOTOR1_IN1, GPIO.HIGH)
            GPIO.output(MOTOR1_IN2, GPIO.LOW)
        elif direction == "backward":
            GPIO.output(MOTOR1_IN1, GPIO.LOW)
            GPIO.output(MOTOR1_IN2, GPIO.HIGH)
        else:
            # Stop the motor
            GPIO.output(MOTOR1_IN1, GPIO.LOW)
            GPIO.output(MOTOR1_IN2, GPIO.LOW)

    # Function to control the speed and direction of motor 2

    def set_motor2(self, direction):
        if direction == "forward":
            GPIO.output(MOTOR2_IN1, GPIO.HIGH)
            GPIO.output(MOTOR2_IN2, GPIO.LOW)
        elif direction == "backward":
            GPIO.output(MOTOR2_IN1, GPIO.LOW)
            GPIO.output(MOTOR2_IN2, GPIO.HIGH)
        else:
            GPIO.output(MOTOR2_IN1, GPIO.LOW)
            GPIO.output(MOTOR2_IN2, GPIO.LOW)

    # Function to make the robot move in any direction

    def move(self, direction):

        if direction == 'forward':
            self.set_motor1('forward')
            self.set_motor2('forward')

        elif direction == 'backward':
            self.set_motor1('backward')
            self.set_motor2('backward')
            print("sara was right")

        elif direction == 'left':
            self.set_motor1('backward')
            self.set_motor2('forward')

        elif direction == 'right':
            self.set_motor1('forward')
            self.set_motor2('backward')

    # Function to stop the motors

    def stop_motors(self):
        self.set_motor1('stop')
        self.set_motor2('stop')

    # Define a function to be called whenever the encoder value changes

    def update_encoder_value(self, channel):
        if channel == ENCODER_PIN:
            if GPIO.input(ENCODER_PIN) == GPIO.HIGH:
                self.encoder_val += 1
        elif channel == ENCODER_PIN2:
            if GPIO.input(ENCODER_PIN2) == GPIO.HIGH:
                self.encoder_val2 += 1

    # Calculate current linear velocity in cm/s based on the encoder values

    def get_linear_vel(self):
        current_time = time.time()
        if current_time - self.old_time >= 0.5:
            pulses_per_sec = self.encoder_val / (current_time - self.old_time)
            rev_per_sec = pulses_per_sec / self.pulses_per_rev
            self.linear_vel = rev_per_sec * self.wheel_circumference
            pulses_per_sec = self.encoder_val2 / (current_time - self.old_time)
            rev_per_sec = pulses_per_sec / self.pulses_per_rev
            self.linear_vel2 = rev_per_sec * self.wheel_circumference
            self.old_time = time.time()
            self.encoder_val = 0
            self.encoder_val2 = 0
            self.update_speed = True

    # Get PID control value and update PWM duty cycle

    def adjust_speed(self):
        self.get_linear_vel()
        if self.update_speed:
            control = self.pid(self.linear_vel)
            control2 = self.pid(self.linear_vel2)
            self.update_speed = False

            print(self.pwm_speed, control, self.pwm_speed + control,
                  self.pwm_speed2, control2, self.pwm_speed2 + control)

            print("Vel 1: " + str(self.linear_vel) +
                  ", Vel 2: " + str(self.linear_vel2))

            if (self.pwm_speed + control <= 100 and self.pwm_speed + control >= 0):
                self.pwm_speed += control
                self.motor1_pwm.ChangeDutyCycle(self.pwm_speed)
            if (self.pwm_speed2 + control2 <= 100 and self.pwm_speed2 + control2 >= 0):
                self.pwm_speed2 += control2
                self.motor2_pwm.ChangeDutyCycle(self.pwm_speed2)
        return self.encoder_val, self.encoder_val2, self.linear_vel, self.linear_vel2
