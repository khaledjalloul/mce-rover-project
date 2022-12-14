import RPi.GPIO as GPIO
import cv2
import numpy as np
import csv
from motor_controller import MotorController
import datetime
import time

# Set the GPIO mode to BCM and define pins
GPIO.setmode(GPIO.BCM)

FLAME_SENSOR_PIN = 9
RELAY_PIN = 10

GPIO.setup(FLAME_SENSOR_PIN, GPIO.IN)
GPIO.setup(RELAY_PIN, GPIO.OUT)

# Define initial variables
pump_on = False
motors_on = False
flame_location = 'None'
flame_coordinates = 'None'
encoder1_val = 0
encoder2_val = 0
speed1_val = 0
speed2_val = 0
flame_sensor_val = 0

# Open the USB serial camera
camera = cv2.VideoCapture(0)

# Read the flame sensor's value
def read_flame_sensor():
    global flame_sensor_val
    value = GPIO.input(FLAME_SENSOR_PIN)
    if (value == 0):
        flame_sensor_val = 1
    else:
        flame_sensor_val = 0
    return value == 0


if __name__ == '__main__':
    controller = MotorController()

    with open('data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)

        # Write csv data column
        writer.writerow(['Time', 'Encoder 1 Value', 'Encoder 2 Value', 'Speed 1 (cm/s)',
                        'Speed 2 (cm/s)', 'Flame Location', 'Flame X Coordinate', 'Flame Sensor'])

        try:
            while True:

                if (not read_flame_sensor()):

                    # Turn the water pump off
                    GPIO.output(RELAY_PIN, GPIO.LOW)
                    pump_on = False

                    # Capture a frame from the camera
                    ret, frame = camera.read()
                    height, width, dim = frame.shape

                    # Convert the frame to the HSV color space
                    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                    # Define the lower and upper bounds for the flame color
                    lower_bound = np.array([10, 100, 100])
                    upper_bound = np.array([50, 255, 255])

                    # Threshold the HSV image to get only the flame colors
                    mask = cv2.inRange(hsv, lower_bound, upper_bound)

                    # Bitwise-AND the mask and the original image
                    res = cv2.bitwise_and(frame, frame, mask=mask)

                    # Find contours in the mask
                    contours, _ = cv2.findContours(
                        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

                    # Find the contour with the largest area
                    contour_areas = [cv2.contourArea(c) for c in contours]

                    if contour_areas:
                        largest_contour_index = np.argmax(contour_areas)
                        largest_contour = contours[largest_contour_index]

                        # Draw a bounding box around the largest flame
                        (x, y, w, h) = cv2.boundingRect(largest_contour)

                        # Limit detected contours to those larger than threshold
                        if w > 70 and h > 70:
                            flame_center_x = x + (w//2)
                            flame_center_y = y + (h//2)
                            cv2.rectangle(frame, (x, y), (x+w, y+h),
                                          (0, 0, 255), 2)
                            # Add a label to the frame
                            cv2.putText(frame, 'Flame', (x, y-10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                            # Get encoder and speed values
                            encoder1_val, encoder2_val, speed1_val, speed2_val = controller.adjust_speed()
                            flame_coordinates = flame_center_x
                            motors_on = True

                            # Move robot based on flame location
                            if (flame_center_x >= 2*width/5 and flame_center_x <= 3*width/5):
                                print("Center")
                                flame_location = 'Center'
                                controller.move('forward')
                            elif flame_center_x < 2*width/5:
                                print("Left")
                                flame_location = 'Left'
                                controller.move('left')
                            else:
                                print("Right")
                                flame_location = 'Right'
                                controller.move('right')

                        # Stop the robot if the flame is very close
                        if w >= 0.7 * width or h >= 0.7 * height:
                            controller.stop_motors()
                            time.sleep(6)
                    else:
                        flame_coordinates = 'None'
                        flame_location = 'None'
                        # Turn off motors
                        if motors_on:
                            encoder1_val = 0
                            encoder2_val = 0
                            speed1_val = 0
                            speed2_val = 0
                            controller.stop_motors()
                            motors_on = False

                    # Display the original and processed frames
                    cv2.imshow('Original', frame)
                    cv2.imshow('Flame', res)

                    # Display frame for 1ms and check if the 'q' key is pressed
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                else:
                    if motors_on:
                        encoder1_val = 0
                        encoder2_val = 0
                        speed1_val = 0
                        speed2_val = 0
                        controller.stop_motors()
                        motors_on = False

                    # Turn the water pump on after a short delay
                    time.sleep(2)
                    if not pump_on:
                        GPIO.output(RELAY_PIN, GPIO.HIGH)
                        print("Pump actuated.")
                        pump_on = True
                        time.sleep(5)

                # Write sensor data to csv file
                writer.writerow([datetime.datetime.now().strftime('%H:%M:%S'), encoder1_val, encoder2_val,
                                speed1_val, speed2_val, flame_location, flame_coordinates, flame_sensor_val])
        except Exception as e:
            print(e)
        finally:
            # Release the camera and destroy the windows
            camera.release()
            cv2.destroyAllWindows()

            # Stop motors and clear GPIO pins
            controller.stop_motors()
            GPIO.cleanup()
