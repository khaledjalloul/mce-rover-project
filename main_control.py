import RPi.GPIO as GPIO
import cv2
import numpy as np

from motor_controller import MotorController

# Set the GPIO mode to BCM
GPIO.setmode(GPIO.BCM)

FLAME_SENSOR_PIN = 9
RELAY_PIN = 10

GPIO.setup(FLAME_SENSOR_PIN, GPIO.IN)
GPIO.setup(RELAY_PIN, GPIO.OUT)

pump_on = False

# Open the USB serial camera
camera = cv2.VideoCapture(0)


def read_flame_sensor():
    # Read the flame sensor's value
    value = GPIO.input(FLAME_SENSOR_PIN)
    return value == 0


if __name__ == '__main__':
    controller = MotorController()

    try:
        while True:

            if (not read_flame_sensor()):
                # Turn the water pump off
                if pump_on:
                    GPIO.output(RELAY_PIN, GPIO.LOW)
                    pump_on = False

                # Capture a frame from the camera
                ret, frame = camera.read()
                height, width, dim = frame.shape
                center_x = width // 2
                center_y = height // 2

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

                    if w > 100 and h > 100:
                        flame_center_x = x + (w//2)
                        flame_center_y = y + (h//2)
                        cv2.rectangle(frame, (x, y), (x+w, y+h),
                                      (0, 0, 255), 2)
                        # Add a label to the frame
                        cv2.putText(frame, 'Flame', (x, y-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                        controller.adjust_speed()
                        if (flame_center_x >= 2*width/5 and flame_center_x <= 3*width/5):
                            controller.move('forward')
                        elif flame_center_x < 2*width/5:
                            controller.move('right')
                        else:
                            controller.move('left')

                # Display the original and processed frames
                cv2.imshow('Original', frame)
                cv2.imshow('Flame', res)

                # Check if the 'q' key is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                controller.stop_motors()
                # Turn the water pump on
                GPIO.output(RELAY_PIN, GPIO.HIGH)
                pump_on = True

    except Exception as e:
        print(e)
    finally:
        # Release the camera and destroy the windows
        camera.release()
        cv2.destroyAllWindows()

        # Stop motors and clear GPIO pins
        controller.stop_motors()
        GPIO.cleanup()
