import picamera
import time
import cv2
import numpy as np

# Load the trained fire detection model
model = cv2.ml.SVM_load('fire_detection_model.xml')

def detect_fire(image):
  # Resize the image to fit the model's input size
  image = cv2.resize(image, (64, 64))
  
  # Convert the image to grayscale
  gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
  # Flatten the image into a 1D array
  flatten = gray.flatten()
  
  # Reshape the array into a 2D array of 1x4096 (the model's input size)
  reshaped = flatten.reshape(1, -1)
  
  # Use the model to make a prediction on the image
  _, result = model.predict(reshaped)
  
  # Return the result (1 indicates fire, 0 indicates no fire)
  return result[0][0]

# Start the camera preview and wait for the camera to adjust to the lighting
with picamera.PiCamera() as camera:
  camera.start_preview()
  time.sleep(2)

  while True:
    # Capture an image from the camera
    image = camera.capture('image.jpg')
    
    # Use the detection function to check if the image contains fire
    if detect_fire(image):
      print("Fire detected!")
    else:
      print("No fire detected.")

    time.sleep(1)
  
  # Stop the camera preview
  camera.stop_preview()
