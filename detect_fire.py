import picamera
import time

def detect_fire(threshold=10):
  with picamera.PiCamera() as camera:
    camera.start_preview()
    time.sleep(2) # Give the camera time to adjust to the lighting

    while True:
      image = camera.capture('image.jpg')
      fire_pixels = 0
      for pixel in image:
        if pixel > threshold:
          fire_pixels += 1
      if fire_pixels > 100:
        return True
      time.sleep(1)
    camera.stop_preview()

if detect_fire():
  print("Fire detected!")
else:
  print("No fire detected.")
