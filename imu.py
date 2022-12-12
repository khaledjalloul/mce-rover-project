# Import the math library
import math

# Initialize MPU6050
sensor = mpu6050(0x68)

# Loop indefinitely
while True:
  # Read accelerometer and gyroscope data from the MPU6050
  accel_data = sensor.get_accel_data()
  gyro_data = sensor.get_gyro_data()

  # Extract the x, y, and z values from the accelerometer and gyroscope data
  accel_x = accel_data['x']
  accel_y = accel_data['y']
  accel_z = accel_data['z']
  gyro_x = gyro_data['x']
  gyro_y = gyro_data['y']
  gyro_z = gyro_data['z']

  # Calculate the pitch, roll, and yaw using the accelerometer and gyroscope data
  pitch = math.atan2(accel_y, accel_z)
  roll = math.atan2(-accel_x, math.sqrt(accel_y * accel_y + accel_z * accel_z))
  yaw = math.atan2(gyro_z

  # Print the orientation of the rover
print("Pitch: ", pitch)
print("Roll: ", roll)
print("Yaw: ", yaw)