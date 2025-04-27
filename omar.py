import smbus					#import SMBus module of I2C
from time import sleep, time          #import
import numpy as np
import math

#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value

def get_rotation(gyro_x, gyro_y, gyro_z, dt):
    # Convert gyro values to degrees per second
    rate_roll = gyro_x / 131.0
    rate_pitch = gyro_y / 131.0
    rate_yaw = gyro_z / 131.0
    
    # Calculate angles based on gyro data
    roll += rate_roll * dt
    pitch += rate_pitch * dt
    yaw += rate_yaw * dt
    
    return roll, pitch, yaw

def apply_complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt):
    global roll, pitch, yaw
    
    # Calculate roll and pitch from accelerometer (excluding yaw as it can't be determined from gravity)
    acc_roll = math.atan2(acc_y, acc_z) * 180.0 / math.pi
    acc_pitch = math.atan2(-acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z)) * 180.0 / math.pi
    
    # Get gyro-based angles
    gyro_roll, gyro_pitch, gyro_yaw = get_rotation(gyro_x, gyro_y, gyro_z, dt)
    
    # Complementary filter - combine accelerometer and gyroscope data
    # 0.98 and 0.02 are filter coefficients that can be tuned
    roll = 0.98 * gyro_roll + 0.02 * acc_roll
    pitch = 0.98 * gyro_pitch + 0.02 * acc_pitch
    # Yaw can only be calculated from gyro data or magnetometer (not included)
    
    return roll, pitch, yaw

def remove_gravity(acc_x, acc_y, acc_z, roll, pitch):
    # Convert angles to radians
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    
    # Rotate acceleration vectors to remove gravity component
    acc_x_without_gravity = acc_x - 9.81 * math.sin(pitch_rad)
    acc_y_without_gravity = acc_y + 9.81 * math.sin(roll_rad) * math.cos(pitch_rad)
    acc_z_without_gravity = acc_z + 9.81 * math.cos(roll_rad) * math.cos(pitch_rad)
    
    return acc_x_without_gravity, acc_y_without_gravity, acc_z_without_gravity

bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

# Initialize variables
position_x, position_y, position_z = 0, 0, 0
velocity_x, velocity_y, velocity_z = 0, 0, 0
roll, pitch, yaw = 0, 0, 0
prev_time = time()

# Calibration
print("Calibrating sensor. Please keep the sensor still...")
sum_ax, sum_ay, sum_az = 0, 0, 0
sum_gx, sum_gy, sum_gz = 0, 0, 0
samples = 100

for _ in range(samples):
    acc_x = read_raw_data(ACCEL_XOUT_H) / 16384.0
    acc_y = read_raw_data(ACCEL_YOUT_H) / 16384.0
    acc_z = read_raw_data(ACCEL_ZOUT_H) / 16384.0
    
    gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0
    gyro_y = read_raw_data(GYRO_YOUT_H) / 131.0
    gyro_z = read_raw_data(GYRO_ZOUT_H) / 131.0
    
    sum_ax += acc_x
    sum_ay += acc_y
    sum_az += acc_z
    sum_gx += gyro_x
    sum_gy += gyro_y
    sum_gz += gyro_z
    sleep(0.01)

# Calculate offsets
offset_ax = sum_ax / samples
offset_ay = sum_ay / samples
offset_az = sum_az / samples - 1.0  # Account for gravity
offset_gx = sum_gx / samples
offset_gy = sum_gy / samples
offset_gz = sum_gz / samples

print("Calibration complete. Starting position tracking...")

try:
    while True:
        # Calculate time difference
        current_time = time()
        dt = current_time - prev_time
        prev_time = current_time
        
        # Read sensor data
        acc_x = read_raw_data(ACCEL_XOUT_H) / 16384.0 - offset_ax
        acc_y = read_raw_data(ACCEL_YOUT_H) / 16384.0 - offset_ay
        acc_z = read_raw_data(ACCEL_ZOUT_H) / 16384.0 - offset_az
        
        gyro_x = read_raw_data(GYRO_XOUT_H) / 131.0 - offset_gx
        gyro_y = read_raw_data(GYRO_YOUT_H) / 131.0 - offset_gy
        gyro_z = read_raw_data(GYRO_ZOUT_H) / 131.0 - offset_gz
        
        # Update orientation estimation with complementary filter
        roll, pitch, yaw = apply_complementary_filter(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, dt)
        
        # Remove gravity from acceleration measurements
        lin_acc_x, lin_acc_y, lin_acc_z = remove_gravity(acc_x, acc_y, acc_z, roll, pitch)
        
        # Convert g to m/s²
        lin_acc_x *= 9.81
        lin_acc_y *= 9.81
        lin_acc_z *= 9.81
        
        # Apply threshold to reduce noise
        threshold = 0.1
        if abs(lin_acc_x) < threshold: lin_acc_x = 0
        if abs(lin_acc_y) < threshold: lin_acc_y = 0
        if abs(lin_acc_z) < threshold: lin_acc_z = 0
        
        # Integrate acceleration to get velocity
        velocity_x += lin_acc_x * dt
        velocity_y += lin_acc_y * dt
        velocity_z += lin_acc_z * dt
        
        # Apply high-pass filter to velocity (reduces drift)
        filter_coeff = 0.95
        velocity_x *= filter_coeff
        velocity_y *= filter_coeff
        velocity_z *= filter_coeff
        
        # Integrate velocity to get position
        position_x += velocity_x * dt
        position_y += velocity_y * dt
        position_z += velocity_z * dt
        
        # Print position data
        print(f"Position: X={position_x:.2f}m, Y={position_y:.2f}m, Z={position_z:.2f}m")
        print(f"Orientation: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")
        
        sleep(0.05)  # 50ms sampling rate
        
except KeyboardInterrupt:
    print("Program terminated by user")