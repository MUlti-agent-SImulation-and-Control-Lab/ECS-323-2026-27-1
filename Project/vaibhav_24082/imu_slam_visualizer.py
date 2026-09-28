import sys
import time
import math
import smbus2
import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *

# ==========================================
# MPU6050 (GY-521) REGISTERS & CONFIG
# ==========================================
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

ACCEL_SCALE = 16384.0  # ±2g range
GYRO_SCALE  = 131.0    # ±250 deg/s range
ALPHA = 0.98           # Complementary filter weight

class MPU6050:
    def __init__(self, bus_num):
        self.bus = smbus2.SMBus(bus_num)
        self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01) 
        self.bus.write_byte_data(MPU6050_ADDR, CONFIG, 0x03)       
        self.bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, 0x00)  
        self.bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, 0x00) 

    def _read_word_2c(self, reg):
        high = self.bus.read_byte_data(MPU6050_ADDR, reg)
        low = self.bus.read_byte_data(MPU6050_ADDR, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def get_data(self):
        ax = self._read_word_2c(ACCEL_XOUT_H) / ACCEL_SCALE
        ay = self._read_word_2c(ACCEL_XOUT_H + 2) / ACCEL_SCALE
        az = self._read_word_2c(ACCEL_XOUT_H + 4) / ACCEL_SCALE

        gx = self._read_word_2c(GYRO_XOUT_H) / GYRO_SCALE
        gy = self._read_word_2c(GYRO_XOUT_H + 2) / GYRO_SCALE
        gz = self._read_word_2c(GYRO_XOUT_H + 4) / GYRO_SCALE

        return ax, ay, az, gx, gy, gz

# ==========================================
# 3D AEROPLANE VISUALIZATION
# ==========================================
def draw_aeroplane():
    """Draws a stylized 3D aeroplane shape"""
    glBegin(GL_QUADS)
    
    # Fuselage / Main Body (Silver/Gray)
    glColor3f(0.75, 0.75, 0.75)
    glVertex3f( 0.4,  0.15, -1.2)
    glVertex3f(-0.4,  0.15, -1.2)
    glVertex3f(-0.4,  0.15,  1.2)
    glVertex3f( 0.4,  0.15,  1.2)

    glVertex3f( 0.4, -0.15,  1.2)
    glVertex3f(-0.4, -0.15,  1.2)
    glVertex3f(-0.4, -0.15, -1.2)
    glVertex3f( 0.4, -0.15, -1.2)

    # Main Wings (Bright Red)
    glColor3f(0.85, 0.1, 0.1)
    glVertex3f( 2.2,  0.0,  0.4)
    glVertex3f(-2.2,  0.0,  0.4)
    glVertex3f(-2.2,  0.0, -0.2)
    glVertex3f( 2.2,  0.0, -0.2)
    
    # Tail Wings (Dark Red)
    glColor3f(0.6, 0.0, 0.0)
    glVertex3f( 0.8,  0.0, -0.8)
    glVertex3f(-0.8,  0.0, -0.8)
    glVertex3f(-0.8,  0.0, -1.1)
    glVertex3f( 0.8,  0.0, -1.1)
    glEnd()

    # Tail Fin (Blue Vertical Stabilizer)
    glBegin(GL_TRIANGLES)
    glColor3f(0.1, 0.3, 0.8)
    glVertex3f( 0.0,  0.15, -0.8)
    glVertex3f( 0.0,  0.70, -1.1)
    glVertex3f( 0.0,  0.15, -1.1)
    glEnd()

# ==========================================
# MAIN CONTROL LOOP
# ==========================================
def main():
    mpu = None
    for bus_idx in [7, 8, 1]:
        try:
            mpu = MPU6050(bus_num=bus_idx)
            print(f"Successfully connected to MPU6050 (GY-521) on I2C Bus {bus_idx}!")
            break
        except Exception:
            continue

    if not mpu:
        print("CRITICAL: MPU6050 not detected.")
        sys.exit(1)

    # Full 3-Axis Gyro Calibration
    print("Calibrating 3-axis gyro drift... KEEP IMU COMPLETELY STILL.")
    gyro_x_bias, gyro_y_bias, gyro_z_bias = 0, 0, 0
    calibration_samples = 300
    for _ in range(calibration_samples):
        _, _, _, gx, gy, gz = mpu.get_data()
        gyro_x_bias += gx
        gyro_y_bias += gy
        gyro_z_bias += gz
        time.sleep(0.005)
    gyro_x_bias /= calibration_samples
    gyro_y_bias /= calibration_samples
    gyro_z_bias /= calibration_samples
    print(f"Calibration Complete! Bias offsets -> X: {gyro_x_bias:.2f}, Y: {gyro_y_bias:.2f}, Z: {gyro_z_bias:.2f}")

    # Initialize Pygame & OpenGL Context
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    pygame.display.set_caption("3-Axis SLAM IMU Filter Visualizer")

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -5)

    # Filter State Variables
    roll, pitch, yaw = 0.0, 0.0, 0.0
    last_time = time.time()
    clock = pygame.time.Clock()

    print("\n--- LOGGING SENSOR DATA (Close visualization window to stop) ---")
    print("Format: [ACCEL X,Y,Z] | [GYRO X,Y,Z] || [FILTERED ROLL, PITCH, YAW]")

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            try:
                ax, ay, az, gx, gy, gz = mpu.get_data()
            except IOError:
                continue  

            # Remove calibrated gyro drift biases
            gx -= gyro_x_bias
            gy -= gyro_y_bias
            gz -= gyro_z_bias

            # Calculate raw Tilt angles from Accelerometer (Gravity reference)
            accel_roll = math.atan2(ay, az) * 57.2958
            accel_pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 57.2958

            # FUSE DATA VIA COMPLEMENTARY FILTER
            roll = ALPHA * (roll + gx * dt) + (1 - ALPHA) * accel_roll
            pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * accel_pitch
            
            # Yaw integration (Relies entirely on gyro tracking)
            yaw += gz * dt
            # Keep yaw bounds strictly within 0-360 degrees for stability
            yaw = (yaw + 180) % 360 - 180

            # Complete sensor message log stream to terminal
            log_msg = (
                f"ACC: {ax:+5.2f},{ay:+5.2f},{az:+5.2f} g | "
                f"GYR: {gx:+6.1f},{gy:+6.1f},{gz:+6.1f} °/s || "
                f"FILTERED -> R: {roll:6.1f}° P: {pitch:6.1f}° Y: {yaw:6.1f}°"
            )
            print(log_msg)

            # Update OpenGL Scene Dynamics
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            glEnable(GL_DEPTH_TEST)
            
            glPushMatrix()
            # Apply all 3 rotational configurations to the matrix
            glRotatef(yaw, 0, 0, 1)     # Z-axis (Yaw) - Turning left/right
            glRotatef(roll, 1, 0, 0)    # X-axis (Roll) - Bank left/right
            glRotatef(-pitch, 0, 1, 0)  # Y-axis (Pitch) - Nose up/down
            
            draw_aeroplane()
            glPopMatrix()

            pygame.display.flip()
            clock.tick(60)

    except KeyboardInterrupt:
        pass
    finally:
        pygame.quit()
        print("\nVisualizer session safely closed.")

if __name__ == '__main__':
    main()
