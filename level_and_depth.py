# Import modules
import termux_api
import serial
import time
import math

# Define constants
TARGET_DEPTH = 10 # The target depth in meters
MAX_SPEED = 100 # The maximum speed of the motors in percent
PID_GAINS = [0.1, 0.01, 0.001] # The PID controller gains
SERIAL_PORT = "/dev/rfcomm0" # The serial port for Bluetooth communication
SERIAL_BAUD = 9600 # The serial baud rate

# Initialize variables
current_depth = 0 # The current depth in meters
current_pitch = 0 # The current pitch angle in degrees
current_roll = 0 # The current roll angle in degrees
error = 0 # The error between the target and current depth
integral = 0 # The integral of the error
derivative = 0 # The derivative of the error
output = 0 # The output of the PID controller
motor_left = 0 # The speed of the left motor in percent
motor_right = 0 # The speed of the right motor in percent
prev_time = 0 # The previous time in seconds
prev_error = 0 # The previous error

# Create serial object
ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD)

# Define functions
def get_depth():
    # Get the pressure from the barometer sensor
    pressure = termux_api.Sensor.get_pressure()
    # Convert the pressure to depth using the formula
    # depth = (pressure - atmospheric pressure) / (density * gravity)
    # Assume atmospheric pressure = 101325 Pa, density = 1025 kg/m3, gravity = 9.81 m/s2
    depth = (pressure - 101325) / (1025 * 9.81)
    return depth

def get_pitch():
    # Get the accelerometer values from the sensor
    acc_x, acc_y, acc_z = termux_api.Sensor.get_accelerometer()
    # Convert the accelerometer values to pitch angle using the formula
    # pitch = atan2(acc_y, sqrt(acc_x^2 + acc_z^2))
    pitch = math.degrees(math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)))
    return pitch

def get_roll():
    # Get the accelerometer values from the sensor
    acc_x, acc_y, acc_z = termux_api.Sensor.get_accelerometer()
    # Convert the accelerometer values to roll angle using the formula
    # roll = atan2(-acc_x, acc_z)
    roll = math.degrees(math.atan2(-acc_x, acc_z))
    return roll

def pid_controller(target, current):
    # Get the current time in seconds
    global prev_time
    global prev_error
    global integral
    current_time = time.time()
    # Calculate the error
    error = target - current
    # Calculate the time difference
    dt = current_time - prev_time
    # Calculate the integral
    integral = integral + error * dt
    # Calculate the derivative
    derivative = (error - prev_error) / dt
    # Calculate the output
    output = PID_GAINS[0] * error + PID_GAINS[1] * integral + PID_GAINS[2] * derivative
    # Update the previous time and error
    prev_time = current_time
    prev_error = error
    return output

def motor_control(output, pitch, roll):
    # Adjust the output based on the pitch and roll angles
    # Assume that positive pitch means the nose is up and positive roll means the right side is up
    # If the pitch is positive, decrease the speed of the front motor and increase the speed of the back motor
    # If the pitch is negative, increase the speed of the front motor and decrease the speed of the back motor
    # If the roll is positive, decrease the speed of the right motor and increase the speed of the left motor
    # If the roll is negative, increase the speed of the right motor and decrease the speed of the left motor
    # The adjustment factor is proportional to the absolute value of the angle
    # The adjustment factor is capped at 50 percent
    pitch_factor = min(abs(pitch), 50) / 100
    roll_factor = min(abs(roll), 50) / 100
    if pitch > 0:
        output_front = output - pitch_factor * output
        output_back = output + pitch_factor * output
    else:
        output_front = output + pitch_factor * output
        output_back = output - pitch_factor * output
    if roll > 0:
        output_right = output - roll_factor * output
        output_left = output + roll_factor * output
    else:
        output_right = output + roll_factor * output
        output_left = output - roll_factor * output
    # Map the output to the motor speed
    # Assume that the output range is -100 to 100 and the motor speed range is 0 to 255
    # If the output is positive, the motor moves forward
    # If the output is negative, the motor moves backward
    # The motor speed is proportional to the absolute value of the output
    motor_left = int((output_left + 100) / 200 * 255)
    motor_right = int((output_right + 100) / 200 * 255)
    return motor_left, motor_right

def send_data(motor_left, motor_right):
    # Send the motor speed data to the Arduino via Bluetooth serial
    # The data format is "LxxxRyyy\n" where xxx and yyy are the motor speed values
    # The values are padded with zeros if they are less than 100
    data = "L{:03d}R{:03d}\n".format(motor_left, motor_right)
    ser.write(data.encode())

# Main loop
while True:
    # Get the current depth, pitch and roll
    current_depth = get_depth()
    current_pitch = get_pitch()
    current_roll = get_roll()
    # Print the current values
    print("Current depth: {:.2f} m".format(current_depth))
    print("Current pitch: {:.2f} deg".format(current_pitch))
    print("Current roll: {:.2f} deg".format(current_roll))
    # Run the PID controller
    output = pid_controller(TARGET_DEPTH, current_depth)
    # Print the output
    print("Output: {:.2f}".format(output))
    # Run the motor control
    motor_left, motor_right = motor_control(output, current_pitch, current_roll)
    # Print the motor speed
    print("Motor left: {}".format(motor_left))
    print("Motor right: {}".format(motor_right))
    # Send the data to the Arduino
    send_data(motor_left, motor_right)
    # Wait for 0.1 seconds
    time.sleep(0.1)
