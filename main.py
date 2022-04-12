from dis import dis
import threading, queue, serial
from regex import R
import numpy as np
from gpiozero import Motor

# thread safe queues of size one to only allow most recent message to be in it
odom_q = queue.Queue(1)
sens_q = queue.Queue(1)

# motor objects
motor_l = Motor(2, 3)
motor_r = Motor(9, 10)

# read from the arduino, Send the four sensor values along to the the main thread
def arduino_read():
    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)
    ser.flushInput()
    while True:
        if ser.in_waiting > 20:
            sens_values = ser.readline().decode().rstrip().split(" ")
            sens_q.put(
                [
                    int(sens_values[0]),
                    int(sens_values[1]),
                    int(sens_values[2]),
                    int(sens_values[3]),
                ]
            )


# read from the teensy the encoder values, calculate and update pose and pass to main thread
def teensy_read():
    # make theta between pi and -pi
    def minimize_angle(angle):
        while angle < -np.pi:
            angle = angle + 2 * np.pi
        while angle >= np.pi:
            angle = angle - 2 * np.pi
        return angle

    # open the serial object to talk to teensy board
    ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
    ser.flushInput()
    while ser.in_waiting < 40:
        pass
    # get the init values for wheel odometry
    prev_l, prev_r = ser.readline().decode().rstrip().split(" ")
    prev_l = int(prev_l)
    prev_r = int(prev_r)
    print(prev_l, prev_r)

    x = 0
    y = 0
    theta = 0
    dt = 0.050

    # wheel odometry constants
    gear_reduction = 784.0 / 81.0
    wheel_circum = 0.0905 * np.pi
    wheel_base = 0.200  # TODO get actual value
    dist_const = wheel_circum / (512 * 4 * gear_reduction)

    while ser.is_open:
        # only grab from buffer where there is at least one message in it
        if ser.in_waiting > 40:
            # do wheel odom stuff
            left, right = ser.readline().decode().rstrip().split(" ")
            left = int(left)
            right = int(right)
            delta_l = (left - prev_l) * dist_const
            delta_r = (right - prev_r) * dist_const
            d_center = (delta_r + delta_l) / 2
            phi = minimize_angle((delta_r - delta_l) / wheel_base)
            x += d_center * np.cos(phi)
            y += d_center * np.sin(phi)
            theta += phi
            theta = minimize_angle(theta)
            print(x, y, theta)

            prev_r = right
            prev_l = left
            # send pose to queue to be processed
            odom_q.put([x, y, theta])


def main():
    # Start threads for teensy and arduino boards
    threading.Thread(
        target=teensy_read,
    ).start()
    threading.Thread(target=arduino_read).start()

    # run the state machine
    while True:
        try:
            print(odom_q.get_nowait())
        except queue.Empty:
            pass
        try:
            print(sens_q.get_nowait())
        except queue.Empty:
            pass


main()
