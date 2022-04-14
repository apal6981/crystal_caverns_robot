import serial
import numpy as np

# try:
#     ser = serial.Serial("/dev/ttyACM0", baudrate=115200)
# except Exception:
#     print("can't open serial")
#     exit(1)
# print("serial initialized")

# while ser.is_open:
#     if ser.in_waiting > 40:
#         print(ser.readline().decode())

# print("serial closing")
# ser.close()

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
    # print(prev_l, prev_r)

    x = 0
    y = 0
    theta = 0
    dt = 0.050

    # wheel odometry constants
    gear_reduction = 784.0 / 81.0
    wheel_circum = 0.0905 * np.pi
    wheel_base = 0.206  # TODO get actual value
    dist_const = wheel_circum / (512 * 4 * gear_reduction)

    print("start main loop")
    while ser.is_open:
        # only grab from buffer where there is at least one message in it
        if ser.in_waiting > 40:
            # do wheel odom stuff
            while ser.in_waiting > 40:
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
            # print(x, y, theta)

            prev_r = right
            prev_l = left
            print(x, y, theta)

teensy_read()
