import serial
import threading, queue
import numpy as np
from gpiozero import Motor

odom_q = queue.Queue(1)
motor_l = Motor(3, 2)
motor_r = Motor(10, 9)

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

    while ser.is_open:
        # print("waiting for bytes", ser.in_waiting)
        # only grab from buffer where there is at least one message in it
        if ser.in_waiting > 40:
            # do wheel odom stuff
            in_wait = ser.in_waiting
            in_wait -= in_wait % 41
            # print(in_wait)
            # while in_wait > 40:
            #     print(in_wait)
            #     in_wait -= 41
            #     stuff = ser.read(in_wait)
            left, right = ser.read(in_wait)[-41:-2].decode().rstrip().split(" ")
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
            # print("odom:",x,y,theta,"time:",time.time())
            prev_r = right
            prev_l = left
            # send pose to queue to be processed
            odom_q.put([x, y, theta, d_center / dt, phi / dt])

# WAY_POINTS = [np.pi/2, -np.pi*3/4, -np.pi/4, np.pi/4,3*np.pi/4,-np.pi/2]
WAY_POINTS =[2,-3,-1,1,3,-2]



KP = .2

def main():
    threading.Thread(
        target=teensy_read,
    ).start()
    way_point_index = 0
    while way_point_index < 6:
        try:
            odom = odom_q.get_nowait()
            # goal = abs(WAY_POINTS[way_point_index])
            # start = abs(odom[2])

            # theta_err = min(2*np.pi-start+goal, goal+start)
            theta_err = (WAY_POINTS[way_point_index] % 4 + abs(odom[2] % -4) if WAY_POINTS[way_point_index] < 0 and odom[2] >= 0 else WAY_POINTS[way_point_index]-odom[2])*np.pi/4

            if theta_err < .3:
                way_point_index += 1
                print("waypoint",way_point_index,"was hit")
                continue
            motor_command = max(min(KP*theta_err, .4),.17)
            print("err:",theta_err,"speed:", motor_command,"orient:",odom[2],"waypoint:",WAY_POINTS[way_point_index])
            motor_l.value = -motor_command
            motor_r.value = motor_command

            # dist_err = np.sqrt((odom[0]-CENTER_GOAL[0])**2+(odom[1]-CENTER_GOAL[1])**2)
            # if dist_err < .05:
            #     motor_l.stop()
            #     motor_r.stop()
            #     center = True
            #     print("in the center")
            #     continue
            # # theta_err = 
            # motor_command = max(min(KP_d*dist_err-KD_d*odom[3], .7),.4)
            # theta_err = calc_theta_error([CENTER_GOAL[0]-odom[0],CENTER_GOAL[1]-odom[1]],[dist_err*np.cos(odom[3]),dist_err*np.sin(odom[3])])
            # theta_adjust = KP_t*theta_err- KD_t*odom[4]
            # print("odom:",odom[:3],"dist error:", dist_err, "new motor command:",motor_command, "theta err:", theta_err, "theta Adjust:",theta_adjust)
            # motor_l.forward(motor_command-theta_adjust)
            # motor_r.forward(motor_command+theta_adjust)
        except queue.Empty:
            pass

main()