from distutils import dist
import threading, queue, serial
import numpy as np
from gpiozero import Motor, Servo

import time
# thread safe queues of size one to only allow most recent message to be in it
odom_q = queue.Queue(1)
sens_q = queue.Queue(1)

# motor objects
motor_l = Motor(3, 2)
motor_r = Motor(10, 9)

# servo objects
servo_l = Servo(20)
servo_r = Servo(21)


# read from the arduino, Send the four sensor values along to the the main thread
def arduino_read():
    ard = serial.Serial("/dev/ttyUSB0", baudrate=115200)
    ard.flushInput()
    while True:
        # print("waiting for ard bytes",ard.in_waiting)
        if ard.in_waiting > 20:
            sens_values = ard.readline().decode().rstrip().split(" ")
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
    ser = serial.Serial("/dev/ttyACM1", baudrate=115200)
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


from enum import Enum, auto


class State(Enum):
    INIT = auto()
    IR_START = auto()
    DRIVE_TO_CENTER = auto()
    SPIN_CYCLE = auto()
    DRIVE_TO_BALL = auto()
    BACK_UP = auto()
    IR_FINISH = auto()
    DRIVE_HOME = auto()
    FINISH = auto()

def calc_theta_error(array1, array2):
    inner = np.inner(array1, array2)
    norms = np.linalg.norm(array1) * np.linalg.norm(array2)

    cos = inner / norms
    return np.arccos(np.clip(cos, -1.0, 1.0))




SENSOR_START_THRESH = 10
SENSOR_FINISH_THRESH = 100
CENTER_GOAL = [.70,0]


KP_d = .7
KD_d = 0
KP_t = .3
KD_t = .05


def main():
    # Start threads for teensy and arduino boards
    threading.Thread(
        target=teensy_read,
    ).start()
    threading.Thread(target=arduino_read).start()

    current_state = State.INIT
    center = False
    have_ball = False
    frame_centered = False
    home = False

    # run the state machine
    while True:
        # state transitions
        try:
            if current_state == State.INIT:
                current_state = State.IR_START

            elif current_state == State.IR_START:
                 # check if sensor is seeing the start command
                sensor_values = sens_q.get_nowait()
                print(sensor_values)
                if sensor_values[3] > SENSOR_START_THRESH:  # TODO add actual sensor index value.... Done but set threshold value
                    current_state = State.DRIVE_TO_CENTER

            elif current_state == State.DRIVE_TO_CENTER:
                # check to see if we made it to the center
                if center:  # TODO how are we checking if we made it to the center
                    print("Enter the spin cycle, time to get groovy")
                    current_state = State.SPIN_CYCLE

            elif current_state == State.SPIN_CYCLE:
                # check if ball is aligned enough
                if frame_centered:
                    print("driving to ball")
                    current_state = State.DRIVE_TO_BALL

            elif current_state == State.DRIVE_TO_BALL:
                # check to see if we have the ball
                if have_ball:
                    current_state = State.BACK_UP

            elif current_state == State.BACK_UP:
                if center:  # TODO how are we checking if we made it to the center
                    current_state = State.IR_FINISH

            elif current_state == State.IR_FINISH:
                # Start corner as been identified
                if sens_q.get_nowait()[0] > SENSOR_FINISH_THRESH:  # TODO add actual sensor index value
                    current_state = State.DRIVE_HOME

            elif current_state == State.DRIVE_HOME:
                if home:  # TODO how are we checking if we made it home
                    current_state == State.FINISH

            elif current_state == State.FINISH:
                pass

            else:
                print("you shouldn't be here")
                raise RuntimeError()
        except queue.Empty:
            pass

        try:
            # State Actions
            if current_state == State.INIT:
                # Bring arm up to middle
                # servo_l.max()
                # servo_r.max()
                pass
            elif current_state == State.IR_START:
                pass
            elif current_state == State.DRIVE_TO_CENTER:
                odom = odom_q.get_nowait()
                dist_err = np.sqrt((odom[0]-CENTER_GOAL[0])**2+(odom[1]-CENTER_GOAL[1])**2)
                if dist_err < .05:
                    motor_l.stop()
                    motor_r.stop()
                    center = True
                    print("in the center")
                    continue
                # theta_err = 
                motor_command = max(min(KP_d*dist_err-KD_d*odom[3], .7),.4)
                theta_err = calc_theta_error([CENTER_GOAL[0]-odom[0],CENTER_GOAL[1]-odom[1]],[dist_err*np.cos(odom[3]),dist_err*np.sin(odom[3])])
                theta_adjust = KP_t*theta_err- KD_t*odom[4]
                print("odom:",odom[:3],"dist error:", dist_err, "new motor command:",motor_command, "theta err:", theta_err, "theta Adjust:",theta_adjust)
                motor_l.forward(motor_command-theta_adjust)
                motor_r.forward(motor_command+theta_adjust)

                pass
            elif current_state == State.SPIN_CYCLE:
                motor_l.backward(.17)
                motor_r.forward(.17)
                
                pass
            elif current_state == State.DRIVE_TO_BALL:
                pass
            elif current_state == State.BACK_UP:
                pass
            elif current_state == State.IR_FINISH:
                pass
            elif current_state == State.DRIVE_HOME:
                pass
            elif current_state == State.FINISH:
                pass
            else:
                print("you shouldn't be here")
                raise RuntimeError()
        except queue.Empty:
            # print("nothing in odom q")
            pass


main()
