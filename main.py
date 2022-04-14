# from distutils import dist
import threading, queue, serial
import numpy as np
from gpiozero import Motor
import RPi.GPIO as GPIO

import time

import camera_hough as camera

# thread safe queues of size one to only allow most recent message to be in it
# odom_q = queue.Queue(1)
sens_q = queue.Queue(1)
drive_q = queue.Queue(1)

# motor objects

# servo objects
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
servo_l = GPIO.PWM(20, 50)
servo_r = GPIO.PWM(21, 50)
servo_l.start(0)
servo_r.start(0)

real_cam = camera.Camera()

L_STOP = 1508
R_STOP = 1498

# read from the arduino, Send the four sensor values along to the the main thread
def arduino_read():
    ard = serial.Serial("/dev/ttyUSB0", baudrate=9600)
    ard.flushInput()
    while True:
        # print("waiting for ard bytes",ard.in_waiting)
        if ard.in_waiting > 20:
            in_wait = ard.in_waiting
            in_wait -= in_wait % 21
            # print(in_wait)
            # while in_wait > 40:
            #     print(in_wait)
            #     in_wait -= 41
            #     stuff = ser.read(in_wait)
            sens_values = ser.read(in_wait)[-21:-2].decode().rstrip().split(" ")
            # sens_values = ard.readline().decode().rstrip().split(" ")
            sens_q.put(
                [
                    int(sens_values[0]),
                    int(sens_values[1]),
                    int(sens_values[2]),
                    int(sens_values[3]),
                ]
            )
        try:
            drive_command = drive_q.get_nowait()
            ard.write(
                (str(drive_command[0]).zfill(4)+ " "+ str(drive_command[1]).zfill(4)+ " ").encode("utf-8"))
            
        except queue.Empty:
            pass


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


# Depth is in meters
def calc_way_point(odom, depth):
    return [depth * np.cos(odom[2]) + odom[0], depth * np.sin(odom[2]) + odom[1]]


def set_arm(angle):
    duty_l = angle / 18 + 2
    duty_r = (135 - angle) / 18 + 2
    GPIO.output(20, True)
    GPIO.output(21, True)
    servo_l.ChangeDutyCycle(duty_l)
    servo_r.ChangeDutyCycle(duty_r)
    # sleep(1)
    GPIO.output(20, False)
    GPIO.output(21, False)
    servo_l.ChangeDutyCycle(duty_l)
    servo_r.ChangeDutyCycle(duty_r)

def calc_orientation(val):
    l_baseline = 1508
    r_baseline = 1498
    l_scale = -100
    r_scale = -100

    return int((l_baseline + (val * l_scale))), int((r_baseline + (1 - val) * r_scale))


SENSOR_START_THRESH = -1
SENSOR_FINISH_THRESH = 100


def main():
    # Start threads for teensy and arduino boards
    # threading.Thread(
    #     target=teensy_read,
    # ).start()
    threading.Thread(target=arduino_read).start()

    current_state = State.INIT
    center = False
    have_ball = False
    frame_centered = False
    home = False

    way_point_index = 0
    ball_depth = 0
    d_goal = 1.0

    # run the state machine
    while True:
        # state transitions
        try:
            if current_state == State.INIT:
                current_state = State.IR_START
                set_arm(30)

            elif current_state == State.IR_START:
                # check if sensor is seeing the start command
                # sensor_values = sens_q.get_nowait()
                # print(sensor_values)
                # if (
                #     sensor_values[3] > SENSOR_START_THRESH
                # ):  # TODO add actual sensor index value.... Done but set threshold value
                #     current_state = State.DRIVE_TO_CENTER
                #     set_arm(85)
                current_state = State.DRIVE_TO_CENTER
                print("we are going driving")

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
                    print("beep.....beep.....beep")
                    set_arm(130)
                    current_state = State.BACK_UP

            elif current_state == State.BACK_UP:
                if center:  # TODO how are we checking if we made it to the center
                    print("go towards the light")
                    current_state = State.IR_FINISH
                    drive_q.put([1518,1485]])

            elif current_state == State.IR_FINISH:
                # Start corner as been identified
                if (
                    sens_q.get_nowait()[0] > SENSOR_FINISH_THRESH
                ):  # TODO add actual sensor index value
                    print("Found the holy light, now stay on the straight and narrow")
                    current_state = State.DRIVE_HOME
                    drive_q.put([1350,1350])

            elif current_state == State.DRIVE_HOME:
                if home:  # TODO how are we checking if we made it home
                    print("you made it home")
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
                pass
            elif current_state == State.IR_START:
                pass
            elif current_state == State.DRIVE_TO_CENTER:
                # drive_q.put[180, 180]
                ball_found, val, dist = real_cam.find_ball()
                print("val:", val, "dist:",dist)
                if dist <= d_goal:
                    center = True
                    continue

                # l_val, r_val = calc_orientation(val)
                drive_q.put([1300, 1300])
                
                pass

            elif current_state == State.SPIN_CYCLE:
                ball_found, val, dist = real_cam.find_ball()
                if ball_found:
                    print("ball found, val:",val)
                    print("stopping")
                    # if .35 < val < .65:
                    #     print("ball centered")
                    drive_q.put([1508,1498])
                    frame_centered = True
                    continue
                    

                drive_q.put([1518,1485])
                pass

            elif current_state == State.DRIVE_TO_BALL:
                depth = real_cam.drive_to_ball()
                if depth < .5:
                    print("got the ball")
                    have_ball = True
                    
                drive_q.put([1480,1470])
                pass

            elif current_state == State.BACK_UP:

                drive_q.put([1600,1600])
                time.sleep(1)
                drive_q.put([1508,1498])
                print("back uped")
                pass

            elif current_state == State.IR_FINISH:
                # we are just spinning here
                pass

            elif current_state == State.DRIVE_HOME:
                time.sleep(2)
                # we should be home
                drive_q.put([1508,1498])
                home = True
                pass

            elif current_state == State.FINISH:
                # Free GPIO from servos
                drive_q.put([1508,1498])
                servo_l.stop()
                servo_r.stop()
                GPIO.cleanup()

                pass
            else:
                print("you shouldn't be here")
                raise RuntimeError()
        except queue.Empty:
            # print("nothing in odom q")
            pass


main()
