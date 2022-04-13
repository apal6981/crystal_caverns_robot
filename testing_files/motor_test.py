from gpiozero import Motor
import time

motor_l = Motor(3, 2)
motor_r = Motor(10, 9)

time.sleep(3)
motor_l.forward(.5)
motor_r.forward(.5)

time.sleep(2)
motor_l.stop()
motor_r.stop()

