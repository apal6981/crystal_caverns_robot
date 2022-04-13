from gpiozero import Servo
import time

servo_l = Servo(20)
servo_r = Servo(21)

time.sleep(3)
servo_l.min()
servo_r.min()

# time.sleep(2)
# motor_l.stop()
# motor_r.stop()

