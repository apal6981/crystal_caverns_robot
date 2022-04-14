#Example Servo Code
#Control the angle of a 
#Servo Motor with Raspberry Pi

# free for use without warranty
# www.learnrobotics.org

import RPi.GPIO as GPIO
from time import sleep

# GPIO.setmode(GPIO.BOARD)
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

servo_l=GPIO.PWM(20, 50)
servo_r=GPIO.PWM(21, 50)
servo_l.start(0)
servo_r.start(0)

def setAngle(angle):
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

count = 0
numLoops = 2

while count < numLoops:
    print("set to 0-deg")
    setAngle(0)
    sleep(.5)

        
    print("set to 90-deg")
    setAngle(90)
    sleep(.5)

    print("set to 135-deg")
    setAngle(130)
    sleep(.5)
    
    count=count+1

servo_l.stop()
servo_r.stop()
GPIO.cleanup()