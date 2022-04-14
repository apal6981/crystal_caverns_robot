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

pwm=GPIO.PWM(20, 50)
pwm.start(0)

def setAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(20, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(20, False)
    pwm.ChangeDutyCycle(duty)

count = 0
numLoops = 2

while count < numLoops:
    print("set to 0-deg")
    setAngle(0)
    sleep(1)

        
    print("set to 90-deg")
    setAngle(90)
    sleep(1)

    print("set to 135-deg")
    setAngle(135)
    sleep(1)
    
    count=count+1

pwm.stop()
GPIO.cleanup()