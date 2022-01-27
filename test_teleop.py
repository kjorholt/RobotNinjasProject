#!/usr/bin/env python
import RPi.GPIO as GPIO
import rospy
import sys
from geometry_msgs.msg import Twist
import time

v = 0.0
wc = 0.0
def getback(indata):
    v = indata.linear.x
    wc = indata.angular.z
    print (wc)
    print (v)

    if (v>0):
        forward()
    elif(v<0):
        backward()
    else:
        if(wc>0):
            left()
        elif(wc<0):
            right()
        else:
            stop()

def forward():
	print ("Moving Forward")
	GPIO.output(Motor1PWM,GPIO.HIGH)
	GPIO.output(Motor1D,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)
	pwm.ChangeDutyCycle(80)
	
	GPIO.output(Motor2PWM,GPIO.HIGH)
	GPIO.output(Motor2D,GPIO.HIGH)
	GPIO.output(Motor2E,GPIO.HIGH)
	pwm2.ChangeDutyCycle(80)
	#time.sleep(2)

def backward():
	print ("Moving Backwards")
	#GPIO.output(Motor1PWM,GPIO.HIGH)
	GPIO.output(Motor1D,GPIO.HIGH)
	GPIO.output(Motor1E,GPIO.HIGH)
	pwm.ChangeDutyCycle(80)
	
	#GPIO.output(Motor2PWM,GPIO.HIGH)
	GPIO.output(Motor2D,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.HIGH)
	pwm2.ChangeDutyCycle(80)
	#time.sleep(2)

def right():
	print ("Moving Right")
	#GPIO.output(Motor1PWM,GPIO.HIGH)
	GPIO.output(Motor1D,GPIO.LOW)
	GPIO.output(Motor1E,GPIO.HIGH)
        pwm.ChangeDutyCycle(40)

	#GPIO.output(Motor2PWM,GPIO.HIGH)
	GPIO.output(Motor2D,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.HIGH)
        pwm2.ChangeDutyCycle(40)
	#time.sleep(2)

def left():
	print ("Moving Left")
	#GPIO.output(Motor1PWM,GPIO.HIGH)
	GPIO.output(Motor1D,GPIO.HIGH)
	GPIO.output(Motor1E,GPIO.HIGH)
	pwm.ChangeDutyCycle(20)
	#GPIO.output(Motor2PWM,GPIO.HIGH)
	GPIO.output(Motor2D,GPIO.HIGH)
	GPIO.output(Motor2E,GPIO.HIGH)
	pwm2.ChangeDutyCycle(20)

def stop():
	print ("Now stop")
	GPIO.output(Motor1E,GPIO.LOW)
	GPIO.output(Motor2E,GPIO.LOW)

def listener():
    rospy.init_node('drive_control', anonymous=True)
    rospy.Subscriber('cmd_vel',Twist, getback)
    rospy.spin()


if __name__=='__main__':
    try:
        Motor1PWM = 12
        Motor1D = 24
        Motor1E= 22	
        Motor2PWM = 13
        Motor2D = 25
        Motor2E= 23
        GPIO.setmode(GPIO.BCM)	
        GPIO.setup(Motor1PWM, GPIO.OUT)
        GPIO.setup(Motor1D, GPIO.OUT)
        GPIO.setup(Motor1E, GPIO.OUT)	
        GPIO.setup(Motor2PWM, GPIO.OUT)
        GPIO.setup(Motor2D, GPIO.OUT)
        GPIO.setup(Motor2E, GPIO.OUT)

        pwm = GPIO.PWM(Motor1PWM, 1000)
        pwm2 = GPIO.PWM(Motor2PWM, 1000)
        pwm.start(0)
        pwm2.start(0)
        listener()


    finally:
	GPIO.cleanup()
