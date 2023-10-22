#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import socket
import os
import pigpio



############## Board setup
def general_init():
    #Set the GPIO port to BCM encoding mode
    GPIO.setmode(GPIO.BCM)
    #Ignore warning information
    GPIO.setwarnings(False)

def general_cleanup():
    GPIO.cleanup()

############## motor controls
#Definition of  motor pins
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

def motors_init():
    global pwm_ENA
    global pwm_ENB
    global pwm_servo_horizontal
    global pwm_servo_vertical
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)

    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)
    pwm_ENA.start(0)
    pwm_ENB.start(0)

#advance
def forward(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#back
def backward(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn left
def left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#trun right
def right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn left in place
def spin_left(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#turn right in place
def spin_right(leftspeed, rightspeed):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_ENA.ChangeDutyCycle(leftspeed)
    pwm_ENB.ChangeDutyCycle(rightspeed)

#brake
def brake():
   GPIO.output(IN1, GPIO.LOW)
   GPIO.output(IN2, GPIO.LOW)
   GPIO.output(IN3, GPIO.LOW)
   GPIO.output(IN4, GPIO.LOW)

#motor clean up
def motors_cleanup():
    pwm_ENA.stop()
    pwm_ENB.stop()



###############LEDs
#Definition of RGB module pins
LED_R = 22
LED_G = 27
LED_B = 24

#initialize LEDs
def LED_init():
    GPIO.setup(LED_R, GPIO.OUT)
    GPIO.setup(LED_G, GPIO.OUT)
    GPIO.setup(LED_B, GPIO.OUT)

def LED_magenta():
        # #Magenta
    GPIO.output(LED_R, GPIO.HIGH)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)

def LED_blue():
    #Blue
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.HIGH)

def LED_off():
    GPIO.output(LED_R, GPIO.LOW)
    GPIO.output(LED_G, GPIO.LOW)
    GPIO.output(LED_B, GPIO.LOW)




##################### Servos
#Definition of servo pin    J2(BCM 11) is horizontal, J3(BCM 9) is vertical
Servo_Horizontal_Pin = 11
Servo_Vertical_Pin = 9

def servos_init(initial_yaw, initial_pitch):

    # GPIO.setup(Servo_Horizontal_Pin, GPIO.OUT)
    # GPIO.setup(Servo_Vertical_Pin, GPIO.OUT)
    # global pwm_servo_horizontal
    # global pwm_servo_vertical
    # pwm_servo_horizontal = GPIO.PWM(Servo_Horizontal_Pin, 50)
    # pwm_servo_vertical = GPIO.PWM(Servo_Vertical_Pin, 50)
    # pwm_servo_horizontal.start(0)
    # pwm_servo_vertical.start(0)
    # vertical_servo_control(initial_pitch)
    # horizontal_servo_control(initial_yaw)

    #to call $sudo pigiod properly
    sudoPassword = 'Jtzy1012'
    command = 'pigpiod'
    os.system('echo %s|sudo %s' % (sudoPassword, command))

    time.sleep(2)

    global pwm_servo_horizontal
    global pwm_servo_vertical
    pwm_servo_horizontal = pigpio.pi()
    pwm_servo_vertical = pigpio.pi()
    pwm_servo_horizontal.set_mode(Servo_Horizontal_Pin, pigpio.OUTPUT)
    pwm_servo_vertical.set_mode(Servo_Vertical_Pin, pigpio.OUTPUT)
    pwm_servo_horizontal.set_PWM_frequency(Servo_Horizontal_Pin, 50)
    pwm_servo_vertical.set_PWM_frequency(Servo_Vertical_Pin, 50)
    horizontal_servo_control(initial_yaw)
    vertical_servo_control(initial_pitch)




#The servo rotates to the specified angle
def horizontal_servo_control(pos):
    # for i in range(18):
    for i in range(180):
        # pwm_servo_horizontal.ChangeDutyCycle(2.5 + 10 * pos/180)
        pwm_servo_horizontal.set_servo_pulsewidth( Servo_Horizontal_Pin, 500 + 1000 * pos/90)
    print("horizontal servo")

#The servo rotates to the specified angle
def vertical_servo_control(pos):
    # for i in range(18):
    for i in range(180):
        pwm_servo_vertical.set_servo_pulsewidth( Servo_Vertical_Pin, 500 + 1000 * pos/90)
    print("vertical servo")

def camera_up(camera_pitch):
    vertical_servo_control(camera_pitch)

def camera_down(camera_pitch):
    vertical_servo_control(camera_pitch)

def camera_left(camera_yaw):
    horizontal_servo_control(camera_yaw)

def camera_right(camera_yaw):
    horizontal_servo_control(camera_yaw)

def servos_cleanup():
    #kill pigpio
    sudoPassword = 'Jtzy1012'
    command = 'killall pigpiod'
    os.system('echo %s|sudo %s' % (sudoPassword, command))

def camera_init():
    #start motion
    sudoPassword = 'Jtzy1012'
    command = 'service motion start'
    os.system('echo %s|sudo %s' % (sudoPassword, command))