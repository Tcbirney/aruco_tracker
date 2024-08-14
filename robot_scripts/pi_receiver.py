import json
import socket
import sys
import pickle
import numpy as np
import struct ## new
import zlib
import ast
import io
import socket
import struct
from PIL import Image
import numpy as np
import cv2
import math
import RPi.GPIO as gpio
import rospy
import cv2
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera
import time

"""Nicholas Novak
ENPM 673
May 12, 2023"""
"""Recieve instructions and interpret into driving instructions. Execute driving."""
def init():
    # Prepare pins for operation
    gpio.setmode(gpio.BOARD)
    gpio.setup(31,gpio.OUT) #IN1
    gpio.setup(33,gpio.OUT) #IN2
    gpio.setup(35,gpio.OUT) #IN3
    gpio.setup(37,gpio.OUT) #IN4
    gpio.setup(36, gpio.OUT)
    gpio.output(36, False)

    gpio.setup(12,gpio.IN,pull_up_down = gpio.PUD_UP)
    gpio.setup(7,gpio.IN,pull_up_down = gpio.PUD_UP)


def gameover():
    # Cleanup for last stage to safely release pxm pins
    # Set all pins low
    gpio.output(31,False)
    gpio.output(33,False)
    gpio.output(35,False)
    gpio.output(37,False)
    
    
    gpio.cleanup()


def forward(dist):
    init()
    print("Forward ",dist)

    pwm = gpio.PWM(37,50)
    pwm2 = gpio.PWM(31,50)
    counter = np.uint64(0)
    counter2 = np.uint64(0) 
    button = int(0)
    button2 = int(0)

    KP = 0.9
    dist2 = float(dist)
    dist1 = dist2*20/8

    d_cycle = 50
    d2_cycle = 50
    pwm.start(0)
    pwm2.start(0)
    forward = True
    while forward:
        
        if counter >= dist1 or counter2 >= dist1:
            print("Complete")
            forward = False

    
        pwm.start(d_cycle)
        pwm2.start(d_cycle)
            
        if int (gpio.input(12)) != int(button):
            button = int(gpio.input(12))
            counter+= 1
        if int (gpio.input(7)) != int(button2):
            button2 = int(gpio.input(7))
            counter2+=1

        if counter > counter2:
            if d_cycle < 90 and d2_cycle > 2:
                d_cycle/=KP
                d2_cycle*=KP
                pwm.ChangeDutyCycle(d_cycle)
                pwm2.ChangeDutyCycle(d2_cycle)
            else:
                pass
            
        if counter2 > counter:
            if d_cycle > 2 and d2_cycle < 90:
                d_cycle*=KP
                d2_cycle/=KP
                pwm.ChangeDutyCycle(d_cycle)
                pwm2.ChangeDutyCycle(d2_cycle)
            else:
                pass
        
        r_dist = (counter+counter2)/2
        # print(d_cycle, d2_cycle)

    pwm.stop()
    pwm2.stop()

def reverse(dist):
    init()
    print("Reversing")

    pwm = gpio.PWM(35,50)
    pwm2 = gpio.PWM(33,50)
    counter = np.uint64(0)
    counter2 = np.uint64(0) 
    button = int(0)
    button2 = int(0)

    KP = 0.9
    distance = float(dist)
    distance = distance*20/8
    d_cycle = 50
    d2_cycle = 50
    pwm.start(0)
    pwm2.start(0)
    reverse = True

    while reverse:
        
        if counter >= distance or counter2 >= distance:
            print("Complete")
            reverse = False

    
        pwm.start(d_cycle)
        pwm2.start(d_cycle)
            
        if int (gpio.input(12)) != int(button):
            button = int(gpio.input(12))
            counter+= 1
        if int (gpio.input(7)) != int(button2):
            button2 = int(gpio.input(7))
            counter2+=1

        if counter > counter2:
            if d_cycle < 90 and d2_cycle > 2:
                d_cycle/=KP
                d2_cycle*=KP
                pwm.ChangeDutyCycle(d_cycle)
                pwm2.ChangeDutyCycle(d2_cycle)
            else:
                pass
            
        if counter2 > counter:
            if d_cycle > 2 and d2_cycle < 90:
                d_cycle*=KP
                d2_cycle/=KP
                pwm.ChangeDutyCycle(d_cycle)
                pwm2.ChangeDutyCycle(d2_cycle)
            else:
                pass        
    pwm.stop()
    pwm2.stop()

def pivot_right(deg):
    print("Right ",deg, " from ")

    print("Right Turn ", deg)
    pwm = gpio.PWM(35,50)
    pwm2 = gpio.PWM(31,50)
    counter = np.uint64(0)
    counter2 = np.uint64(0) 
    button = int(0)
    button2 = int(0)
    
    count = 0
    out_deg = 0
    d_cycle = 50
    while True:
        if(ser.in_waiting > 0):
            line = ser.readline()
            count += 1
            if count > 10:
                line = line.rstrip().lstrip()
                line = str(line)
                line = line.strip("b'X: \r\n")
                line = round(float(line))
                print("Starting at: ",line)
                line_1 = line
                break
    
    count = 0
    while (deg+line_1)%360>out_deg:
        if(ser.in_waiting > 0):
            count +=1
            line = ser.readline()            
            if count > 10:
                
                time.sleep(0.01)
                pwm.start(d_cycle)
                pwm2.start(d_cycle)
                line = line.rstrip().lstrip()
                line = str(line)
                
                line = line.strip("b'X: \r\n")
                
                line = float(line)

                if line >350:
                    line -= 350
                out_deg = round(line)
            if int (gpio.input(12)) != int(button):
                button = int(gpio.input(12))
                counter+= 1
            if int (gpio.input(7)) != int(button2):
                button2 = int(gpio.input(7))
                counter2+=1
            print(line)
    pwm.stop()
    pwm2.stop()
    # return(out_deg)

def pivot_left(deg):
    print("Left ",deg)


    c_angle = 0
    print("Left Turn ", deg)
    pwm = gpio.PWM(37,50)
    pwm2 = gpio.PWM(33,50)
    counter = np.uint64(0)
    counter2 = np.uint64(0) 
    button = int(0)
    button2 = int(0)
    
    count = 0
    out_deg = 0
    d_cycle = 50
    
    while deg+c_angle>out_deg:
        if(ser.in_waiting > 0):
            count +=1
        
            line = ser.readline()
            
            if count > 10:
                
                time.sleep(0.01)
                pwm.start(d_cycle)
                pwm2.start(d_cycle)
                
                line = line.rstrip().lstrip()
                line = str(line)
                
                line = line.strip("b'X: \r\n")
                
                line = float(line)
                
                if c_angle == 0:
                    c_angle = round(line)
                
                out_deg = 360-round(line)
            time.sleep(0.05)
            if int (gpio.input(12)) != int(button):
                button = int(gpio.input(12))
                counter+= 1
            if int (gpio.input(7)) != int(button2):
                button2 = int(gpio.input(7))
                counter2+=1
            print(line)
    pwm.stop()
    pwm2.stop()
    # return(out_deg)



def move_process(incoming_data):
    

    for instruction in incoming_data:
        if instruction == "forward":
            forward(incoming_data[instruction])
        elif instruction == "reverse":
            reverse(incoming_data[instruction])
        elif instruction == "pivot_right":
            pivot_right(incoming_data[instruction])
        elif instruction == "pivot_left":
            pivot_left(incoming_data[instruction])
    gameover()



global ser 
ser = serial.Serial('/dev/ttyUSB0',9600)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('192.168.129.190',8485))
print("Listening...")
s.listen(10)
conn,addr=s.accept()
data = b''
payload_size = struct.calcsize(">L")
print("payload_size: {}".format(payload_size))
while True:

    init()

    while len(data) < payload_size:
        print("Recv: {}".format(len(data)))
        data += conn.recv(4096)

    print("Done Recv: {}".format(len(data)))
    packed_msg_size = data[:payload_size]
    data = data[payload_size:]
    msg_size = struct.unpack(">L", packed_msg_size)[0]
    print("msg_size: {}".format(msg_size))
    while len(data) < msg_size:
        data += conn.recv(4096)
    move_data = data[:msg_size]
    move_data = pickle.loads(data)
    # print(move_data)
    # print(type(move_data))
    d = ast.literal_eval(move_data)
    print("moving: ",d)
    print(type(d))
    move_process(d)
