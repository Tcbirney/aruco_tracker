import cv2
import io
import socket
import struct
import time
import pickle
import math
import zlib
import RPi.GPIO as gpio
import rospy
import numpy as np
import cv2
import serial
from picamera.array import PiRGBArray
from picamera import PiCamera

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
                # print("Starting at: ",line)
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
            # print(line)
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
            # print(line)
    pwm.stop()
    pwm2.stop()
    # return(out_deg)



def move_process(incoming_data):
    
    init()

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
    # message = "Request next move"
    # size = len(message)
    # message = pickle.dumps(message, 0)

    # client_socket.sendall(struct.pack("L",), size + message)



def no_aruco(frame):
    """Expects a grayscale image as frame."""
    arucos = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, arucos, parameters=parameters)
    print("Aruco corners: ", markerCorners, len(markerCorners) < 1)
    return len(markerCorners) < 1
def get_aruco_dist(vec):
    vec = vec[0][0]
    return math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2) / 2.54

def get_aruco_angle(frame, corners):
    # x_vals_top = corners[0][0][0][0]
    # x_vals_top.append(corners[0][0][3][0])
    center = (corners[0][0][0][0] + corners[0][0][3][0]) / 2
    width = frame.shape[0]
    if center < width / 2 - (width/20): # Tolerance for center
        return ["pivot_left", 5]
    elif center > width / 2 - (width/20):
        return ["pivot_right", 5]
    else: return["forward", 1]

def process_image_data(frame):
    """Convert image with or without ArUco into movement instructions"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if (no_aruco(gray)):
        print("No image found. Spinning")
        instructions = {
            "pivot_right": 10
        }
        # pub = rospy.Publisher('movement_data', String, queue_size=10)
        # rospy.init_node('data', anonymous=True)
        # pub.publish(str(instructions))
        move_process(instructions)
    print("Image found. Processing image with corners:")


    # From calibration:
    cam_mat = np.array([[438.82164334,0,291.39742015],[0,434.38844849,231.3656673],[0,0,1]])
    dist_coeffs = np.array([0.18058037, -0.84386195, -0.02003987, -0.01876743, 0.99554009])
                       

    arucos = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, arucos, parameters=parameters, )
    
    print(len(markerCorners), markerCorners)
    objPoints = np.array([[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]])
    
    instructions = {}
    if len(markerCorners) == 1:
        print("Marker corners seen and processing at length ", len(markerCorners))
        print(markerCorners[0],markerCorners[0][0],markerCorners[0][0][0])

        pivot_data = get_aruco_angle(frame, markerCorners)
        
        instructions = {
            pivot_data[0]: pivot_data[1],
            # "forward": get_aruco_dist(tvec) - 6
        }
    else:
        instructions = {"pivot_right": 1}



    
    print("Giving instructions: ",instructions)
    move_process(instructions)


global ser 
ser = serial.Serial('/dev/ttyUSB0',9600)
# cam = cv2.VideoCapture(0)

camera = PiCamera()
rawCap = PiRGBArray(camera)
# cam.set(3, 320);
# cam.set(4, 240);
time.sleep(0.1)
img_counter = 0

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
while True:
    camera.capture(rawCap, format='bgr')

    frame1 = rawCap.array
    process_image_data(frame1)

    result, frame = cv2.imencode('.jpg', frame1, encode_param)
#    data = zlib.compress(pickle.dumps(frame, 0))
    data = pickle.dumps(frame, 0)
    size = len(data)


    print("{}: {}".format(img_counter, size))
    # client_socket.sendall(struct.pack(">L", size) + data)
    img_counter += 1
    rawCap.truncate(0)
