import json
import socket
import sys
import cv2
import pickle
import numpy as np
import struct ## new
import zlib
import io
import socket
import struct
from PIL import Image
import numpy as np
import cv2
import rospy
import math
from std_msgs.msg import String
import multiprocessing
import multiprocessing.connection as connection
"""Nicholas Novak
ENOM 673
May 11, 2023"""
"""Performs processing and detection and streams instructions back to the robot"""

# source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

def no_aruco(frame):
    """Expects a grayscale image as frame."""
    arucos = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, arucos, parameters=parameters)
    return len(markerCorners) > 1
def get_aruco_dist(vec):
    vec = vec[0][0]
    return math.sqrt(vec[0]**2+vec[1]**2+vec[2]**2) / 2.54

def get_aruco_angle(frame, corners):
    # x_vals_top = corners[0][0][0][0]
    # x_vals_top.append(corners[0][0][3][0])
    center = (corners[0][0][0][0] + corners[0][0][3][0]) / 2
    width = frame.shape[0]
    if center < width / 2 - (width/20): # Tolerance for center
        return ["pivot_left", 20]
    elif center > width / 2 - (width/20):
        return ["pivot_right", 20]
    else: return["pivot_right", 10]

def process_image_data(frame):
    """Convert image with or without ArUco into movement instructions"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if (no_aruco(gray)):
        print("No image found. Spinning")
        instructions = {
            "pivot_right": 65
        }
        pub = rospy.Publisher('movement_data', String, queue_size=10)
        rospy.init_node('data', anonymous=True)
        pub.publish(str(instructions))
        return frame
    print("Image found. Processing image with corners:")


    # From calibration:
    cam_mat = np.array([[438.82164334,0,291.39742015],[0,434.38844849,231.3656673],[0,0,1]])
    dist_coeffs = np.array([0.18058037, -0.84386195, -0.02003987, -0.01876743, 0.99554009])
                       

    arucos = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    parameters = cv2.aruco.DetectorParameters_create()
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(frame, arucos, parameters=parameters, )
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, 10, cam_mat, dist_coeffs)

    print(len(markerCorners), markerCorners)
    print("Rotation: ", rvec)
    print("Translation: ", tvec)
    cv2.aruco.drawDetectedMarkers(frame,markerCorners)

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



    # connection = client_socket.makefile('wb')
    encoded_instructions = json.dumps(instructions).encode('utf-8')
    print("instruction size ", sys.getsizeof(encoded_instructions))
    encoded_instructions = pickle.dumps(encoded_instructions, 0)

    print("Giving instructions: ",instructions)
    return frame

HOST='192.168.129.140'
PORT=8485

s=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
print('Socket created')

s.bind((HOST,PORT))
print('Socket bind complete')
s.listen(10)
print('Socket now listening')

conn,addr=s.accept()


data = b""
payload_size = struct.calcsize(">L")
print("payload_size: {}".format(payload_size))
iter = 0

global client_socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.129.190', 8085))
# frame_seen = false
while True:
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
    frame_data = data[:msg_size]
    data = data[msg_size:]

    frame = pickle.loads(frame_data, fix_imports=True, encoding="bytes")
    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
    
    frame = process_image_data(frame)
    cv2.imshow('ImageWindow',frame)

    print("Hit iteration: ",iter)
    iter += 1
    cv2.waitKey(1)