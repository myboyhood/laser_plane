#!/usr/bin/python
# coding: utf-8

import socket
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import os
import sys
from math import exp as exp
from time import time
import numpy as np
import cv2
import socket
import pyrealsense2 as rs



target_corner_msg = QuaternionStamped()
pickup_corner_msg = QuaternionStamped()






cameraMatrix1 = np.array([[4.021600058871760e+02,0,3.256257492316297e+02],
                          [0,4.042465054244421e+02,2.363862572079333e+02],
                          [0,0,1]])
distCoeffs1 = np.array([0.021783506391129,0.041588347987938,0,0])
cameraMatrix2 = np.array([[5.912765186796250e+02,0,3.149171177529422e+02],
                          [0,5.929122177632328e+02,2.454413937259454e+02],
                          [0,0,1]])
distCoeffs2 = np.array([0.033837545644885,0.104562708182371,0,0])
imageSize = np.array([0,0])
R = np.array([[0.999756671556603,-0.019685697115233,0.009953441974998],
              [0.019893236464260,0.999577305708524,-0.021200685259327],
              [-0.009531884443245,0.021393532704426,0.999725692346348]])
T = np.array([42.839622912636670,-2.272286334377919,-2.534953507215020])
R1, R2, P1, P2, Q, validPixROI1, validPixROI2 = cv2.stereoRectify(cameraMatrix1,distCoeffs1,
                                                                  cameraMatrix2,distCoeffs2,
                                                                  (640,480),R,T,alpha=-1, flags=cv2.CALIB_ZERO_DISPARITY )
map1_x, map1_y = cv2.initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, (640,480), cv2.CV_32FC1)
map2_x, map2_y = cv2.initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, (640,480), cv2.CV_32FC1)


def ir_img_init():
    print ("jump into ir_img_init")


    # Start streaming
    pipeline.start(config)
    get_img_flag = False
    count = 0
    print("count: ", count)
    while not (get_img_flag and count > 10): # 多取几幅图片，前几张不清晰
        # Wait for a coherent pair of frames（一对连贯的帧）: depth and color
        frame = pipeline.wait_for_frames()
        print('wait for frames in the first loop')
        ir_frame = frame.get_infrared_frame(1)

        if not ir_frame: # 如果ir没有得到图像，就continue继续
            continue

        ir_frame_src = np.asanyarray(ir_frame.get_data())
        get_img_flag = True # 跳出循环
        count += 1



def clint():

    rospy.init_node('yolo_clint', anonymous=True)

    yolo_target_pub = rospy.Publisher('yolo_target_corner', QuaternionStamped, queue_size=1)
    yolo_pickup_pub = rospy.Publisher('yolo_pickup_corner', QuaternionStamped, queue_size=1)
    ir_rect_img_pub = rospy.Publisher('ir_rect_img',Image, queue_size=1)
    rate = rospy.Rate(30)


    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 连接服务端
    print ('connect state: ', s.connect_ex(('127.0.0.1', 8000)))

    # 初始化realsense camera
    # ir_img_init()
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.infrared,1,640,480,rs.format.y8,30)
    pipeline.start(config)

    while not rospy.is_shutdown():
        # 输入IR图像

        frame = pipeline.wait_for_frames()
        print('wait for frames in the loop')
        ir_frame = frame.get_infrared_frame(1)

        if not ir_frame: # 如果ir没有得到图像，就continue继续
            print ("no ir img ! continue")
            continue

        ir_frame_src = np.asanyarray(ir_frame.get_data())

        print ("ir_frame_src.size", ir_frame_src.size)
        cv2.imshow("ir", ir_frame_src)
        ir_rect = cv2.remap(ir_frame_src, map1_x, map1_y, cv2.INTER_LINEAR)
        ir_rect_img_pub.publish(CvBridge().cv2_to_imgmsg(ir_rect, "mono8"))
        print ("publish done!")

        # 发布yolo置信区域
        target_get_flag = False
        pickup_get_flag = False
        receive_msg = s.recv(100).decode()
        # print (receive_msg)
        msg = receive_msg.split(',')
        print(msg)
        if msg[0] == '0' and msg[1] > '0':
            target_get_flag = True
            print ("target corner")
            """ QuaternionStamped.x, y, z, w = xmin, ymin, xmax, ymax """
            target_corner_msg.header.stamp = rospy.get_rostime()
            target_corner_msg.quaternion.x = float(msg[1])
            target_corner_msg.quaternion.y = float(msg[2])
            target_corner_msg.quaternion.z = float(msg[3])
            target_corner_msg.quaternion.w = float(msg[4])

        if msg[0] == '1' and msg[1] > '0':
            pickup_get_flag = True
            print ("pickup corner")
            """ QuaternionStamped.x, y, z, w = xmin, ymin, xmax, ymax """
            pickup_corner_msg.header.stamp = rospy.get_rostime()
            pickup_corner_msg.quaternion.x = float(msg[1])
            pickup_corner_msg.quaternion.y = float(msg[2])
            pickup_corner_msg.quaternion.z = float(msg[3])
            pickup_corner_msg.quaternion.w = float(msg[4])

        if not target_get_flag:
            print (" target not found ... ")
            target_corner_msg.header.stamp = rospy.get_rostime()
            target_corner_msg.quaternion.x = -1.0
            target_corner_msg.quaternion.y = 0.0
            target_corner_msg.quaternion.z = 0.0
            target_corner_msg.quaternion.w = 0.0

        if not pickup_get_flag:
            print (" pickup not found ... ")
            pickup_corner_msg.header.stamp = rospy.get_rostime()
            pickup_corner_msg.quaternion.x = -1.0
            pickup_corner_msg.quaternion.y = 0.0
            pickup_corner_msg.quaternion.z = 0.0
            pickup_corner_msg.quaternion.w = 0.0

        yolo_target_pub.publish(target_corner_msg)
        yolo_pickup_pub.publish(pickup_corner_msg)
        rate.sleep()


if __name__ == "__main__":
    clint()