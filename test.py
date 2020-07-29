#!/usr/bin/env python
import cv2
import numpy as np

distCoeffs = np.zeros((5), dtype=np.float)
print(distCoeffs)

def do_process():
    src = cv2.imread('yolo3.png')
    print('shape: ', src.shape)
    height,width,channel = src.shape
    center = np.array([int(width/2.0), int(height/2.0)])
    print(center)
    # find red markers in horizontal band and vertical band
    # horizontal band
    # the start point is left-up corner of cropped image
    hori_left_start = int(center[0]-height/5.0)
    verti_up_start = int(center[0]-width/5.0)
    hori_left_band = src[int(center[0]-height/5.0):int(center[0]+height/5.0), 10:int(width/2.0)]
    hori_right_band = src[int(center[0]-height/5.0):int(center[0]+height/5.0), int(width/2.0):width-10]
    verti_up_band = src[10:int(height/2.0) , int(center[0]-width/5.0):int(center[0]+width/5.0)]
    verti_down_band = src[int(height/2.0):height-10 , int(center[0]-width/5.0):int(center[0]+width/5.0)]
    # cv2.imshow('hori_left_band',hori_left_band)
    # cv2.imshow('hori_right_band',hori_right_band)
    # cv2.imshow('verti_up_band',verti_up_band)
    # cv2.imshow('verti_down_band',verti_down_band)

    hori_left_hsv = cv2.cvtColor(hori_left_band,cv2.COLOR_BGR2HSV)
    hori_right_hsv = cv2.cvtColor(hori_right_band,cv2.COLOR_BGR2HSV)
    verti_up_hsv = cv2.cvtColor(verti_up_band,cv2.COLOR_BGR2HSV)
    verti_down_hsv = cv2.cvtColor(verti_down_band,cv2.COLOR_BGR2HSV)
    hori_lower_hsv = np.array([0,50,50])
    hori_upper_hsv = np.array([30,255,255])
    verti_lower_hsv = np.array([0,50,50])
    verti_upper_hsv = np.array([100,255,255])

    hori_left_mask = cv2.inRange(hori_left_hsv, hori_lower_hsv, hori_upper_hsv)
    hori_right_mask = cv2.inRange(hori_right_hsv, hori_lower_hsv, hori_upper_hsv)
    verti_up_mask = cv2.inRange(verti_up_hsv, verti_lower_hsv, verti_upper_hsv)
    verti_down_mask = cv2.inRange(verti_down_hsv, verti_lower_hsv, verti_upper_hsv)
    # cv2.imshow('hori_left_mask', hori_left_mask)
    # cv2.imshow('hori_right_mask', hori_right_mask)
    # cv2.imshow('verti_up_mask', verti_up_mask)
    # cv2.imshow('verti_down_mask', verti_down_mask)

    #(B,G,R)=cv2.split(src)
    #sub_BR = cv2.subtract(B,R)
    #cv2.imshow('R',R)
    #cv2.imshow('G',G)
    #cv2.imshow('B',B) # use blue
    #cv2.imshow('sub_BR',sub_BR)
    # B = cv2.threshold(B,150,255,cv2.THRESH_BINARY)[1]
    # B = cv2.threshold(B,150,255,cv2.THRESH_BINARY)[1]
    # B = cv2.threshold(B,120,255,cv2.THRESH_BINARY)[1] # good external contours
    # B = cv2.threshold(B,100,255,cv2.THRESH_BINARY)[1] # good 1 internal circles
    #B = cv2.threshold(B,80,255,cv2.THRESH_BINARY)[1] # good 2 internal circles
    #B = cv2.threshold(B,70,255,cv2.THRESH_BINARY)[1] # not see internal circles
    #cv2.imshow('B_threshold',B)
    #contours, hierarchy = cv2.findContours(B,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)
    #contours, hierarchy = cv2.findContours(B,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_NONE)
    # erode and dilate
    erode_kernel = np.ones((2, 2), np.uint8)
    hori_left_erosion = cv2.erode(hori_left_mask, erode_kernel, iterations=1)
    hori_right_erosion = cv2.erode(hori_right_mask, erode_kernel, iterations=1)
    verti_up_erosion = cv2.erode(verti_up_mask, erode_kernel, iterations=1)
    verti_down_erosion = cv2.erode(verti_down_mask, erode_kernel, iterations=1)
    #cv2.imshow('hori_erosion', hori_erosion)
    #cv2.imshow('verti_erosion', verti_erosion)

    dilate_kernel = np.ones((5, 5), np.uint8)
    hori_left_dilate = cv2.dilate(hori_left_erosion, dilate_kernel, iterations=1)
    hori_right_dilate = cv2.dilate(hori_right_erosion, dilate_kernel, iterations=1)
    verti_up_dilate = cv2.dilate(verti_up_erosion, dilate_kernel, iterations=1)
    verti_down_dilate = cv2.dilate(verti_down_erosion, dilate_kernel, iterations=1)
    # cv2.imshow('hori_left_dilate', hori_left_dilate)
    # cv2.imshow('hori_right_dilate', hori_right_dilate)
    # cv2.imshow('verti_up_dilate', verti_up_dilate)
    # cv2.imshow('verti_down_dilate', verti_down_dilate)

    # find contours
    hori_left_contours,hereachy= cv2.findContours(hori_left_dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    hori_right_contours,hereachy= cv2.findContours(hori_right_dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    print(len(hori_left_contours))
    print(len(hori_right_contours))

    cv2.drawContours(hori_left_band,hori_left_contours,-1,(0,255,255), thickness=1 )
    cv2.imshow('hori_left_contours',hori_left_band)
    cv2.drawContours(hori_right_band,hori_right_contours,-1,(0,255,255), thickness=1 )
    cv2.imshow('hori_right_contours',hori_right_band)

    verti_up_contours,hereachy = cv2.findContours(verti_up_dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    verti_down_contours,hereachy = cv2.findContours(verti_down_dilate,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    print(len(verti_up_contours))
    print(len(verti_down_contours))
    cv2.drawContours(verti_up_band,verti_up_contours,-1,(0,255,255), thickness=1 )
    cv2.imshow('verti_up_contours',verti_up_band)
    cv2.drawContours(verti_down_band,verti_down_contours,-1,(0,255,255), thickness=1 )
    cv2.imshow('verti_down_contours',verti_down_band)

    #for i in hori_contours:


    cv2.waitKey(0)



if __name__ =="__main__":
    do_process()
