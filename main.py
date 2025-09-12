#!usr/bin/env python3
import numpy as np
from stream import Stream 
import pyrealsense2 as rs
import cv2

max_value = 255
max_value_H = 360//2
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_detection_name = 'HSV Display'
window_2 = 'Threshold Display'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

morph_size = 0
max_operator = 4
max_elem = 2
max_kernel_size = 21
title_trackbar_operator_type = 'Operator:\n 0: Opening - 1: Closing  \n 2: Gradient - 3: Top Hat \n 4: Black Hat'
title_trackbar_element_type = 'Element:\n 0: Rect - 1: Cross - 2: Ellipse'
title_trackbar_kernel_size = 'Kernel size:\n 2n + 1'
morph_op_dic = {0: cv2.MORPH_OPEN, 1: cv2.MORPH_CLOSE, 2: cv2.MORPH_GRADIENT, 3: cv2.MORPH_TOPHAT, 4: cv2.MORPH_BLACKHAT}


video = "saved_video"

def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)
def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)
def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)
def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)

def morphology_operations(val, img):
    morph_operator = cv2.getTrackbarPos(title_trackbar_operator_type, window_2)
    morph_size = cv2.getTrackbarPos(title_trackbar_kernel_size, window_2)
    morph_elem = 0
    val_type = cv2.getTrackbarPos(title_trackbar_element_type, window_2)
    if val_type == 0:
        morph_elem = cv2.MORPH_RECT
    elif val_type == 1:
        morph_elem = cv2.MORPH_CROSS
    elif val_type == 2:
        morph_elem = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(morph_elem, (2*morph_size + 1, 2*morph_size+1), (morph_size, morph_size))
    operation = morph_op_dic[morph_operator]
    dst = cv2.morphologyEx(img, operation, element)
    cv2.imshow(window_2, dst)



def main():
    with Stream() as f:
        cv2.namedWindow(window_detection_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(window_2, cv2.WINDOW_NORMAL)
        cv2.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
        cv2.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
        cv2.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
        cv2.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
        cv2.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
        cv2.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
        # cv2.createTrackbar(title_trackbar_operator_type, window_2 , 0, max_operator, morphology_operations)
        # cv2.createTrackbar(title_trackbar_element_type, window_2, 0, max_elem, morphology_operations)
        # cv2.createTrackbar(title_trackbar_kernel_size, window_2 , 0, max_kernel_size, morphology_operations)
        while True:
            f.set_scale()
            f.align_self()
            frame = f.capture_frame()
            frame_HSV = f.convert_color(frame)
            frame_threshold = cv2.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))

            # images = np.hstack((frame, frame_threshold))
            cv2.imshow(window_detection_name, frame_threshold)
            key = cv2.waitKey(1)
            cv2.namedWindow('Demo Display', cv2.WINDOW_NORMAL)
            cv2.imshow('Demo Display', frame)

            frame_purple, mask = f.threshold_purple(frame_HSV)
            
            

            (cx, cy, cz), pnt = f.find_pen(mask)
            if cx != -1:
                print(cx, cy, cz)
                cv2.circle(frame_purple, center=(pnt[0], pnt[1]), radius = 10, color=(0,0,255), thickness =-1)

            # Press esc or 'q' to close the image window
            cv2.imshow('Threshold Display', frame_purple)
            # morphology_operations(0, mask)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

if __name__ == "__main__":
    main()