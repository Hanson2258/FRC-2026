# Code used to manually determine HSV values to mask specific colours and ranges
# code from: https://github.com/freedomwebtech/colortracking/blob/main/track.py

import time
import numpy as np
import cv2
import pyrealsense2 as rs

# cap=cv2.VideoCapture(0)

# Initialize and configure the pipeline
pipeline = rs.pipeline()
config = rs.config()

# Set up pipeline and get device info
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Enable depth and color streams at 640x480, 30fps
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline 
profile = pipeline.start(config)

def nothing(x):
    pass
cv2.namedWindow("Trackbars")
cv2.createTrackbar("L - H", "Trackbars", 0, 180, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 180, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)


while True:
     # Wait for a new set of frames
     frames = pipeline.wait_for_frames()
     colour_frame=frames.get_color_frame()
     frame = np.asanyarray(colour_frame.get_data())
     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
     l_h = cv2.getTrackbarPos("L - H", "Trackbars")
     l_s = cv2.getTrackbarPos("L - S", "Trackbars")
     l_v = cv2.getTrackbarPos("L - V", "Trackbars")
     u_h = cv2.getTrackbarPos("U - H", "Trackbars")
     u_s = cv2.getTrackbarPos("U - S", "Trackbars")
     u_v = cv2.getTrackbarPos("U - V", "Trackbars")
     lower_bound = np.array([l_h, l_s, l_v])
     upper_bound = np.array([u_h, u_s, u_v])
     mask = cv2.inRange(hsv, lower_bound, upper_bound)
     result = cv2.bitwise_and(frame, frame, mask=mask)    

    # show thresholded image
     cv2.imshow("mask", mask)
     cv2.imshow("result", result)  

     key = cv2.waitKey(1) & 0xFF
     if key == ord("q"):
        break
pipeline.stop()
cv2.destroyAllWindows()
