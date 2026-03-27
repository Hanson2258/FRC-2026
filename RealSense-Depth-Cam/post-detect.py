# Version 6.2 (previously named post-HSV-Depth-nt.py)
#
# This will connect with the Roborio and publish a table 'PostDetection' that includes:
#     if a post is detected (boolean) 
#     the horizontal left/right (lateral) position in metres
#     the depth (horizontal distance normal front of camera front) in metres
#
# Connects with RealSense camera and:
#   sets clipping distance (ignores everything from 3D camera > clipping distance)
#   1. all pixels > clipping distance are set to black
#   2. depth mask is applied to colour image
#   3. HSV colour mask is then applied: all pixels out of colour range (red or blue) are set to black
#      All pixels in HSV range are set to light grey
#   4. OpenCV "contours" used to choose most post-like piece of image:
#         - small contours are ignored as noise (<5 wide or <20 high)
#         - only contours with minimum aspect ratio (height/width) greater than set value are considered
#         - Note: minimum aspect-ratio was originally 2.0, but reduced to 1.6 after testing
#         - contours are scored based on area, the largest area is selected
#         - future improvement: include average depth of each part of 
#         - centre of this contour is considered returned as the x-value in pixels
#         - intrinsics are used to convert the x-value to metres
#         - as camera gets closer to post, precision should increase as edge noise has less weight
#   5. Results are published to NetworkTable 'PostDetection'
#         - lateral: horizontal displacement perpendicular to depth-direction, in metres
#         - depth: distance from camera (normal to camera-face) in metres
#
# Java code to retrieve this information: 
#   NetworkTable table = NetworkTableInstance.getDefault().getTable("PostDetection");
#   boolean detected = table.getEntry("post_detected").getBoolean(false);
#   double postX = table.getEntry("post_lateral").getDouble(0.0);
#   double postDepth = table.getEntry("post_depth").getDouble(0.0);
#

import time
import pyrealsense2 as rs
import numpy as np
import cv2
from networktables import NetworkTables
import requests

##########
# set a variable to turn off all displays if we're in actual match
BenchTesting = False 

###########
# Set up RealSense camera before checking for RoboRIO connection

# Initialize and configure the pipeline
pipeline = rs.pipeline()
config = rs.config()
                                        
# get the depth & colour stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the pipeline and get depth scale
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

# Set clipping distance to 50cm
clipping_distance_in_meters = 0.5
clipping_distance = clipping_distance_in_meters / depth_scale

# Set up depth-to-color alignment
align_to = rs.stream.color
align = rs.align(align_to)

# This adds intrinsics so we can convert x-value in pixels to metres later
# instrinsics removes the distortions from the camera lens/detector 
# geometry, so that we get a flat 3D positions from the image
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# If testing, create the display window once, outside the loop
if BenchTesting == True:
    cv2.namedWindow('PostDetection', cv2.WINDOW_NORMAL)

##################################################
# wait for RoboRio to boot and respond on network
RoboReady = False

while RoboReady == False:
    try:
        response = requests.get(f"http://10.73.34.2/nisysapi/server", timeout=5)
        print("Status_code", response.status_code)
        if response.status_code == 404:
            RoboReady = True
            print("RoboRIO is ready.")
        else:
            print("RoboRIO is not ready.")
    except requests.exceptions.RequestException as e:
        print("Error connecting to RoboRIO:", e)

# Connect to the RoboRIO
NetworkTables.initialize(server='10.73.34.2')

# set up variables and NetworkTables outside the loop

# Get the NetworkTables table to publish depth and lateral position
table = NetworkTables.getTable('PostDetection')

# set redalliance True or False 
redalliance = False

if BenchTesting == False:
    fmsTable = NetworkTables.getTable('FMSInfo')
    time.sleep(1)
    redalliance = fmsTable.getBoolean('IsRedAlliance', None)

print("red alliance is", redalliance)

# default to no post detected
Post_detected = False

try:
    while True:
        redcheck = fmsTable.getBoolean('IsRedAlliance', None)
        if redcheck != None:
            redalliance = redcheck

        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        colour_frame = frames.get_color_frame()

        # Align depth frame to colour frame
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        colour_frame = aligned_frames.get_color_frame()

        # Skip if either frame is missing
        if not aligned_depth_frame or not colour_frame:
            continue

        # Convert frames to NumPy arrays
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        colour_image = np.asanyarray(colour_frame.get_data())

        # mask out all pixels further than the clipping distance
        depth_mask0 = np.where(
            (depth_image > 0) & (depth_image < clipping_distance),
            255, 0
            ).astype(np.uint8)

        # clean up depth mask
        # get a rectangular structuring element
        dkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 10))
        # Apply morphological closing to fill small gaps in the post shape
        depth_mask = cv2.morphologyEx(depth_mask0, cv2.MORPH_CLOSE, dkernel)
        # apply depth mask (cleaned) to colour image
        depth_masked_image = cv2.bitwise_and(colour_image, colour_image, mask=depth_mask)

        if BenchTesting == True:
            cv2.imshow('depth_mask0', depth_mask0)
            #cv2.imshow('depth_mask', depth_mask)
            cv2.imshow('depth_masked_image', depth_masked_image)

        # create a colour mask
        # convert to use HSV values for colour masking
        hsv_image = cv2.cvtColor(depth_masked_image, cv2.COLOR_BGR2HSV)

        ##################################################
        # NOTE: fine tune HSV values to FRC post colours #
        #       using ColRngCheck.py                     #
        ##################################################
        
        # red alliance so keep red pixels
        if redalliance == True:
            # reds wrap around hue: from 170 (past 180 back to 0) around to 10
            lower_red1 = np.array([0, 175, 40])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([130, 97, 42])
            upper_red2 = np.array([180, 255, 255])

            rmask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            rmask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

            # combine the red masks
            colour_depth_mask0 = rmask1 + rmask2

        # not red alliance, so keep blue pixels
        else:
            lower_blue = np.array([90, 200, 100])   # lower bound for blue
            upper_blue = np.array([125, 255, 255])  # upper bound for blue
            colour_depth_mask0 = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # clean up colour mask using same method as depth, but smaller rectangular element
        ckernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,5))
        colour_depth_mask = cv2.morphologyEx(colour_depth_mask0, cv2.MORPH_CLOSE, ckernel)

        if BenchTesting == True:
            cv2.imshow('Colour Image', colour_image)
            cv2.imshow('colour_depth_mask0', colour_depth_mask0)
            cv2.imshow('colour_depth_mask', colour_depth_mask)
            # Build the display image from the mask regardless of detection
            display = cv2.cvtColor(colour_depth_mask, cv2.COLOR_GRAY2BGR)

        # Find all contours in the mask
        contours, _ = cv2.findContours(colour_depth_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_score = 0

        for contour in contours:
            x, y, width, height= cv2.boundingRect(contour)

            # Filter out very small detections (noise)
            if width < 5 or height< 20:
                continue

            # A post should be significantly taller than it is wide
            aspect_ratio = height/ float(width)
            if aspect_ratio < 1.6:
                continue

            # Score contours: prefer taller, narrower shapes with more area
            area = cv2.contourArea(contour)
            score = aspect_ratio * area

            if score > best_score:
                best_score = score
                best_contour = contour

        if best_contour is None:
            Post_detected = False

        else:
            # Get bounding box of the best candidate
            x, y, width, height = cv2.boundingRect(best_contour)
            
            # Only need centre x since the post is vertical
            centre_x = x + width // 2

           # Get median depth of valid pixels within the bounding box, converted to metres
            depth_roi = depth_image[y:y+height, x:x+width]
            valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < clipping_distance)]
            depth_in_meters = float(np.median(valid_depths) * depth_scale)

            # Get position in metres in 3D, using the intrinsics 
            point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [centre_x, y], depth_in_meters)
            centre_x_m = point_3d[0]
            if BenchTesting == True:
                print(f"3d x {point_3d[0]}  3d y {point_3d[1]}    3d z {point_3d[2]}")

            if len(valid_depths) == 0:
                Post_detected = False

            else:
                Post_detected = True

                # Publish values to NetworkTables
                table.putBoolean('post_detected', True)
                table.putNumber('post_lateral', centre_x_m)
                table.putNumber('post_depth', depth_in_meters)
  
                if BenchTesting == True:
                    print(f"Post centre x: {centre_x}  Depth: {depth_in_meters:.3f}m")
                    # Draw a vertical line at the detected x position
                    cv2.line(display, (centre_x, 0), (centre_x, display.shape[0]),
                             color=(0, 255, 0), thickness=2)

                    # Label at the top of the line
                    #cv2.putText(display, f"x={centre_x}  {depth_in_meters:.3f}m",
                    cv2.putText(display, f"x mm={centre_x_m * 1000}  {depth_in_meters:.3f}m",
                                (centre_x + 10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if Post_detected == False:
            # Publish a flag to indicate no post is detected
            table.putBoolean('post_detected', False)
            if BenchTesting == True:
                    print("No post detected within clipping distance (" + str(clipping_distance) + ")")

        if BenchTesting == True:
            cv2.imshow('Post Detection', display)
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            else:
                # add ability to switch between red and blue masking by pressing key
                if key & 0xFF == ord('r'):
                    redalliance = True
                if key & 0xFF == ord('b'):
                    redalliance = False

# Would be better to add exception and else to cover errors

finally:
    pipeline.stop()

