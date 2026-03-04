# This will connect with the Roborio and publish a table 'Post Detection' that includes:
#     if a post is detected (boolean) 
#     the horizontal position (x) in pixels
#     the depth (distance from camera) in metres
#
# Connects with RealSense camera and:
#   sets clipping distance (ignores everything from 3D camera > clipping distance)
#   All pixels > clipping distance are set to black
#   A colour mask is then applied based on HSV values
#  
#   makes a grey display-image mask 
#     uses OpenCV "contours" to choose most post-like piece of image:
#         - small contours are ignored as noise (<5 wide or <20 high)
#         - only contours at least twice as tall as wide are considered
#         - contours are scored based on area, the largest area is selected
#         - centre of this contour is considered returned as the x-value in pixels
#
#     as camera gets closer to post, precision should increase as edge noise has less weight
# 
# Results are published to NetworkTable 'Post Detection'
#       x: unit is pixels, relative to LHS of image (640x480): 320 is centre
#       depth: distance from camera in metres
#
# NOTE: future version of code could convert 'x' in pixels to distance in metres
#       This is simple to understand (depth of pixel determines x-offset) 
#       But it needs to compensate image distortion (not as simple!)
#
#
# Java code to retrieve this information: 
#   NetworkTable table = NetworkTableInstance.getDefault().getTable("Post Detection");
#   boolean detected = table.getEntry("post_detected").getBoolean(false);
#   double postX = table.getEntry("post_x").getDouble(0.0);
#   double postDepth = table.getEntry("post_depth").getDouble(0.0);
#


import pyrealsense2 as rs
import numpy as np
import cv2
from networktables import NetworkTables

Post_detected = False

# Set up RealSense camera before checking for RoboRIO connection

# Initialize and configure the pipeline
pipeline = rs.pipeline()
config = rs.config()

# Only need the depth stream
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

# Create the display window once, outside the loop
cv2.namedWindow('Post Detection', cv2.WINDOW_NORMAL)


# UPDATE NEEDED: loop here to check until RoboRIO is ready

# Connect to the RoboRIO

NetworkTables.initialize(server='10.72.87.2')

# Get the NetworkTables table to publish values to
table = NetworkTables.getTable('Post Detection')

# UPDATE NEEDE: pull from RoboRIO NetworkTable to determine Alliance colour
redalliance = True

try:
    while True:
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

        depth_mask0 = np.where(
            (depth_image > 0) & (depth_image < clipping_distance),
            255, 0
            ).astype(np.uint8)

        # clean up the mask
        # get a rectangular structuring element
        dkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 10))

        # Apply morphological closing to fill small gaps in the post shape
#        depth_mask = cv2.morphologyEx(depth_mask0, cv2.MORPH_OPEN, dkernel)
        depth_mask = cv2.morphologyEx(depth_mask0, cv2.MORPH_CLOSE, dkernel)

#        cv2.imshow('depth_mask0', depth_mask0)
#        cv2.imshow('depth_mask', depth_mask)

        depth_masked_image = cv2.bitwise_and(colour_image, colour_image, mask=depth_mask)

        cv2.imshow('depth_masked_image', depth_masked_image)

        # create a colour mask
        # convert to HSV to use HSV values for colour masking
        hsv_image = cv2.cvtColor(depth_masked_image, cv2.COLOR_BGR2HSV)

        ##############################################################################
        # NOTE: best to use ColRngCheck.py to check HSV values with FRC post colours #
        ##############################################################################
        
        # keep red if redalliance
        if redalliance == True:
            # reds wrap around hue: from 170 (past 180 back to 0) around to 10
            lower_red1 = np.array([0, 175, 40])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([130, 97, 42])
            upper_red2 = np.array([180, 255, 255])

            rmask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            rmask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

            # combine the red masks
            colour_mask = rmask1 + rmask2

        # not red alliance, so keep blue pixels
        else:
            lower_blue = np.array([90, 200, 100])    # lower bound for blue
            upper_blue = np.array([125, 255, 255])  # upper bound for blue
            colour_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # clean up colour mask
        ckernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2,5))
        colour_mask1 = cv2.morphologyEx(colour_mask, cv2.MORPH_CLOSE, ckernel)
        colour_mask2 = cv2.morphologyEx(colour_mask1, cv2.MORPH_OPEN,  ckernel)

        # apply colour mask to depth-masked image 
        post_colour_mask_clean = cv2.bitwise_and(depth_masked_image, depth_masked_image, mask=colour_mask2)
        post_colour_mask = cv2.bitwise_and(depth_masked_image, depth_masked_image, mask=colour_mask)

#        cv2.imshow('Colour Image', colour_image)
#        cv2.imshow('Colour Mask0', colour_mask0)
#        cv2.imshow('Colour Mask1', colour_mask1)
#        cv2.imshow('Colour Mask2', colour_mask2)
#        cv2.imshow('post_colour_mask_clean', post_colour_mask_clean)
#        cv2.imshow('post_colour_mask', post_colour_mask)

        # Build the display image from the mask regardless of detection
        display = cv2.cvtColor(colour_mask2, cv2.COLOR_GRAY2BGR)

        # Find all contours in the mask
        contours, _ = cv2.findContours(colour_mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_contour = None
        best_score = 0

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)

            # Filter out very small detections (noise)
            if w < 5 or h < 20:
                continue

            # A post should be significantly taller than it is wide
            aspect_ratio = h / float(w)
            if aspect_ratio < 2.0:
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
            x, y, w, h = cv2.boundingRect(best_contour)

            # Only need centre x since the post is vertical
            centre_x = x + w // 2

           # Get median depth of valid pixels within the bounding box, converted to metres
            depth_roi = depth_image[y:y+h, x:x+w]
            valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < clipping_distance)]
            depth_in_meters = float(np.median(valid_depths) * depth_scale)

            if len(valid_depths) == 0:
                Post_detected = False

            else:
                Post_detected = True
                print(f"Post centre x: {centre_x}  Depth: {depth_in_meters:.3f}m")

                # Publish values to NetworkTables
                table.putBoolean('post_detected', True)
                table.putNumber('post_x', centre_x)
                table.putNumber('post_depth', depth_in_meters)
  
                # Draw a vertical line at the detected x position
                cv2.line(display, (centre_x, 0), (centre_x, display.shape[0]),
                         color=(0, 255, 0), thickness=2)

                # Label at the top of the line
                cv2.putText(display, f"x={centre_x}  {depth_in_meters:.3f}m",
                            (centre_x + 10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if Post_detected == False:
            print("No post detected within clipping distance (" + str(clipping_distance) + ")")
            # Publish a flag to indicate no post is detected
            table.putBoolean('post_detected', False)

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
