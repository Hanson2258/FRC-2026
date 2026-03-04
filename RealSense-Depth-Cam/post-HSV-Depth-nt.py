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
#     uses OpenCV "moments" to determine centre of post horizontally,
#      cv2.moments(post_mask) computes a set of raw moments: 
#        m00 — sum of all pixel values (for our binary mask is the total number of pixels in mask)
#        m10 — sum of all pixel value (1 or 0 for binary mask) multiplied by x coordinate
#              for binary mask just sum up of all x coordinates add up x*pixel_value 
#        m01 — this is for the y coordinate, we are not using.
#     as camera gets closer to post, precision should increase as edge noise has less weight
# 
# Note: this code publishes X and Z coordinates (relative to camera) to NetworkTable 'Post Detection'
#       x-position: unit is pixels, relative to LHS of image (640x480): 320 is centre
#       z-position: distacne from camera in metres
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

redalliance = True

# Connect to the RoboRIO - NOTE: might need to change server number
NetworkTables.initialize(server='10.72.87.2')

# Get the NetworkTables table to publish values to
table = NetworkTables.getTable('Post Detection')

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
#        dkernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7,7))
#        depth_mask1 = cv2.morphologyEx(depth_mask0, cv2.MORPH_CLOSE, dkernel, iterations=2)
#        depth_mask2 = cv2.morphologyEx(depth_mask1, cv2.MORPH_OPEN,  dkernel, iterations=2)

        # Apply morphological closing to fill small gaps in the post shape
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 10))
        depth_mask = cv2.morphologyEx(depth_mask0, cv2.MORPH_CLOSE, kernel)

        cv2.imshow('depth_mask0', depth_mask0)
        cv2.imshow('depth_mask', depth_mask)

        depth_masked_image = cv2.bitwise_and(colour_image, colour_image, mask=depth_mask)


        # Stack depth into 3 channels to match colour image dimensions
#        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
################
        # Mask out everything beyond clipping distance (set to black)
 #       depth_masked_image = np.where(
 #           (depth_image_3d > clipping_distance) | (depth_image_3d <= 0),
 #           0,
 #           colour_image
 #       )
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
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([125, 120, 70])
            upper_red2 = np.array([180, 255, 255])

            rmask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            rmask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

            # combine the red masks
            colour_mask0 = rmask1 + rmask2

        # not red alliance, so keep blue pixels
        else:
            lower_blue = np.array([90, 150, 22])    # lower bound for blue
            upper_blue = np.array([126, 255, 255])  # upper bound for blue
            colour_mask0 = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # clean up colour mask
        ckernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,10))
        colour_mask1 = cv2.morphologyEx(colour_mask0, cv2.MORPH_CLOSE, ckernel, iterations=2)
        colour_mask2 = cv2.morphologyEx(colour_mask1, cv2.MORPH_OPEN,  ckernel, iterations=2)


        # apply colour mask to already depth-masked image, but these are already combined...?
        post_colour_mask_clean = cv2.bitwise_and(depth_masked_image, depth_masked_image, mask=colour_mask2)
        post_colour_mask = cv2.bitwise_and(depth_masked_image, depth_masked_image, mask=colour_mask0)

#        cv2.imshow('Colour Image', colour_image)
#        cv2.imshow('Colour Mask0', colour_mask0)
#        cv2.imshow('Colour Mask1', colour_mask1)
#        cv2.imshow('Colour Mask2', colour_mask2)
        cv2.imshow('post_colour_mask_clean', post_colour_mask_clean)
        cv2.imshow('post_colour_mask', post_colour_mask)
#        masked_image = cv2.bitwise_and(
        

#        post_mask = np.where(
#            (depth_image > 0) & (depth_image < clipping_distance),
#            255, 0
#        ).astype(np.uint8)

        # Use moments to find the centroid of the masked region
        moments = cv2.moments(colour_mask2)

        # Build the display image from the mask regardless of detection
        display = cv2.cvtColor(colour_mask2, cv2.COLOR_GRAY2BGR)

        if moments["m00"] == 0:
            print("No post detected within clipping distance (" + str(clipping_distance) + ")")

            # Publish a flag to indicate no post is detected
            table.putBoolean('post_detected', False)
        else:
            # Only need x from the centroid — post is vertical so y is irrelevant
            centre_x = int(moments["m10"] / moments["m00"])

            # Median depth of all valid close pixels, converted to metres
            valid_depths = depth_image[(depth_image > 0) & (depth_image < clipping_distance)]
            depth_in_meters = float(np.median(valid_depths) * depth_scale)

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

        cv2.imshow('Post Detection', display)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()
