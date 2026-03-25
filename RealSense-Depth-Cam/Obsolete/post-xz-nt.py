# This will connect with the Roborio and publish a table 'Post Detection' that includes:
#     if a post is detected (boolean) 
#     the horizontal position (x) in pixels
#     the depth (distance from camera) in metres
#
# Connects with RealSense camera and:
#   sets clipping distance to 50cm (ignores everything from 3D camera > 50cm away)
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

# Connect to the RoboRIO - NOTE: might need to change server number
NetworkTables.initialize(server='10.72.87.2')

# Get the NetworkTables table to publish values to
table = NetworkTables.getTable('Post Detection')

# Initialize and configure the pipeline
pipeline = rs.pipeline()
config = rs.config()

# Only need the depth stream
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Start the pipeline and get depth scale
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is:", depth_scale)

# Set clipping distance to 50cm
clipping_distance_in_meters = 0.5
clipping_distance = clipping_distance_in_meters / depth_scale

# Create the display window once, outside the loop
cv2.namedWindow('Post Detection', cv2.WINDOW_NORMAL)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())

        # Mask pixels within 50cm with a valid depth reading
        post_mask = np.where(
            (depth_image > 0) & (depth_image < clipping_distance),
            255, 0
        ).astype(np.uint8)

        # Use moments to find the centroid of the masked region
        moments = cv2.moments(post_mask)

        # Build the display image from the mask regardless of detection
        display = cv2.cvtColor(post_mask, cv2.COLOR_GRAY2BGR)

        if moments["m00"] == 0:
            print("No post detected within 50cm")

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
