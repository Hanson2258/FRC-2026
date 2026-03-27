# This code is similar to the more simple program "post-xz-nt.py" but adds 
# a fairly simple layer of object recognition (height/width ration must be > 2).
#
# This will connect with the Roborio and publish a table 'Post Detection' that includes:
#     if a post is detected (boolean) 
#     the horizontal position (x) in pixels
#     the depth (distance from camera) in metres
#
# Connects with RealSense camera and:
#   sets clipping distance to 50cm (ignores everything from 3D camera > 50cm away)
#   makes a grey display-image mask 


import pyrealsense2 as rs
import numpy as np
import cv2
from networktables import NetworkTables

# Connect to the RoboRIO - NOTE: might need to update IP address
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

def find_post(depth_image, clipping_distance):
    """
    Detects a post-like object in the depth image and returns its centre x and depth.

    Strategy:
    - Mask everything beyond 50cm
    - Find contours and filter for tall, narrow shapes (post-like)
    - Return the centre x and median depth of the best candidate
    """

    # Create a mask of pixels within 50cm with a valid depth reading
    post_mask = np.where(
        (depth_image > 0) & (depth_image < clipping_distance),
        255, 0
    ).astype(np.uint8)

    # Apply morphological closing to fill small gaps in the post shape
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 20))
    post_mask = cv2.morphologyEx(post_mask, cv2.MORPH_CLOSE, kernel)

    # Find all contours in the mask
    contours, _ = cv2.findContours(post_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

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
        return post_mask, None, None

    # Get bounding box of the best candidate
    x, y, w, h = cv2.boundingRect(best_contour)

    # Only need centre x since the post is vertical
    centre_x = x + w // 2

    # Get median depth of valid pixels within the bounding box, converted to metres
    depth_roi = depth_image[y:y+h, x:x+w]
    valid_depths = depth_roi[(depth_roi > 0) & (depth_roi < clipping_distance)]

    if len(valid_depths) == 0:
        return post_mask, centre_x, None

    depth_in_meters = float(np.median(valid_depths) * depth_scale)

    return post_mask, centre_x, depth_in_meters

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        depth_image = np.asanyarray(depth_frame.get_data())

        # Detect the post
        post_mask, centre_x, depth_in_meters = find_post(depth_image, clipping_distance)

        # Build display image from the mask
        display = cv2.cvtColor(post_mask, cv2.COLOR_GRAY2BGR)

        if centre_x is None or depth_in_meters is None:
            print("No post detected within 50cm")
            table.putBoolean('post_detected', False)
        else:
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
