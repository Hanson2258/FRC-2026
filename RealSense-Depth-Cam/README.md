Code resides on the Raspberry Pi with a specific installation (Raspberry Pi OS with virtual environment allowing modified Python binary).

On the Pi, this code is named "post-detect.py" (previously named "post-HSV-Depth-nt.py")

This will connect with the Roborio and publish a table 'PostDetection' that includes:
     if a post is detected (boolean) 
     the horizontal left/right (lateral) position in metres
     the depth (horizontal distance normal front of camera front) in metres

Connects with RealSense camera and:
   0. sets clipping distance (ignores everything from 3D camera > clipping distance)
   1. all pixels > clipping distance are set to black
   2. depth mask is applied to colour image
   3. HSV colour mask is then applied: all pixels out of colour range (red or blue) are set to black
      All pixels in HSV range are set to light grey
   4. OpenCV "contours" used to choose most post-like piece of image:
         - small contours are ignored as noise (<5 wide or <20 high)
         - only contours with minimum aspect ratio (height/width) greater than set value are considered
         - Note: minimum aspect-ratio was originally 2.0, but reduced to 1.6 after testing
         - contours are scored based on area, the largest area is selected
         - future improvement: include average depth of each part of 
         - centre of this contour is considered returned as the x-value in pixels
         - intrinsics are used to convert the x-value to metres
         - as camera gets closer to post, precision should increase as edge noise has less weight
   5. Results are published to NetworkTable 'PostDetection'
         - lateral: horizontal displacement perpendicular to depth-direction, in metres
         - depth: distance from camera (normal to camera-face) in metres

            'post_detected' - Boolean, True if post detected
            'post_lateral' - Number, horizontal centre of the post-mask in pixels (0 is left-side, 480 is max/right-side of image)
            'post_depth' - Number, horizontal distance from camera (depth) in metres

At the time of writing, the plan is to use this camera and code for final alignment during climbing stage. The turret-targeting cameras and April-tags will be used to get "close" to the post (30 to 40cm away). Once the "Post Detection" NetworkTable boolen "post_detected" is TRUE, the RoboRIO climbing code will switch to using the "Post Detection" NetworkTable data, "post_x" and "post_depth" to accurately position so it can attach and climb.

