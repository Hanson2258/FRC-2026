Two initial, working code examples. Both of them set up a NetworkTable that publishes 3 values:
     'post_detected' - Boolean, True if post detected
     'post_x' - Number, horizontal centre of the post-mask in pixels (0 is left-side, 480 is max/right-side of image)
     'post_depth' - Number, horizontal distance from camera (depth) in metres

At the time of writing, the plan if to use this camera and code for final-alignment during climbing stage. The turret-targeting cameras and April-tags will be used to get "close" to the post (30 to 40cm away). Once the "Post Detection" NetworkTable boolen "post_detected" is TRUE, the RoboRIO climbing code will switch to using the "Post Detection" NetworkTable data, "post_x" and "post_depth" to accurately position Alpha so it can attach and climb.

Because of this, we expect nothing in the field of vision other than the post (and the wall, which is >50cm behind the climber post) during final alignment.

NOTE: once the camera is positioned, parts of Alpha (like the bumpers) might be within the RealSense field of vision. If so, the Region of Interest (ROI) may need to be set to exclude the lower frame. Alternatively, there may be another method to crop the image before calculating the 'x' position.

The two post detection algorithms differ, but both scripts set a 50cm clipping distance (everything further away than 50cm is ignored).

1. post-xz-nt.py 
     This script is very simple with no object recognition: if there's a pixel, it's assumed to be part of the post. So, the centre of the post is just the average of all the x-coordinates of the pixels.

2. post-xz-nt-objrecon.py
    This script starts with the above script, but adds very simple object recognition: anything that is larger than a few pixels has a quick dimension check: if the height is at least twice the width, then it will be assumed to be a post. Then the centre of the post is just the average of the x-pixels. 
