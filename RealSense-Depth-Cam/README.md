Two initial, working code examples. Both of them set up a NetworkTable that posts if a post was detected and the position in two-dimensions: horizontal (x) and depth (z)
Post detection differs, but both first set a clipping distance of 50cm (everything further away than 50cm is ignored). Given how this code/camera is going to be used,
we are assuming that there will be nothing in front of the camera but the climbing post. 
1. post-xz-nt.py 
     This script is very simple and doesn't perform any object recognition: if there's a pixel, it's assumed to be part of the post. So, the centre of the post is
     just the average of all the x-coordinates of the pixels.

2. post-xz-nt-objrecon.py
    This script starts with the above script, but adds very simple object recognition: anything that is larger than a few pixels has a quick dimension check:
    if the height is at least twice the width, then it will be assumed to be a post. Then the centre of the post is just the average of the x-pixels. 

At the time of writing, the plan is to use the turret-targeting cameras to use April-tags to get "close" to the post (at least within the clipping distance) and move 'in.'
Once the "Post Detection" NetworkTable boolen "post_detected" is TRUE, the RoboRIO climbing code will switch to using the "Post Detection" NetworkTable data,
"post_x" and "post_depth" (from RealSense camera), to accurately position Alpha so it can attach and climb.
