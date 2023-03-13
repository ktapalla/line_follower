# line_follower

The line follower assignment can be ran by entering 'roslaunch line_followe line.launch'.
Essentially, the code uses a turtlebot3 waffle robot to follow a yellow line. It is placed in
front of/on the line upon startup, and uses the images it receives through the camera to continue 
following it. It uses a lower and upper bound to account for the yellow line, and uses that to turn 
the image into black and white. From there, it has an easier time differentiate/separating the 
yellow line from the rest of the world. It uses a 'centroid', which is shown as a red dot/circle in the
'image' window to keep track of the line and where it should go. Once it nears the end of the line, it 
stops and turns until it can see the line again, and follows it in the opposite direction, thus allowing 
it to follow the line infinitely. However, due to the position of the camera, it comes to a stop slightly 
before the actual end of the yellow line before turning around. This is because it can no longer see the 
line after a certain point. 
