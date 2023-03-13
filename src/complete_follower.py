#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import math


def ang_vel_control(x):
    return -1/(1+math.e**x) + 0.5



class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # Subscribes to /camera/rgb/image_raw --> originally /camera/image, but it wasn't giving me the Images from the camera
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        # Publisher for the red centroid dot (this is what the robot tries to follow as it centers it to the line)
        self.centroid_pub = rospy.Publisher('/centroid', Image, queue_size=1)
        # Publisher for Twist commands --> in charge of movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    def image_callback(self, msg):

        # get image from camera
        image = self.bridge.imgmsg_to_cv2(msg)
        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # bounds for what can be perceived as yellow
        lower_yellow = numpy.array([ 40, 0, 0])
        upper_yellow = numpy.array([ 120, 255, 255])
        # makes everything black and white --> yellow line turns white, everything else is black, follows the yellow-turned-white
        # helps differentiate between what needs to be follower
        # only shows/takes into account bottom portion of screen
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        # shows more of the screen --> the rest of the line is visible from further from the robot
        masked = cv2.bitwise_and(image, image, mask=mask)
        # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        # windows showing what is seen/understood by the robot for the mask and masked images (what the robot sees)
        cv2.imshow("mask", mask)
        cv2.imshow("masked", masked)
        
    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        self.logcount += 1
        print("M00 %d %d" % (M['m00'], self.logcount))

        # if the yellow line is visible, this is the code to follow it
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00']) + 100
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = 0.2
            ang_vel  = ang_vel_control(-float(err) / 100)
            self.twist.angular.z = ang_vel
            self.cmd_vel_pub.publish(self.twist)
            self.centroid_pub.publish(self.bridge.cv2_to_imgmsg(image))
        cv2.imshow("image", image)
        cv2.waitKey(3)

        # if the yellow line is no longer visible, this is the code to follow it
        # M['m00'] == 0 --> it can't see a yellow line anymore
        # it stops moving and starts turning around until the line is back in its view, then starts following it again once it can see it 
        # this assumes that it was previously following the line/it is behind it
        # stops a bit before it actually gets to the end of the yellow line due to camera placement on the robot and where it is able to see
        if M['m00'] == 0:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.25
            self.cmd_vel_pub.publish(self.twist)
            self.centroid_pub.publish(self.bridge.cv2_to_imgmsg(image))
        cv2.imshow("image", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
