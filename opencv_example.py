#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("rrbot/camera1/image_circle",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("rrbot/camera1/image_raw",Image,self.callback)
    self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.last_proportional = 0
    self.last_middle = 0

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    ret, thresh = cv2.threshold(cv_image, 100, 255, cv2.THRESH_BINARY)
    startPixel = -1
    endPixel = -1
 
    for i in range(800):
        val = thresh[650,i,0]   # val is 0 when it sees black, 255 when it sees white
        
        if val == 0:    # if you see black
            if startPixel == -1:
                startPixel = i
    
        else:           # if you see white
            if endPixel == -1 and startPixel != -1:
                endPixel = i
                break

    middle = startPixel + (endPixel - startPixel)//2

    if middle <= startPixel + 10:
      middle = self.last_middle

    cv2.circle(thresh, (middle,650), 20, (255,205,195), -1) # y,x coordinate

    proportional = abs(middle-400)
    derivative = abs(proportional-self.last_proportional)
    power_difference = proportional/80 + derivative/90;
    print(str(proportional) + "   " + str(self.last_proportional) + "   " + str(power_difference))

    move = Twist()

    if middle < 400:
      move.angular.z = 0.5 + float(0.4 * power_difference)
    if middle > 400:
      move.angular.z = -(0.5 + float(0.4 * power_difference))

    move.linear.x = 0.5
    self.move_pub.publish(move)

    self.last_middle = middle
    self.last_proportional = proportional

    cv2.imshow("Image window", thresh)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)