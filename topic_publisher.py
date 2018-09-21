#!/usr/bin/env python
import rospy
import cv2
import numpy
from std_msgs.msg import Int32

cap = cv2.VideoCapture(0)
 
if (cap.isOpened() == False): 
  print("Unable to read camera feed")

pub = rospy.Publisher('ros_linetracer_msg', Int32, queue_size = 10)
rospy.init_node('topic_publisher')

rate = rospy.Rate(66)
pos = 0
while not rospy.is_shutdown():
	ret, frame = cap.read()

	min_white = 80
	lower_white = numpy.array([min_white, min_white, min_white])
	upper_white = numpy.array([255, 255, 255])

	mask = cv2.inRange(frame, lower_white, upper_white)
	mask = cv2.bitwise_not(mask)

	h, w, d = frame.shape
	search_top = 3*h/4
	search_bot = search_top + 20
	mask[0:search_top, 0:w] = 0
	mask[search_bot:h, 0:w] = 0
	M = cv2.moments(mask)
	if M['m00'] > 0:
		cx = int(M['m10']/M['m00'])
		cy = int(M['m01']/M['m00'])
		cv2.circle(frame, (cx, cy), 20, (0, 0, 255), -1)
		pos = cx - w/2
	pub.publish(pos)
	rate.sleep()

	if ret == True: 
		#cv2.imshow('frame',frame)
 
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break
	else:
		break

cap.release()
out.release()

cv2.destroyAllWindows()
