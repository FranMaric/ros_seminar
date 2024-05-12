#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from math import pi
import numpy as np


current_image = None

def image_callback(data):
    global current_image

    current_image = data

bridge = CvBridge()

def save_jpg_image(index):
    if current_image == None:
        print('current_image is None. Try again later')
        return
    
    try:
        cv2_img = bridge.imgmsg_to_cv2(current_image, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        img_name = 'images/camera_image_' + str(index) + '.jpeg'
        cv2.imwrite(img_name, cv2_img)
    rospy.loginfo('Saved image as ' + os.getcwd() + '/' + img_name)


def set_waypoint(x, y, z, rotation):
    pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)

    rospy.init_node('seminar', anonymous=True)

    goal = PoseStamped()

    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "base_link"

    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.position.z = z

    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = rotation
    goal.pose.orientation.w = 0.0
    
    pub.publish(goal)


def go_through_one_row(x):
    for y in range(3):
        for z in range(3):
            set_waypoint(x, 6 + y * 7, 1 + z * 3, 0)
            rospy.sleep(6) # time it takes for the drone the get to the waypoint
            save_jpg_image(str(y) + '_' + str(z))


def count_red_fruit_in_image(image_filename):
    img = cv2.imread(image_filename)

    lower_red = np.array([1,84,160])
    upper_red = np.array([20,211,255])

    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    gray_blurred_img = cv2.medianBlur(gray_img, 5)

    detected_circles = cv2.HoughCircles(gray_blurred_img, cv2.HOUGH_GRADIENT, 0.9, 20, param1 = 50, param2 = 26, minRadius = 5, maxRadius = 40) 

    red_fruit_count = 0

    if detected_circles is not None:
        # Convert the circle parameters a, b and r to integers. 
        detected_circles = np.uint16(np.around(detected_circles)) 

        for pt in detected_circles[0, :]: 
            x, y, r = pt[0], pt[1], pt[2] 

            bgr_pixel_value = img[y][x]
            hsv_pixel_value = cv2.cvtColor(np.uint8([[bgr_pixel_value]]), cv2.COLOR_BGR2HSV)[0][0]

            circle_color = (0, 255, 0)

            if hsv_pixel_value[0] <= 30:
                red_fruit_count += 1
                circle_color = (255, 0, 0)

            cv2.circle(img, (x, y), r, circle_color, 2) 

            cv2.circle(img, (x, y), 1, (0, 0, 255), 3)

        cv2.imshow("Detected Circle", img)
        cv2.waitKey(0)

    return red_fruit_count


if __name__ == '__main__':
    rospy.Subscriber('/red/camera/color/image_raw', Image, image_callback)

    set_waypoint(1, 1, 1, 0)
    go_through_one_row(1)

    red_fruit_count = 0

    for root, dirs, files in os.walk('/images'):
        for file in files:
            file_path = os.path.join(root, file)
            
            red_fruit_count += count_red_fruit_in_image(file_path)
    
    print(red_fruit_count)
