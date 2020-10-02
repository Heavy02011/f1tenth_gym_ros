#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

import random # rbx
from plnLaserScan import LidarScan


class Agent(object):
    def __init__(self):
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

    def scan_callback(self, scan_msg):
        # analize scan
        lidarScan = LidarScan(scan_msg)
        dist_left, dist_front, dist_right = lidarScan.get_ranges()
        #print(dist_left, dist_front, dist_right)
        track_width = dist_left + dist_right
        #print(track_width)
        steering = -(track_width / 2. - dist_left) / 2.

        # print('got scan, now plan')
        drive = AckermannDriveStamped()

        # RBX
        st = steering # random.random() * 2.0 - 1.0
        th = 1.0 #0.3
        # RBX
        
        drive.drive.speed = th #0.1
        drive.drive.steering_angle = st #-0.01
        #print(st, th)

        self.drive_pub.publish(drive)



if __name__ == '__main__':
    rospy.init_node('pln_racer')
    dummy_agent = Agent()
    rospy.spin()