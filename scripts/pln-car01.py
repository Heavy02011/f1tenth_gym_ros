#!/usr/bin/env python

# start: roslaunch f1tenth_gym_ros pln-car01.launch

# fixing tf import: https://answers.ros.org/question/326226/importerror-dynamic-module-does-not-define-module-export-function-pyinit__tf2/

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from nav_msgs.msg import Odometry # rbx

import random # rbx
import csv # rbx
import math #rbx

from plnLaserScan import LidarScan
import tf

RADDEG = 180. / math.pi 
DEGRAD = math.pi / 180.


class Agent(object):
    def __init__(self):
        self.drive_pub = rospy.Publisher('/drive', AckermannDriveStamped, queue_size=1)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=1)

        self.scan_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

        #self.scan_sub = rospy.Subscriber('/initialpose', LaserScan, self.initialpose_callback, queue_size=1)

        self.outfile = "/home/rainer/catkin_ws/data/track.csv"
        print("writing xy to file = ",self.outfile)
        self.fieldname = ['x', 'y', 'z', 'left_x', 'left_y', 'center_x', 'center_y','right_x', 'right_y','ax', 'ay', 'az', 'aw']
        self.log_file = open(self.outfile, 'w')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.fieldname)
        self.log_writer.writeheader()

        self.dist_left  = 0
        self.dist_front = 0
        self.dist_right = 0
        self.idx_left   = 0
        self.idx_front  = 0
        self.idx_right  = 0

    def scan_callback(self, scan_msg):
        # analize scan
        lidarScan = LidarScan(scan_msg)
        self.dist_left, self.dist_front, self.dist_right, self.idx_left, self.idx_front, self.idx_right, self.ranges, self.lidar_angles = lidarScan.get_ranges()
        #print(dist_left, dist_front, dist_right)
        track_width = self.dist_left + self.dist_right
        #print(track_width)
        steering = -(track_width / 2. - self.dist_left) / 2.

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

    def odom_callback(self, odom_msg):

        # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
        quaternion = (
          odom_msg.pose.pose.orientation.x,
          odom_msg.pose.pose.orientation.y,
          odom_msg.pose.pose.orientation.z,
          odom_msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print(roll*RADDEG, pitch*RADDEG, yaw*RADDEG)

        #print(odom_msg)
        car_x = odom_msg.pose.pose.position.x
        car_y = odom_msg.pose.pose.position.y

        # left lane
        left_x = car_x + math.cos(self.lidar_angles[self.idx_left]+yaw) * self.dist_left
        left_y = car_y + math.sin(self.lidar_angles[self.idx_left]+yaw) * self.dist_left

        # right lane
        right_x = car_x + math.cos(self.lidar_angles[self.idx_right]+yaw) * self.dist_right
        right_y = car_y + math.sin(self.lidar_angles[self.idx_right]+yaw) * self.dist_right

        # center lane
        center_x = (left_x + right_x) / 2.
        center_y = (left_y + right_y) / 2.



        #print(car_x, car_y)
        self.log_writer.writerow({
            'x':  odom_msg.pose.pose.position.x,
            'y':  odom_msg.pose.pose.position.y,
            'z':  odom_msg.pose.pose.position.z,
            'left_x' : left_x,
            'left_y' : left_y,
            'center_x' : center_x,
            'center_y' : center_y,
            'right_x' : right_x,
            'right_y' : right_y,
            'ax': odom_msg.pose.pose.orientation.x,
            'ay': odom_msg.pose.pose.orientation.y,
            'az': odom_msg.pose.pose.orientation.z,
            'aw': odom_msg.pose.pose.orientation.w
            })
        self.log_file.flush()

if __name__ == '__main__':
    rospy.init_node('pln_racer')
    dummy_agent = Agent()
    rospy.spin()

"""
pose: 
  pose: 
    position: 
      x: 3.10414228834
      y: 0.238839850503
      z: 0.0
    orientation: 
      x: -0.0
      y: 0.0
      z: 0.0591281509751
      w: -0.998250400332
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
twist: 
  twist: 
    linear: 
      x: 1.0
      y: 0.0
      z: 0.0
    angular: 
      x: 0.0
      y: 0.0
      z: -0.135837812271
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

"""