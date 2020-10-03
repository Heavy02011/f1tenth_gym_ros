#!/usr/bin/env python

# code base by John Chen / Udacity SDC
# adapted by Rainer Bareiss, 10/2020

# start: roslaunch f1tenth_gym_ros pln-dump-xy.launch
#  <param name="path" value="~/catkin_ws/data/track.csv" />

import argparse
import rospy
from std_msgs.msg import Int32
#from geometry_msgs.msg import PoseStamped
#from styx_msgs.msg import Lane, Waypoint
#import math
#import sys
#import numpy as np
import csv

from nav_msgs.msg import Odometry # rbx

class DumpCurrentPos():
    def __init__(self, outfile):
        # initialize and subscribe to the current position and waypoint base topic
        rospy.init_node('pln_current_pos_dump')
        sub1 = rospy.Subscriber('/odom', Odometry, self.pose_cb)
        self.pose = None
        self.outfile = outfile
        print("writing xy to file = ",self.outfile)
        self.fieldname = ['x', 'y', 'z', 'ax', 'ay', 'az', 'aw']
        self.log_file = open(self.outfile, 'w')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=self.fieldname)
        self.log_writer.writeheader()
        rospy.spin()

    def pose_cb(self, msg):
        """Grab the pose of the car
        Args:
           msg (PoseStamped): car pose with (x,y,z) and rotation (x,y,z,q)
        """
        self.pose = msg
        self.log_writer.writerow({
            'x': self.pose.pose.position.x,
            'y': self.pose.pose.position.y,
            'z': self.pose.pose.position.z,
            'ax': self.pose.pose.orientation.x,
            'ay': self.pose.pose.orientation.y,
            'az': self.pose.pose.orientation.z,
            'aw': self.pose.pose.orientation.w
            })
        self.log_file.flush()


if __name__ == "__main__":
    #parser = argparse.ArgumentParser(description='PLN: Udacity SDC System Integration, Waypoints Dumper')
    #parser.add_argument('outfile', type=str, help='current pose save file, first point is the starting point')
    #args = parser.parse_args()

    outfile = "/home/rainer/catkin_ws/data/track.csv"
    
    try:
        #DumpCurrentPos(args.outfile)
        DumpCurrentPos(outfile)

    except rospy.ROSInterruptException:
        rospy.logerr('Could not dump current pose.')