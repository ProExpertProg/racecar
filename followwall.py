#!/usr/bin/env python2

import numpy as np

import time, math
import rospy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:

  def __init__(self):
    LIDAR_TOPIC = "/scan"
    DRIVE_TOPIC = "/vesc/high_level/ackermann_cmd_mux/input/nav_0"

    ##################################################
    # TODO >>>
    # Do any preprocessing you need to here.
    ##################################################

    ##################################################
    # <<< TODO
    ##################################################

    # Initialize a publisher for drive messages
    self.drive_pub = rospy.Publisher(
        DRIVE_TOPIC,
        AckermannDriveStamped,
        queue_size=1)

    # Subscribe to the laser scan data
    rospy.Subscriber(
        LIDAR_TOPIC,
        LaserScan,
        self.callback)

  def callback(self, msg):
    ##################################################
    # TODO >>>
    # Make the robot move, but don't let it get too
    # close to obstacles!
    ##################################################

    filtered_ranges = [360 for i in range(180)] + list(msg.ranges[180:900]) + [360 for i in range(900, len(msg.ranges))]
    right_dist = sum(filtered_ranges[180:260]) / 80
    left_dist = sum(filtered_ranges[820:900]) / 80
    min_dist = min(filtered_ranges[480:600])
    print('Min dist:', min_dist, '| Right dist:', right_dist, '| Left dist:', left_dist)
    
    max_angle = 0.34
    k_p = 2.5
    k_d = 0.4
    error = 0
    
    drive = AckermannDriveStamped()
    drive.drive.speed = 0 if min_dist < 0.4 else 0.4
    '''
    if right_dist>0.7 and right_dist<1:
        drive.drive.steering_angle = thresh_angle*-1*k_p*(right_dist-0.65)
    elif right_dist<0.6 and right_dist>0.2:
        drive.drive.steering_angle = thresh_angle*-1*k_p* (right_dist-0.65)
    '''
    if right_dist>0.2 and right_dist<1:
        drive.drive.steering_angle = -max_angle*k_p*(right_dist-0.6)-max_angle*k_d*(right_dist-0.6-error)
        error = right_dist-0.6
    elif right_dist >1: 
        drive.drive.steering_angle = -max_angle
    elif right_dist < 0.2:
        drive.drive.steering_angle = max_angle
    
    '''
    if right_dist > 0.7:
        drive.drive.steering_angle = -0.15
    if right_dist < 0.6:
       drive.drive.steering_angle = 0.15
    '''
    #drive.drive.steering_angle = 0.5*math.sin(time.time())
    #print(drive.drive.steering_angle)
    self.drive_pub.publish(drive)
  
    ##################################################
    # <<< TODO
    ##################################################

if __name__ == "__main__":
  rospy.init_node("Safety")
  mover = WallFollower()
  rospy.spin()
