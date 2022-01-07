#!/usr/bin/env python
from numpy.matrixlib.defmatrix import matrix
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
import math
from std_msgs.msg       import Int32
from pyquaternion import Quaternion
from nav_msgs.msg import Odometry
# get the distance to the 4 known markers and
# calculates the position of the point


class Trilateration():

    def __init__(self):
        rospy.init_node("localization")      
        self.pos = np.array([0, 0, 0])  
        self.set = np.array([0, 0, 0])
        #subscribers and publishers
        self.position_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                             Odometry,
                                             self.on_sub_position,
                                             queue_size=1)
        self.setpoint_sub = rospy.Subscriber("pose_setpoint",
                                             Point,
                                             self.on_sub_setpoint,
                                             queue_size=1)
        self.local_position_pub = rospy.Publisher("local_position",
                                            Point,
                                            queue_size=1)

    def on_sub_position(self, msg):
        self.pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = Quaternion(msg.pose.pose.orientation.x, 0, 0, msg.pose.pose.orientation.w)
        transMarix = np.array([[math.cos(q.radians), math.sin(q.radians), 0],
                      [-math.sin(q.radians), math.cos(q.radians),0],
                      [0,0,1]])
        localPosition = np.dot(transMarix,(-self.pos + self.set))
        self.local_position_pub.publish(localPosition)

    
    def on_sub_setpoint(self, msg):
        self.set = np.array([msg.x, msg.y, msg.z])

    def run(self):            
            while not rospy.is_shutdown():
                rospy.spin    

def main():
    tri = Trilateration    
    tri.run()


if __name__ == "__main__":
    main()