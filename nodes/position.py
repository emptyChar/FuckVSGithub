#!/usr/bin/env python
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
import math
from std_msgs.msg       import Int32
# get the distance to the 4 known markers and
# calculates the position of the point


class Trilateration():

    def __init__(self):
        rospy.init_node("localization")        
        #subscribers and publishers
        self.distance_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                             RangeMeasurementArray,
                                             self.on_sub_position,
                                             queue_size=1)

        self.position_pub = rospy.Publisher("noisy_position",
                                            Point,
                                            queue_size=1)
        
        self.tag_pub = rospy.Publisher("no_tag_detection_error",
                                            Int32,
                                            queue_size=1)

       

    def on_sub_position(self, msg):
        pos = msg.pose.pose.position
        


    def run(self):            
            while not rospy.is_shutdown():
                rospy.spin    

def main():
    tri = Trilateration    
    tri.run()


if __name__ == "__main__":
    main()