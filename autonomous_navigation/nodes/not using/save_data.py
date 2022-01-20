#! /usr/bin/env python

from os import sep
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry

class SaveData:
    def __init__(self):
        rospy.init_node("save_position")
        self.s_data = PoseStamped()
        self.xy_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                       Odometry,
                                       self.on_sub,
                                       queue_size=1)

        self.data_list = []
        


        self.last_position = rospy.Publisher("data_tuple",
                                                   Point,
                                                   queue_size=1)
        
        

    def on_sub(self, msg):
        self.s_data.header = msg.header
        self.s_data.pose.position = msg.pose.pose.position
        self.data_list.append([self.s_data.header.stamp.secs, self.s_data.pose.position.x, self.s_data.pose.position.y, self.s_data.pose.position.z])
        print(*self.data_list, sep= "\n ")
        rospy.sleep(3)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()


def main():
    node = SaveData()
    node.run()


if __name__ == "__main__":
    main()

        