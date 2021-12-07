#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import Point


class Setpoint():
    def __init__(self):
        rospy.init_node("pose_setpoint")
        self.start_time = rospy.get_time()
        self.sp_1 = np.array([0.4, 2, -0.5])
        self.sp_2 = np.array([0.7, 2, -0.5])
        self.sp_3 = np.array([0.7, 2, -0.9])
        self.sp_4 = np.array([0.4, 2, -0.9])
        # 20 = 40secs in real time
        self.duration = 20.0
        self.count = 0
        self.setpoint = Point()
        self.setpoint_pub = rospy.Publisher("pose_setpoint",
                                            Point,
                                            queue_size=1)

    def get_setpoint(self):
        now = rospy.get_time()
        time = now - self.start_time
        if self.count != 0:
            i = time % (self.duration * 2)
            time = i

        if time <= (self.duration):
            self.setpoint.x = self.sp_1[0]
            self.setpoint.y = self.sp_1[1]
            self.setpoint.z = self.sp_1[2]
            
        elif time < (self.duration * 2) and time >= (self.duration):
            self.setpoint.x = self.sp_2[0]
            self.setpoint.y = self.sp_2[1]
            self.setpoint.z = self.sp_2[2]

        elif time < (self.duration * 3) and time >= (self.duration * 2):
            self.setpoint.x = self.sp_3[0]
            self.setpoint.y = self.sp_3[1]
            self.setpoint.z = self.sp_3[2]

        elif time < (self.duration * 4) and time >= (self.duration * 3):
            self.setpoint.x = self.sp_4[0]
            self.setpoint.y = self.sp_4[1]
            self.setpoint.z = self.sp_4[2]
            self.count = 1

        self.setpoint_pub.publish(self.setpoint)
  
    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.get_setpoint()
            rate.sleep()


def main():
    node = Setpoint()
    node.run()


if __name__ == "__main__":
    main()
    