#!/usr/bin/env python
import math
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion

class Control():
    def __init__(self):
        rospy.init_node("controllerYaw")

        self.pidYaw = PID(1, 0.1, 0.05, -math.pi / 4)
        self.pidYaw.output_limits = (-0.1, 0.1)
        # define variables
        self.yaw = -math.pi / 2   

        self.xy_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                       Odometry,
                                       self.on_sub,
                                       queue_size=1)

        # Publisher
        self.yaw_pub = rospy.Publisher("yaw",
                                        Float64,
                                        queue_size=1)

    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub(self, msg):
        q = Quaternion(msg.pose.pose.orientation.x, 0, 0, msg.pose.pose.orientation.w)
        self.yaw = q.radians
        yaw = self.pidYaw(self.yaw)
        self.yaw_pub.publish(yaw)
    

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
