#!/usr/bin/env python
import math
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg       import Float64
from simple_pid import PID
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from pyquaternion import Quaternion


# controller so that the robot always faces the wall with the tags

class Control():
    def __init__(self):        
        rospy.init_node("controllerYaw")

        self.action = 0
        self.pidYaw = PID(1, 0.1, 0.05, 0)        
        # define variables
        self.yaw = 0


        self.xy_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                       Odometry,
                                       self.on_sub,
                                       queue_size=1)
        self.yaw_sub = rospy.Subscriber("markerAngle",
                                        Float64,
                                        self.on_sub_angle,
                                        queue_size=1)
        # Publisher
        self.yaw_pub = rospy.Publisher("yaw",
                                        Float64,
                                        queue_size=1)

        

    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub(self, msg):
        if self.action == 0:
            q = Quaternion([msg.pose.pose.orientation.z, 0, 0, msg.pose.pose.orientation.w])
            self.yaw = q.radians
            yaw = -self.pidYaw(self.yaw)
            
            self.yaw_pub.publish(Float64(yaw))
        
    def on_sub_angle(self, msg):
        if self.action != 0:
            self.yaw = msg.data
            yaw = self.pidYaw(self.yaw)        
            self.yaw_pub.publish(Float64(yaw))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
