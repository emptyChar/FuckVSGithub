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
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# controller so that the robot always faces the wall with the tags

class Control():
    def __init__(self):
        rospy.init_node("controllerYaw")

        self.action = 0
        self.pidYaw = PID(1, 0.1, 0.05, math.pi/2)
        # define variables
        self.yaw = 0

        self.is_following = True

        self.xy_sub = rospy.Subscriber("/bluerov/mavros/vision_pose/pose_cov",
                                       PoseWithCovarianceStamped,
                                       self.on_sub,
                                       queue_size=1)
        self.yaw_sub = rospy.Subscriber("markerAngle",
                                        Float64,
                                        self.on_sub_marker,
                                        queue_size=1)

        self.isFollowing = rospy.Subscriber("is_following",
                                            Bool,
                                            self.on_sub_following,
                                            queue_size=1)
        # Publisher
        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)

        self.angle_pub = rospy.Publisher("angle_bluerov",
                                         Float64,
                                         queue_size=1)

    def on_sub_following(self, msg):
        self.is_following = bool(msg.data)

    def on_sub(self, msg):
        if self.is_following is False:
            q = Quaternion([msg.pose.pose.orientation.z, 0, 0, msg.pose.pose.orientation.w])
            self.yaw = q.radians
            # self.pidYaw.setpoint = self.yaw
            yaw = -self.pidYaw(self.yaw)
            self.angle_pub.publish(Float64(self.yaw))
            self.yaw_pub.publish(Float64(yaw))

    def on_sub_marker(self, msg):  
        # a = 1      
        if self.is_following is True:
            self.pidYaw.setpoint = 0
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
