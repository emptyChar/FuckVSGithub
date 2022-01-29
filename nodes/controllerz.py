#!/usr/bin/env python
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from simple_pid import PID
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped


class Control():
    def __init__(self):
        rospy.init_node("controllerz")
        self.action = 0
        self.z_sp = -0.3
        self.no_tag_detection = Int32(0)
        self.is_following = True
        self.pidz = PID(1, 0.1, 0.05, setpoint=self.z_sp)
        self.pidz.output_limits = (-0.5, 0.5)
        # define variables
        self.z = -0.7   # y estimate

        # rostopic echo /bluerov/ground_truth/state/pose/pose/position
        # to get the ground truth subscribe to /bluerov/ground_truth/state
        # and then set self.z = msg.pose.pose.position.z

        self.xy_sub = rospy.Subscriber("/bluerov/mavros/vision_pose/pose_cov",
                                       PoseWithCovarianceStamped,
                                       self.on_sub,
                                       queue_size=1)

        self.setpoint_sub = rospy.Subscriber("position_setpoint",
                                             Point,
                                             self.on_sub_setpoint,
                                             queue_size=1)

        self.isFollowing = rospy.Subscriber("is_following",
                                            Bool,
                                            self.on_sub_following,
                                            queue_size=1)

        self.xy_sub_marker = rospy.Subscriber("markerPosition",
                                              Point,
                                              self.on_sub_marker,
                                              queue_size=1)

        # Publisher
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

    def on_sub_following(self, msg):
        self.is_following = bool(msg.data)

    def on_sub_setpoint(self, msg):
        self.z_sp = msg.z

    def on_sub_marker(self, msg):
        # a = 1
        if self.is_following is True:
            self.z = msg.y
            self.pidz.setpoint = 0
            lat = self.pidz(self.z)
            self.vertical_thrust_pub.publish(Float64(lat))

    def on_sub(self, msg):
        if self.is_following is False:
            self.pidz.setpoint = self.z_sp
            self.z = msg.pose.pose.position.z
            # Publish lateral thrust
            vert = self.pidz(self.z)
            self.vertical_thrust_pub.publish(Float64(vert))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
