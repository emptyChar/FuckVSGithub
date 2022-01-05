#!/usr/bin/env python
import math
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID
from std_msgs.msg import Int32


class Control():
    def __init__(self):
        rospy.init_node("controllerYaw")
        
        self.x = 0.7
        self.y = 2.0
        self.setYaw = self.calcAngle(self.x, self.y)

        self.no_tag_detection = Int32(0)s
        self.pidYaw = PID(1, 0.1, 0.05, setpoint=self.setYaw)
        self.pidYaw.output_limits = (-0.2, 0.2)
        # define variables
        self.yaw = -math.pi / 2   # y estimate

        self.xy_sub = rospy.Subscriber("noisy_position",
                                        Point,
                                        self.on_sub,
                                        queue_size=1)

        self.tag_sub = rospy.Subscriber("no_tag_detection_error",
                                        Int32,
                                        self.on_sub_tag,
                                        queue_size=1)

        self.setpoint_sub = rospy.Subscriber("noisy_angle",
                                             Float64,
                                             self.on_sub_setpointYaw,
                                             queue_size=1)

        # Publisher
        self.setpoint_yaw_pub = rospy.Publisher("setpoint_yaw",
                                        Float64,
                                        queue_size=1)

        self.yaw_pub = rospy.Publisher("yaw",
                                        Float64,
                                        queue_size=1)

    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub_setpointYaw(self, msg):
        self.yaw = msg.data        
        self.setpoint_yaw_pub.publish(self.setYaw)
        self.pidYaw.setpoint = self.setYaw
        yaw_thrust = -self.pidYaw(self.yaw)
        self.yaw_pub.publish(yaw_thrust)

    def on_sub(self, msg):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.setYaw = self.calcAngle(self.x, self.y)


    def calcAngle(self, x, y):
        if x < 0.8:
            angle = -math.atan((3.35-y)/abs(x - 0.8))
        else:
            angle = math.atan((3.35-y)/abs(x - 0.8)) - math.pi
        return angle

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
