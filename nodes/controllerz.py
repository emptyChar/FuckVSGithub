#!/usr/bin/env python
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID
from std_msgs.msg import Int32


class Control():
    def __init__(self):
        rospy.init_node("controllerz")
        self.z_sp = -0.5
        self.no_tag_detection = Int32(0)
        self.pidz = PID(1, 0.1, 0.05, setpoint=self.z_sp)
        self.pidz.output_limits = (-0.2, 0.2)
        # define variables
        self.z = -0.7   # y estimate

        self.xy_sub = rospy.Subscriber("depth_estimate",
                                       Float64,
                                       self.on_sub,
                                       queue_size=1)

        self.tag_sub = rospy.Subscriber("no_tag_detection_error",
                                        Int32,
                                        self.on_sub_tag,
                                        queue_size=1)

        self.setpoint_sub = rospy.Subscriber("pose_setpoint",
                                             Point,
                                             self.on_sub_setpointz,
                                             queue_size=1)

        # Publisher
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub_setpointz(self, msg):
        self.z_sp = msg.z
        self.pidz.setpoint = self.z_sp

    def on_sub(self, msg):
        self.z = msg.data
        # Publish lateral thrust
        if self.no_tag_detection.data < 10:
            lat = self.pidz(self.z)
            self.vertical_thrust_pub.publish(Float64(lat))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
