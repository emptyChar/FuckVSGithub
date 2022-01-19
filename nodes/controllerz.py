#!/usr/bin/env python
import rospy
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
# from rospy.topics import Publisher
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from simple_pid import PID
from std_msgs.msg import Int32


class Control():
    def __init__(self):
        rospy.init_node("controllerz")
        self.action = 0
        self.z_sp = 0.4
        self.no_tag_detection = Int32(0)
        self.pidz = PID(1, 0.1, 0.05, setpoint=self.z_sp)
        self.pidz.output_limits = (-0.2, 0.2)
        # define variables
        self.z = -0.7   # y estimate


        #rostopic echo /bluerov/ground_truth/state/pose/pose/position
        # to get the ground truth subscribe to /bluerov/ground_truth/state
        # and then set self.z = msg.pose.pose.position.z


        self.xy_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                       Odometry,
                                       self.on_sub,
                                       queue_size=1)


        self.xy_sub_marker = rospy.Subscriber("markerPosition",
                                       Point,
                                       self.on_sub_marker,
                                       queue_size=1)

        # Publisher
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

    def on_sub_marker(self, msg):
        if self.action == 0:
            self.z = msg.z        
            lat = self.pidz(self.z)
            self.vertical_thrust_pub.publish(Float64(lat))
            
    def on_sub(self, msg):
        if self.action != 0:
            self.z = msg.pose.pose.position.z
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
