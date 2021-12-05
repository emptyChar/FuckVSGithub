#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose


class Setpoint():
    def __init__(self):
        rospy.init_node("pose_setpoint")
        self.x = 0
        self.y = 0
        self.z = -0.5
        self.angle = 0

        self.pose_pub = rospy.Publisher("pose_setpoint",
                                        Pose,
                                        queue_size=1)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            pos = self.get_setpoint()
            self.pose_pub.publish(pos)
            rate.sleep()

    def get_setpoint(self) -> Pose:
        posesp = Pose()
        # x in range 0 - 2
        posesp.position.x = 0.7
        # y in range 0 - 3.35
        posesp.position.y = 2.5
        # z in range 0- -0.7
        posesp.position.z = -0.5

        posesp.orientation.w = 1  # math.cos(0.5*self.angle)
        posesp.orientation.x = 0
        posesp.orientation.y = 0
        posesp.orientation.z = 0  # math.sin(0.5*self.angle)

        return posesp


def main():
    node = Setpoint()
    node.run()


if __name__ == "__main__":
    main()
