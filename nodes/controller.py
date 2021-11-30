#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry


# subscribe: x,y,z-est; x,y,z-setpoint; yaw-est; yaw-sp
# publish: thrust, lateral thrust, vertical thrust, yaw


class Control():
    def __init__(self):
        rospy.init_node("controller")

        # define variables
        self.x = 0.0  # x estimate
        self.y = 0.0  # y estimate
        self.z = 0.0  # depth estimate
        self.angle = 0.0  # angle estimate

        self.x_sp = 0  # x setpoint
        self.y_sp = 0
        self.z_sp = -0.2
        self.angle_sp = 0

        self.p_gain = 1
        self.i_gain = 0
        self.d_gain = 0

        # Subscriber: Estimates
        self.depth_sub = rospy.Subscriber("depth_estimate",
                                          Float64,
                                          self.on_depthest,
                                          queue_size=1)

        # For now, take ground truth x y coordinates instead

        self.xy_sub = rospy.Subscriber("ground_truth/state",
                                       Odometry,
                                       self.on_getxy,
                                       queue_size=1)

        # Subscriber: Setpoints
        self.pose_setpoint_sub = rospy.Subscriber("pose_setpoint",
                                                  Pose,
                                                  self.on_posesub,
                                                  queue_size=1)

        # Publisher
        self.thrust_pub = rospy.Publisher("thrust",
                                          Float64,
                                          queue_size=1)

        self.lateral_thrust_pub = rospy.Publisher("lateral_thrust",
                                                  Float64,
                                                  queue_size=1)

        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                   Float64,
                                                   queue_size=1)

        self.yaw_pub = rospy.Publisher("yaw",
                                       Float64,
                                       queue_size=1)

    # on every subscription, assign the topics current value to a variable
    # that has been defined in __init__
    def on_posesub(self, msg):
        self.x_sp = msg.position.x
        self.y_sp = msg.position.y
        self.z_sp = msg.position.z

    def on_depthest(self, msg):
        self.z = msg.data

    def on_getxy(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.pose_setpoint_sub = rospy.Subscriber("pose_setpoint",
                                                      Pose,
                                                      self.on_posesub,
                                                      queue_size=1)

            self.depth_sub = rospy.Subscriber("depth_estimate",
                                              Float64,
                                              self.on_depthest,
                                              queue_size=1)

            thrust = self.forward()
            self.thrust_pub.publish(thrust)
            lat = self.lateral()
            self.lateral_thrust_pub.publish(lat)

            vert = self.vertical()
            self.vertical_thrust_pub.publish(vert)
            yaw = self.turn()
            self.yaw_pub.publish(yaw)
            rate.sleep()

    def forward(self):
        x_delta = (self.x_sp - self.x)
        thrust_p = x_delta * self.p_gain
        out = Float64()
        out.data = thrust_p
        return out

    def lateral(self):
        y_delta = (self.y_sp - self.y)
        thrust_p = y_delta * self.p_gain
        out = Float64()
        out.data = thrust_p
        return out

    def vertical(self):
        depth_delta = (self.z_sp - self.z)
        thrust_p = depth_delta * self.p_gain
        out = Float64()
        out.data = thrust_p
        return out

    def turn(self):
        out = Float64()
        out.data = 0.0
        return out


def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
