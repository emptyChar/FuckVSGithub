#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import math
# from nav_msgs.msg import Odometry


# subscribe: x,y,z-est; x,y,z-setpoint; yaw-est; yaw-sp
# publish: thrust, lateral thrust, vertical thrust, yaw


class Control():
    def __init__(self):
        rospy.init_node("controller")

        # define variables
        self.x = 0.0  # x estimate
        self.y = 0.0  # y estimate
        self.z = 0.0  # depth estimate
        self.angle = 0.0  # angle estimate, difference from initial pos
        self.initial_angle = 90  # in degree
        self.dt = 0.05  # determined by rospy.Rate(50)

        self.old_x = 0
        self.old_y = 0
        self.old_z = 0

        self.x_int = 0
        self.y_int = 0
        self.z_int = 0

        self.x_sp = 0  # x setpoint
        self.y_sp = 0
        self.z_sp = -0.2
        self.angle_sp = 0

        self.p_gain = 0.2
        self.i_gain = 0.1
        self.d_gain = 2.5*1000

        # Subscriber: Estimates
        self.depth_sub = rospy.Subscriber("depth_estimate",
                                          Float64,
                                          self.on_depthest,
                                          queue_size=1)

        # For now, take ground truth x y coordinates instead

        self.xy_sub = rospy.Subscriber("position",
                                       Point,
                                       self.on_getxy,
                                       queue_size=1)

        self.angle_sub = rospy.Subscriber("noisy_angle",
                                          Float64,
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
        self.x = msg.x
        self.y = msg.y

    def on_angle_sub(self, msg):
        self.angle = msg.data

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

            self.xy_sub = rospy.Subscriber("position",
                                           Point,
                                           self.on_getxy,
                                           queue_size=1)

            self.angle_sub = rospy.Subscriber("noisy_angle",
                                              Float64,
                                              self.on_angle_sub,
                                              queue_size=1)

            # Publish forward thrust
            thrust = self.forward()
            self.thrust_pub.publish(thrust)

            # Publish lateral thrust
            lat = self.lateral()
            self.lateral_thrust_pub.publish(lat)

            # Publish vertical thrust
            vert = self.vertical()
            self.vertical_thrust_pub.publish(vert)

            # Publish yaw
            yaw = self.turn()
            self.yaw_pub.publish(yaw)
            rate.sleep()

            # make current values to next old values
            self.old_x = self.x
            self.old_y = self.y

    def deg2rad(self, deg):
        rad = deg * 3.1415 / 180
        return rad

    def forward(self):
        rad = self.deg2rad((self.angle + self.initial_angle))

        # proportional
        x_delta = (self.x_sp - self.x)
        thrust_p_x = x_delta * self.p_gain * math.cos(rad)
        y_delta = (self.y_sp - self.y)
        thrust_p_y = y_delta * self.p_gain * math.sin(rad)
        thrust_p = thrust_p_x + thrust_p_y
        # safety
        if thrust_p > 0.5:
            thrust_p = 0.5
        elif thrust_p < -0.5:
            thrust_p = -0.5

        # integral
        # safety first
        if self.x_int > 1:
            self.x_int = 0.99
        elif self.x_int < -1:
            self.x_int = -0.99
        else:
            self.x_int += x_delta * self.dt

        thrust_i_x = self.x_int * self.i_gain * math.cos(rad)

        if self.y_int > 1:
            self.y_int = 0.99
        elif self.y_int < -1:
            self.y_int = -0.99
        else:
            self.y_int += y_delta * self.dt
        thrust_i_y = self.y_int * self.i_gain * math.sin(rad)
        thrust_i = thrust_i_x + thrust_i_y

        # derivative
        dxdt = (self.old_x - self.x) / self.dt
        thrust_d_x = dxdt * self.d_gain * math.cos(rad)
        dydt = (self.old_y - self.y) / self.dt
        thrust_d_y = dydt * self.d_gain * math.sin(rad)
        thrust_d = thrust_d_x + thrust_d_y
        self.old_x = self.x
        self.old_y = self.y
        # make current values to next old values
        # (done in run)

        out = Float64()
        out.data = thrust_p + thrust_i + thrust_d
        # out.data =

        if out.data > 1:
            out.data = 1
        elif out.data < -1:
            out.data = -1
        return out

    def lateral(self):

        rad = self.deg2rad((self.angle + self.initial_angle))
        x_delta = (self.x_sp - self.x)
        thrust_p_x = x_delta * self.p_gain * (-math.sin(rad))
        y_delta = (self.y_sp - self.y)
        thrust_p_y = y_delta * self.p_gain * math.cos(rad)
        thrust_p = thrust_p_x + thrust_p_y
        # safety
        if thrust_p > 0.5:
            thrust_p = 0.5
        elif thrust_p < -0.5:
            thrust_p = -0.5

        # integral
        # care, in forward function the integral has already added up
        thrust_i_x = self.x_int * self.i_gain * (-math.sin(rad))
        thrust_i_y = self.y_int * self.i_gain * math.cos(rad)
        thrust_i = thrust_i_x + thrust_i_y

        # derivative
        dxdt = (self.old_x - self.x) / self.dt
        thrust_d_x = dxdt * self.d_gain * (-math.sin(rad))
        dydt = (self.old_y - self.y) / self.dt
        thrust_d_y = dydt * self.d_gain * math.cos(rad)
        thrust_d = thrust_d_x + thrust_d_y

        out = Float64()
        out.data = thrust_p + thrust_i + thrust_d
        
        if out.data > 1:
            out.data = 1
        elif out.data < -1:
            out.data = -1

        out.data = 0
        return out

    def vertical(self):
        depth_delta = (self.z_sp - self.z)
        thrust_p = depth_delta * self.p_gain
        out = Float64()
        out.data = thrust_p * 2
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
