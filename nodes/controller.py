#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID
import math
# from nav_msgs.msg import Odometry


# subscribe: x,y,z-est; x,y,z-setpoint; yaw-est; yaw-sp
# publish: thrust, lateral thrust, vertical thrust, yaw


class Control():
    def __init__(self):
        rospy.init_node("controller")

        # define variables
        self.x = 0.7  # x estimate
        self.y = 2.0  # y estimate
        self.z = -0.7  # depth estimate

        self.x_sp = 1.3
        self.y_sp = 2.5
        self.z_sp = -0.5

        self.pidx = PID(0.5, 0.1, 0.01, setpoint=self.x_sp)
        self.pidy = PID(0.5, 0.1, 0.01, setpoint=self.y_sp)
        self.pidz = PID(0.5, 0.1, 0.01, setpoint=self.z_sp)
        self.pidx.output_limits = (-1, 1)
        self.pidy.output_limits = (-1, 1)
        self.pidz.output_limits = (-1, 1)

        self.xy_sub = rospy.Subscriber("position",
                                       Point,
                                       self.on_getxy,
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

    # on every subscription, assign the topics current value to a variable
    # that has been defined in __init__


    def on_getxy(self, msg):
        if msg != None:
            self.x = msg.x
            self.y = msg.y
            self.z = msg.z


    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.xy_sub = rospy.Subscriber("position",
                                           Point,
                                           self.on_getxy,
                                           queue_size=1)
            # Publish forward thrust
            # thrust = self.pidx(float(self.x))
            # self.thrust_pub.publish(Float64(thrust))

            # Publish lateral thrust
            # lat = self.pidy(float(self.y))
            # self.lateral_thrust_pub.publish(Float64(lat))
            
            # Publish vertical thrust
            vert = self.pidz(float(self.z))
            self.vertical_thrust_pub.publish(Float64(vert))
            

def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
