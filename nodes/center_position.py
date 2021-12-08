#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


class CenterPosition():
    def __init__(self):
        rospy.init_node("center_position")
        # define camera position relative to baseframe
        self.cx = 0.2
        self.cz = 0.1

        self.xc = 0
        self.yc = 0
        self.zc = 0
        # initial quaternion value when not rotated
        self.q0 = [0, 0, 0, 0]
        # vector used when subscribing
        self.qi = [0, 0, 0, 0]
        # the current rotation about z described in quat
        self.q = [0, 0, 0, 0]

        # put cx here in y var, since robot is facing in y direction initially
        self.pinit = [0, 0, self.cx, self.cz]
        self.p0 = [0, 0, 0, 0]

        # take later filtered "cam_position" as sub
        self.campos_sub = rospy.Subscriber("noisy_position",
                                           Point,
                                           self.on_sub,
                                           queue_size=1)

        self.imu_sub = rospy.Subscriber("mavros/imu/data",
                                        Imu,
                                        self.on_imu_sub,
                                        queue_size=1)

        self.position_pub = rospy.Publisher("position",
                                            Point,
                                            queue_size=1)

        self.angle_pub = rospy.Publisher("noisy_angle",
                                         Float64,
                                         queue_size=1)

    def on_sub(self, msg):
        self.xc = msg.x
        self.yc = msg.y
        self.zc = msg.z

    def on_imu_sub(self, msg):
        self.qi[0] = msg.orientation.w
        self.qi[1] = 0
        self.qi[2] = 0
        self.qi[3] = msg.orientation.z
        qq = math.sqrt(self.qi[0]*self.qi[0] + self.qi[3]*self.qi[3])
        a = self.qi[0]/qq
        b = self.qi[3]/qq
        self.q[0] = a
        self.q[1] = 0
        self.q[2] = 0
        self.q[3] = b

    def normalize(self, q):
        qq = math.sqrt(q[0]*q[0] + q[3]*q[3])
        a = q[0]/qq
        b = q[3]/qq
        return [a, 0, 0, b]

    def conj(self, q):
        a = q[0]
        b = -q[1]
        c = -q[2]
        d = -q[3]
        return [a, b, c, d]

    def mult(self, Q, R):

        a = Q[0]*R[0] - Q[1]*R[1] - Q[2]*R[2] - Q[3]*R[3]
        b = Q[0]*R[1] + Q[1]*R[0] + Q[2]*R[3] - Q[3]*R[2]
        c = Q[0]*R[2] - Q[1]*R[3] + Q[2]*R[0] + Q[3]*R[1]
        d = Q[0]*R[3] + Q[1]*R[2] - Q[2]*R[1] + Q[3]*R[0]
        return [a, b, c, d]

    def get_center(self, xc, yc, zc):
        # calculate the "true" point
        a = self.mult(self.q, self.p0)
        b = self.conj(self.q)
        ptrue = self.mult(a, b)
        zentrum = Point()
        zentrum.x = xc - ptrue[1]
        zentrum.y = yc - ptrue[2]
        zentrum.z = zc - ptrue[3]
        self.position_pub.publish(zentrum)

    def get_angle(self):
        a = self.conj(self.q0)
        # print("hi")
        b = self.mult(self.q, a)
        ang = math.acos(b[0]) + math.asin(b[3])
        deg = Float64()
        deg = ang*180/3.1415926536
        self.angle_pub.publish(deg)

    def q_computation(self):
        rate = rospy.Rate(50)
        for i in range(40):
            self.q0[0] += self.q[0]
            self.q0[1] += self.q[1]
            self.q0[2] += self.q[2]
            self.q0[3] += self.q[3]
            rate.sleep()
        
        self.q0 = self.normalize(self.q0)
        a = self.conj(self.q0)
        b = self.mult(a, self.pinit)
        self.p0 = self.mult(b, self.q0)

    def run(self):
        # calibration of IMU sensor before loop (zero is here)
        rate = rospy.Rate(50)
        while self.qi[0]*self.qi[0] < 0.1:
            self.imu_sub = rospy.Subscriber("mavros/imu/data",
                                            Imu,
                                            self.on_imu_sub,
                                            queue_size=1)
            rate.sleep()

        self.q_computation()
        while not rospy.is_shutdown():
            self.get_center(self.xc, self.yc, self.zc)
            self.get_angle()
            rospy.spin


def main():
    center = CenterPosition()
    center.run()


if __name__ == "__main__":
    main()
