#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

#todo rewrie with rospy.spin






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

    def run(self):
        # rate = rospy.Rate(60000)
        # rate.sleep()
        # calibration of IMU sensor before loop (zero is here)
               
        for i in range(40):           
            self.q0[0] += self.q[0]
            self.q0[1] += self.q[1]
            self.q0[2] += self.q[2]
            self.q0[3] += self.q[3]
            
        self.q0 = self.normalize(self.q0)
        a = self.conj(self.q0)
        b = self.mult(a, self.pinit)
        self.p0 = self.mult(b, self.q0)
        while not rospy.is_shutdown():
            self.campos_sub = rospy.Subscriber("noisy_position",
                                               Point,
                                               self.on_sub,
                                               queue_size=1)

            self.imu_sub = rospy.Subscriber("mavros/imu/data",
                                            Imu,
                                            self.on_imu_sub,
                                            queue_size=1)

            zenter = self.get_center(self.xc, self.yc, self.zc)
            self.position_pub.publish(zenter)
            angle = self.get_angle()
            self.angle_pub.publish(angle)
            rate.sleep()

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
        # P = [0, 0, 0, 0]
        # for i in range(4):
        #     for j in range(4):
        #         P[abs(i-j) if i*j == 0 or i == j else 6-i-j] \
        #             += Q[i] * R[j] * (1 if i*j == 0 or i % 3+1 == j else -1)
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
        return zentrum
     
    def euler_from_quaternion(self):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x, y, z, w = [self.q0, ]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_angle(self):
        a = self.conj(self.q0)
        b = self.mult(self.q, a)
        ang = math.acos(b[0]) + math.asin(b[3])
        deg = Float64()
        deg = ang*360/(2*3.1415926536)
        return deg


def main():
    center = CenterPosition()
    center.run()


if __name__ == "__main__":
    main()
