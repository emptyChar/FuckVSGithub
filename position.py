#!/usr/bin/env python
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point

# get the distance to the 4 known markers and
# calculates the position of the point


class Trilateration():
    def __init__(self):
        rospy.init_node("localization")
        self.d1 = 0
        self.d2 = 0
        self.d3 = 0
        self.d4 = 0

        self.distance_sub = rospy.Subscriber("ranges",
                                             RangeMeasurementArray,
                                             self.on_sub,
                                             queue_size=1)

        self.position_pub = rospy.Publisher("noisy_position",
                                            Point,
                                            queue_size=1)

    def on_sub(self, msg):
        try:
            self.d1 = msg.measurements[0].range
        except Exception:
            self.d1 = self.d1
        try:
            self.d2 = msg.measurements[1].range
        except Exception:
            self.d2 = self.d2
        try:
            self.d3 = msg.measurements[2].range
        except Exception:
            self.d3 = self.d3
        try:
            self.d4 = msg.measurements[3].range
        except Exception:
            self.d4 = self.d4

    def trilaterate3D(self, r1, r2, r3, r4):
        # known points in 3D, change this based on measurements
        p1 = np.array([0.5, 3.36, -0.5])
        p2 = np.array([1.1, 3.35, -0.5])
        p3 = np.array([0.5, 3.35, -0.9])
        p4 = np.array([1.1, 3.35, -0.9])
        e_x = (p2-p1)/np.linalg.norm(p2-p1)
        i = np.dot(e_x, (p3-p1))
        e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2-p1)
        j = np.dot(e_y, (p3-p1))
        x = ((r1**2)-(r2**2)+(d**2))/(2*d)
        y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
        z1 = np.sqrt(r1**2-x**2-y**2)
        z2 = np.sqrt(r1**2-x**2-y**2)*(-1)
        ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
        ans2 = p1+(x*e_x)+(y*e_y)+(z2*e_z)
        dist1 = np.linalg.norm(p4-ans1)
        dist2 = np.linalg.norm(p4-ans2)
        if np.abs(r4-dist1) < np.abs(r4-dist2):
            return ans1
        else:
            return ans2

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.distance_sub = rospy.Subscriber("ranges",
                                                 RangeMeasurementArray,
                                                 self.on_sub,
                                                 queue_size=1)
            posi = self.trilaterate3D(self.d1, self.d2, self.d3, self.d4)
            pos = Point()
            pos.x = posi[0]
            if posi[1] < 3.35:
                pos.y = posi[1]
            else:
                pos.y = 6.7 - posi[1]
            pos.z = posi[2]
            self.position_pub.publish(pos)
            rate.sleep()


def main():
    tri = Trilateration()
    tri.run()


if __name__ == "__main__":
    main()
