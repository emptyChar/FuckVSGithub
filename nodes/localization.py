#!/usr/bin/env python
import rospy
import numpy as np

# subscribe: d1, d2, d3, d4, ?z-est?
# publish: estimate of x, y, z

class Position():
    def __init__():
        rospy.init_node("position_calculator")

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            a = self.estimation()
            self.my_pub.publish(a)
            rate.sleep()

    def estimation(self):
        # april tag positions
        a1 = 0
        a2 = 0
        a3 = 0
        b1 = 0
        b2 = 0
        b3 = 0
        c1 = 0
        c2 = 0
        c3 = 0
        d1 = 0
        d2 = 0
        d3 = 0
        d4 = 0
        dist1 = 1
        dist2 = 1
        dist3 = 1
        dist4 = 1

        A1 = -2*a1
        A2 = -2*a2
        A3 = -2*a3
        A4 = dist1*dist1 - a1*a1 - a2*a2 - a3*a3

        B1 = -2*b1
        B2 = -2*b2
        B3 = -2*b3
        B4 = dist2*dist2 - b1*b1 - b2*b2 - b3*b3

        C1 = -2*c1
        C2 = -2*c2
        C3 = -2*c3
        C4 = dist3*dist3 - c1*c1 - c2*c2 - c3*c3

        D1 = -2*d1
        D2 = -2*d2 
        D3 = -2*d3
        D4 = dist4*dist4 - d1*d1 - d2*d2 - d3*d3

        m11 = B1 - A1
        m12 = B2 - A2
        m13 = B3 - A3
        m21 = C1 - A1
        m22 = C2 - A2
        m23 = C3 - A3
        m31 = D1 - A1
        m32 = D2 - A2
        m33 = D3 - A3

        k1 = B4 - A4
        k2 = C4 - A4
        k3 = D4 - A4

        M = np.array([[m11, m12, m13], [m21, m22, m23],[m31, m32, m33]])
        M_inv = np.linalg.inv(M)
        k = np.array()
        p = M_inv.dot(k)
        




def main():
    esti = DepthEstimator()
    esti.run()


if __name__ == "__main__":
    main()
