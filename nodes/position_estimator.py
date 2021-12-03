#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point


class Estimator():
    def __init__(self):
        rospy.init_node("position_estimator")

        self.x = 0
        self.y = 0
        self.z = 0
        self.x_est = 1
        self.y_est = 1
        self.z_est = 1
        self.x_1 = 1
        self.y_1 = 1
        self.z_1 = 1
        self.err1 = 0.1
        self.err2 = 0.1
        self.err3 = 0.1
        self.meaerr = 0.314  # 0.314

        self.position_sub = rospy.Subscriber("position",
                                             Point,
                                             self.on_sub,
                                             queue_size=1)

        self.pos_est_pub = rospy.Publisher("position_estimate",
                                           Point,
                                           queue_size=1)

    def on_sub(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.position_sub = rospy.Subscriber("position",
                                                 Point,
                                                 self.on_sub,
                                                 queue_size=1)

            pointi = Point()
            a = self.x_estimation()
            pointi.x = a
            b = self.y_estimation()
            pointi.y = b
            c = self.z_estimation()
            pointi.z = c
            self.pos_est_pub.publish(pointi)
            rate.sleep()

    def x_estimation(self) -> float:
        xkest = self.x_1
        qk = 0.0004
        self.err1 += qk
        k = self.err1 / (self.err1 + self.meaerr)
        xk = xkest + k*(self.x - xkest)
        olderr = self.err1
        self.err1 = self.meaerr*olderr/(self.meaerr + olderr)
        self.x_1 = xk
        b = xk
        return b

    def y_estimation(self) -> float:
        ykest = self.y_1
        qk = 0.0004
        self.err2 += qk
        k = self.err2 / (self.err2 + self.meaerr)
        yk = ykest + k*(self.y - ykest)
        olderr = self.err2
        self.err2 = self.meaerr*olderr/(self.meaerr + olderr)
        self.y_1 = yk
        b = yk
        return b

    def z_estimation(self) -> float:
        zkest = self.z_1
        qk = 0.0004
        self.err3 += qk
        k = self.err3 / (self.err3 + self.meaerr)
        zk = zkest + k*(self.z - zkest)
        olderr = self.err3
        self.err3 = self.meaerr*olderr/(self.meaerr + olderr)
        self.z_1 = zk
        b = zk
        return b


def main():
    esti = Estimator()
    esti.run()


if __name__ == "__main__":
    main()
