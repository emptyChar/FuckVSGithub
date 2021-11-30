#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Point


class Estimator():
    def __init__(self):
        rospy.init_node("depth_estimator")

        self.mea = 0.0
        self.xk1 = 0
        self.err = 0.1
        self.meaerr = 0.1

        self.x_mea = 0
        self.y_mea = 0
        self.xy_meaerr = 0.314

        self.angle_sub = rospy.Subscriber("noisy_angle",
                                          Float64,
                                          queue_size=1)

        self.posi_sub = rospy.Subscriber("position",
                                         Point,
                                         self.on_pos_sub,
                                         queue_size=1)

        self.posi_est_pub = rospy.Publisher("position_estimate",
                                            Float64,
                                            queue_size=1)

        self.posi_filtered = rospy.Publisher("position_filtered",
                                            Float64,
                                            queue_size=1)
    def on_depth(self, msg):
        self.mea = msg.data

    def on_pos_sub(self, msg):
        self.mea = 0

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            pos = self.filterData(self.posi_sub)
            self.posi_filtered.publish(pos)
            a = self.estimation()
            self.my_pub.publish(a)
            rate.sleep()

    def estimation(self):
        self.depth_sub = rospy.Subscriber("depth",
                                          Float64,
                                          self.on_depth,
                                          queue_size=1)

        # 1: xk = Axk1 + Bu + w -> xk = xk1 no velocity/accel.
        # New prediction is old value (assume nothing changes)
        xkest = self.xk1

        # 2: Pk = A*P^k*A' + Qk Predicted Process covariance
        # play arpund with Qk, it says how much we assume state to change.
        # assume max speed 0.4m/s = 0.02m/0.05s
        # assume slower change if it should be more accurate
        qk = 0.0004
        self.err += qk

        # 3: K = Err_est / (Err_est + Err_mea) Kalman gain
        k = self.err / (self.err + self.meaerr)

        # 4: xk = xk_est + K*(mea-xk_est)
        xk = xkest + k*(self.mea - xkest)

        # 5: Update estimate error
        olderr = self.err
        self.err = self.meaerr*olderr/(self.meaerr + olderr)
        self.xk1 = xk
        b = Float64()
        b.data = xk
        return b

    

def main():
    esti = Estimator()
    esti.run()


if __name__ == "__main__":
    main()
