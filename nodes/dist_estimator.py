#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from bluerov_sim.msg import RangeMeasurementArray


class Estimator():
    def __init__(self):
        rospy.init_node("distance_estimator")

        self.d1 = 0
        self.d2 = 0
        self.d3 = 0
        self.d4 = 0
        self.d1est = 1
        self.d2est = 1
        self.d3est = 1
        self.d4est = 1
        self.d1_1 = 1
        self.d2_1 = 1
        self.d3_1 = 1
        self.d4_1 = 1
        self.err1 = 0.1
        self.err2 = 0.1
        self.err3 = 0.1
        self.err4 = 0.1
        self.meaerr = 0.314  # 0.314

        self.array = RangeMeasurementArray()

        self.distance_sub = rospy.Subscriber("ranges",
                                             RangeMeasurementArray,
                                             self.on_sub,
                                             queue_size=1)

        self.dist_est_pub = rospy.Publisher("ranges_estimate",
                                            RangeMeasurementArray,
                                            queue_size=4)

        self.debug_pub = rospy.Publisher("debug_est",
                                         Float64,
                                         queue_size=1)

    def on_sub(self, msg):
        try:
            self.d1 = msg.measurements[0].range
            try:
                self.d2 = msg.measurements[1].range
                try:
                    self.d3 = msg.measurements[2].range
                    try:
                        self.d4 = msg.measurements[3].range
                        self.array.header = msg.header
                        self.array.measurements = msg.measurements
                        hh1 = self.d1_estimation()
                        hh1 = round(hh1, 4)
                        self.array.measurements[0].range = hh1
                        hh2 = self.d2_estimation()
                        hh2 = round(hh2, 4)
                        self.array.measurements[1].range = hh2
                        hh3 = self.d3_estimation()
                        hh3 = round(hh3, 4)
                        self.array.measurements[2].range = hh3
                        hh4 = self.d4_estimation()
                        hh4 = round(hh4, 4)
                        self.array.measurements[3].range = hh4
                    except Exception:
                        self.d4 = self.d4
                except Exception:
                    self.d3 = self.d3
            except Exception:
                self.d2 = self.d2
        except Exception:
            self.d1 = self.d1

    def run(self):
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            self.distance_sub = rospy.Subscriber("ranges",
                                                 RangeMeasurementArray,
                                                 self.on_sub,
                                                 queue_size=1)

            a = self.d1_estimation()
            a = round(a, 4)
            c = Float64()
            c.data = a
            self.debug_pub.publish(c)
            pubarray = self.array
            self.dist_est_pub.publish(pubarray)
            rate.sleep()

    def d1_estimation(self) -> float:
        # 1: xk = Axk1 + Bu + w -> xk = xk1 no velocity/accel.
        # New prediction is old value (assume nothing changes)
        dkest = self.d1_1
        # 2: Pk = A*P^k*A' + Qk Predicted Process covariance
        # play arpund with Qk, it says how much we assume state to change.
        # assume max speed 0.4m/s = 0.02m/0.05s
        # assume slower change if it should be more accurate
        qk = 0.0004
        self.err1 += qk
        # 3: K = Err_est / (Err_est + Err_mea) Kalman gain
        k = self.err1 / (self.err1 + self.meaerr)
        # 4: xk = xk_est + K*(mea-xk_est)
        dk = dkest + k*(self.d1 - dkest)
        # 5: Update estimate error
        olderr = self.err1
        self.err1 = self.meaerr*olderr/(self.meaerr + olderr)
        self.d1_1 = dk
        b = dk
        return b

    def d2_estimation(self) -> float:
        dkest = self.d2_1
        qk = 0.0004
        self.err2 += qk
        k = self.err2 / (self.err2 + self.meaerr)
        dk = dkest + k*(self.d2 - dkest)
        olderr = self.err2
        self.err2 = self.meaerr*olderr/(self.meaerr + olderr)
        self.d2_1 = dk
        b = dk
        return b

    def d3_estimation(self) -> float:
        dkest = self.d3_1
        qk = 0.0004
        self.err3 += qk
        k = self.err3 / (self.err3 + self.meaerr)
        dk = dkest + k*(self.d3 - dkest)
        olderr = self.err3
        self.err3 = self.meaerr*olderr/(self.meaerr + olderr)
        self.d3_1 = dk
        b = dk
        return b

    def d4_estimation(self) -> float:
        dkest = self.d4_1
        qk = 0.0004
        self.err4 += qk
        k = self.err4 / (self.err4 + self.meaerr)
        dk = dkest + k*(self.d4 - dkest)
        olderr = self.err4
        self.err4 = self.meaerr*olderr/(self.meaerr + olderr)
        self.d4_1 = dk
        b = dk
        return b


def main():
    esti = Estimator()
    esti.run()


if __name__ == "__main__":
    main()
