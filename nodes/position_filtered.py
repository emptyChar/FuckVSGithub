#!/usr/bin/env python
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from std_msgs.msg import Float64
import numpy
#filters the position data


class Filter():

    def __init__(self):
        rospy.init_node("position_filtered")
        self.list = numpy.empty([50,3], dtype= Float64)
        self.cnt = 0
        self.firstLoop = 1
        self.x = any
        self.y = any
        self.z = any
        self.position_pub = rospy.Publisher("afiltered_position",
                                            Float64,
                                            queue_size=1)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pos_sub = rospy.Subscriber("noisy_position",
                                            Point,
                                            self.on_sub,
                                            queue_size=1)
            pos = self.filterData(self.x)
            self.position_pub.publish([self.x,self.y,self.z])
            rate.sleep()

    def on_sub(self, msg):
        self.x = Float64(msg.x)
        self.y = Float64(msg.y)
        self.z = Float64(msg.z)

    def filterData(self, data):        
        if self.cnt < 50:
            self.list[self.cnt] = data
            self.cnt += 1
        else:
            if self.firstLoop != 0:
                self.firstLoop = 0
            self.cnt = 0
            self.list[self.cnt] = data
            self.cnt += 1
        if self.firstLoop != 0:
            return data
        else:                     
            #sum/len
            numpy.sum(self.list, dtype= Float64)
            return data
        


def main():
    fil = Filter()
    fil.run()


if __name__ == "__main__":
    main()
