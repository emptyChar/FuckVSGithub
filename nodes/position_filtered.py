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
        self.lol = 200
        self.list_x = [0] * self.lol
        self.list_y = [0] * self.lol
        self.list_z = [0] * self.lol

        self.cnt_x = 0
        self.cnt_y = 0
        self.cnt_z = 0

        self.firstLoop_x = 1
        self.firstLoop_y = 1
        self.firstLoop_z = 1

        self.x = any
        self.y = any
        self.z = any
        self.position_pub = rospy.Publisher("afiltered_position",
                                            Point,
                                            queue_size=1)

    def run(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pos_sub = rospy.Subscriber("noisy_position",
                                            Point,
                                            self.on_sub,
                                            queue_size=1)
            pos_x = self.filterData_x(self.x)
            pos_y = self.filterData_y(self.y)
            pos_z = self.filterData_z(self.z)
            pointPosition = Point()
            pointPosition.x = pos_x.data
            pointPosition.y = pos_y.data
            pointPosition.z = pos_z.data
            self.position_pub.publish(pointPosition)
            rate.sleep()

    def on_sub(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z


    
    def filterData_x(self, data):
        out = Float64()
        out.data = data      
        if self.cnt_x < self.lol:
            self.list_x[self.cnt_x] = data
            self.cnt_x += 1
        else:
            if self.firstLoop_x != 0:
                self.firstLoop_x = 0
            self.cnt_x = 0
            self.list_x[self.cnt_x] = data
            self.cnt_x += 1
        if self.firstLoop_x != 0:
            return out
        else:                     
            #sum/len
            out.data = sum(self.list_x) / len(self.list_x)
            return out

    def filterData_y(self, data):
        out = Float64()
        out.data = data      
        if self.cnt_y < self.lol:
            self.list_y[self.cnt_y] = data
            self.cnt_y += 1
        else:
            if self.firstLoop_y != 0:
                self.firstLoop_y = 0
            self.cnt_y = 0
            self.list_y[self.cnt_y] = data
            self.cnt_y += 1
        if self.firstLoop_y != 0:
            return out
        else:                     
            #sum/len
            out.data = sum(self.list_y) / len(self.list_y)
            return out

    def filterData_z(self, data):
        out = Float64()
        out.data = data      
        if self.cnt_z < self.lol:
            self.list_z[self.cnt_z] = data
            self.cnt_z += 1
        else:
            if self.firstLoop_z != 0:
                self.firstLoop_z = 0
            self.cnt_z = 0
            self.list_z[self.cnt_z] = data
            self.cnt_z += 1
        if self.firstLoop_z != 0:
            return out
        else:                     
            #sum/len
            out.data = sum(self.list_z) / len(self.list_z)
            return out
        


def main():
    fil = Filter()
    fil.run()


if __name__ == "__main__":
    main()