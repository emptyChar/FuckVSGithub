#!/usr/bin/env python
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
from rospy.topics import Publisher

#filters the position data


class Filter():    

    def __init__(self):
        rospy.init_node("position_filtered")        

        self.position_pub = rospy.Publisher("afiltered_position",
                                            Point,
                                            queue_size=1)
              

    def run(self):
        #for the filter which doesnt work yet
        self.list = [any] * 50
        self.cnt = 0
        self.firstLoop = 1
        rate = rospy.Rate(50)        
        while not rospy.is_shutdown():
            self.pos_sub = rospy.Subscriber("position",
                                            Point,                                                                                        
                                            queue_size=1)
            pos = self.filterData(self.pos_sub)
            self.position_pub.publish(pos)

            
    def filterData(self, data):     
        #moving average filter, but doesnt wor yet for some reason   
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
            return sum(list) / len(list)


def main():
    fil = Filter()
    fil.run()    


if __name__ == "__main__":
    main()
