#!/usr/bin/env python
import rospy
import numpy as np
from bluerov_sim.msg import RangeMeasurementArray
from geometry_msgs.msg import Point
import math
# get the distance to the 4 known markers and
# calculates the position of the point



#   todo what happens if only two markers are seen
#   what happens if not enough markers are seen?
#   fix for real world


class Trilateration():

    def __init__(self):
        rospy.init_node("localization")
        
        self.d = [0.0]*4
        self.x = 0.7
        self.y = 2
        self.z = -0.7
        self.distance_sub = rospy.Subscriber("ranges",
                                             RangeMeasurementArray,
                                             self.on_sub_position,
                                             queue_size=1)

        self.position_pub = rospy.Publisher("noisy_position",
                                            Point,
                                            queue_size=1)

       

    def on_sub_position(self, msg):
        f = 0.04
        for u in [0,1,2,3]:            
            try:
                x = msg.measurements[u].id - 1
                if (self.d[x] - f) < msg.measurements[x].range < (self.d[x] + f):
                    self.d[x] = msg.measurements[x].range
                else:
                    self.d[x] += (msg.measurements[x].range - self.d[x]) / 4
            except Exception:
                self.d[x] = self.d[x]
        
        p1 = np.array([0.5, 3.36, -0.5])
        p2 = np.array([1.1, 3.35, -0.5])
        p3 = np.array([0.5, 3.35, -0.9])
        p4 = np.array([1.1, 3.35, -0.9])        

        a2 = self.trilaterate3D(p1, p2, p3, self.d[0], self.d[1], self.d[2])
        b2 = self.trilaterate3D(p1, p2, p4, self.d[0], self.d[1], self.d[3])
        c2 = self.trilaterate3D(p1, p3, p4, self.d[0], self.d[2], self.d[3])
        d2 = self.trilaterate3D(p2, p3, p4, self.d[1], self.d[2], self.d[3])

        sum_list_1 = [a + b for a, b in zip(a2, b2)]
        sum_list_2 = [a + b for a, b in zip(c2, d2)]

        a3 = [x / 2 for x in sum_list_1]
        b3 = [x / 2 for x in sum_list_2]

        if a3[1] < 3.35:
            pass
        else:
            a3[1] = 6.7 - a3[1]

        if b3[1] < 3.35:
            pass
        else:
            b3[1] = 6.7 - b3[1]

        sum_list_3 = [a + b for a, b in zip(a3, b3)]
        
        posi = [x / 2 for x in sum_list_3]        
        if math.isnan(posi[0]) == False:
            self.x = posi[0]
        else: 
            self.x = self.x
        if math.isnan(posi[1]) == False:
            if posi[1] < 3.35:
                self.y = posi[1]
            else:
                self.y = 6.7 - posi[1]
        else:
            self.y = self.y 
        if math.isnan(posi[2]) == False:
            self.z = posi[2]
        else:
            self.z = self.z
        pos = Point(self.x, self.y, self.z)
        self.position_pub.publish(pos)

    def trilaterate3D(self,p1, p2, p3, r1, r2, r3):        
        e_x = (p2-p1)/np.linalg.norm(p2-p1)
        i = np.dot(e_x, (p3-p1))
        e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2-p1)
        j = np.dot(e_y, (p3-p1))
        x = ((r1**2)-(r2**2)+(d**2))/(2*d)
        y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
        z1 = np.sqrt(r1**2-x**2-y**2)
        # z2 = np.sqrt(r1**2-x**2-y**2)*(-1)
        ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
        # ans2 = p1+(x*e_x)+(y*e_y)+(z2*e_z)
        return ans1
        # dist1 = np.linalg.norm(p4-ans1)
        # dist2 = np.linalg.norm(p4-ans2)

    def run(self):            
            while not rospy.is_shutdown():
                rospy.spin    

def main():
    tri = Trilateration()
    tri.run()


if __name__ == "__main__":
    main()