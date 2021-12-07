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
        #todo fix mit id
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
        posi = self.trilaterate3D_1(self.d1, self.d2, self.d3)
        pos = Point()
        pos.x = posi[0]
        if posi[1] < 3.35:
            pos.y = posi[1]
        else:
            pos.y = 6.7 - posi[1]
        pos.z = posi[2]
        self.position_pub.publish(pos)

    def trilaterate3D_1(self, r1, r2, r3):
        p1 = np.array([0.5, 3.36, -0.5])
        p2 = np.array([1.1, 3.35, -0.5])
        p3 = np.array([0.5, 3.35, -0.9])
        e_x = (p2-p1)/np.linalg.norm(p2-p1)
        i = np.dot(e_x, (p3-p1))
        e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
        e_z = np.cross(e_x, e_y)
        d = np.linalg.norm(p2-p1)
        j = np.dot(e_y, (p3-p1))
        x = ((r1**2)-(r2**2)+(d**2))/(2*d)
        y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
        if isinstance(r1, float) != True or r1 == None:
            r1 = 0
        if isinstance(x, float) != True or x == None:
            x = 0
        if isinstance(y, float) != True or y == None:
            y = 0
        
        z1 = np.sqrt(r1**2-x**2-y**2)
        # z2 = np.sqrt(r1**2-x**2-y**2)*(-1)
        ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
        # ans2 = p1+(x*e_x)+(y*e_y)+(z2*e_z)
        return ans1
        # dist1 = np.linalg.norm(p4-ans1)
        # dist2 = np.linalg.norm(p4-ans2)

    # def trilaterate3D_2(self, r1, r2, r3):
    #     p1 = np.array([0.5, 3.36, -0.5])
    #     p2 = np.array([1.1, 3.35, -0.5])
    #     p3 = np.array([1.1, 3.35, -0.9])
    #     e_x = (p2-p1)/np.linalg.norm(p2-p1)
    #     i = np.dot(e_x, (p3-p1))
    #     e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    #     e_z = np.cross(e_x, e_y)
    #     d = np.linalg.norm(p2-p1)
    #     j = np.dot(e_y, (p3-p1))
    #     x = ((r1**2)-(r2**2)+(d**2))/(2*d)
    #     y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    #     if isinstance(r1, float) != True or r1 == None:
    #         r1 = 0
    #     if isinstance(x, float) != True or x == None:
    #         x = 0
    #     if isinstance(y, float) != True or y == None:
    #         y = 0
    #     z1 = np.sqrt(r1**2-x**2-y**2)
    #     ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
    #     return ans1

    # def trilaterate3D_3(self, r1, r2, r3):self.trilaterate3D_1(self.d1, self.d2, self.d3)
    #     p1 = np.array([0.5, 3.36, -0.5])
    #     p2 = np.array([0.5, 3.35, -0.9])
    #     p3 = np.array([1.1, 3.35, -0.9])
    #     e_x = (p2-p1)/np.linalg.norm(p2-p1)
    #     i = np.dot(e_x, (p3-p1))
    #     e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    #     e_z = np.cross(e_x, e_y)
    #     d = np.linalg.norm(p2-p1)
    #     j = np.dot(e_y, (p3-p1))
    #     x = ((r1**2)-(r2**2)+(d**2))/(2*d)
    #     y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    #     if isinstance(r1, float) != True or r1 == None:
    #         r1 = 0
    #     if isinstance(x, float) != True or x == None:
    #         x = 0
    #     if isinstance(y, float) != True or y == None:
    #         y = 0
    #     z1 = np.sqrt(r1**2-x**2-y**2)
    #     ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
    #     return ans1

    # def trilaterate3D_4(self, r1, r2, r3):
    #     p1 = np.array([1.1, 3.35, -0.5])
    #     p2 = np.array([0.5, 3.35, -0.9])
    #     p3 = np.array([1.1, 3.35, -0.9])
    #     e_x = (p2-p1)/np.linalg.norm(p2-p1)
    #     i = np.dot(e_x, (p3-p1))
    #     e_y = (p3-p1-(i*e_x))/(np.linalg.norm(p3-p1-(i*e_x)))
    #     e_z = np.cross(e_x, e_y)
    #     d = np.linalg.norm(p2-p1)
    #     j = np.dot(e_y, (p3-p1))
    #     x = ((r1**2)-(r2**2)+(d**2))/(2*d)
    #     y = (((r1**2)-(r3**2)+(i**2)+(j**2))/(2*j))-((i/j)*(x))
    #     if isinstance(r1, float) != True or r1 == None:
    #         r1 = 0
    #     if isinstance(x, float) != True or x == None:
    #         x = 0
    #     if isinstance(y, float) != True or y == None:
    #         y = 0
    #     z1 = np.sqrt(r1**2-x**2-y**2)
    #     ans1 = p1+(x*e_x)+(y*e_y)+(z1*e_z)
    #     return ans1

    def run(self):        
        while not rospy.is_shutdown():
            rospy.spin            
            # a2 = self.trilaterate3D_1(self.d1, self.d2, self.d3)
            # b2 = self.trilaterate3D_2(self.d1, self.d2, self.d4)
            # c2 = self.trilaterate3D_3(self.d1, self.d3, self.d4)
            # d2 = self.trilaterate3D_4(self.d2, self.d3, self.d4)

            # sum_list_1 = [a + b for a, b in zip(a2, b2)]
            # sum_list_2 = [a + b for a, b in zip(c2, d2)]
            # sum_list_total = [a + b for a, b in zip(sum_list_1, sum_list_2)]

            # posi = [x / 4 for x in sum_list_total]
            
            
            


def main():
    tri = Trilateration()
    tri.run()


if __name__ == "__main__":
    main()