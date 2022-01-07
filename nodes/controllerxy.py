#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID

from nav_msgs.msg import Odometry
from pyquaternion import Quaternion
# from nav_msgs.msg import Odometry


# subscribe: x,y,z-est; x,y,z-setpoint; yaw-est; yaw-sp
# publish: thrust, lateral thrust, vertical thrust, yaw


class Control():
    def __init__(self):
        rospy.init_node("controllerxy")        
        self.x_sp = 0.4
        self.y_sp = 2.0        
        
        self.pidx = PID(1, 0.1, 0.05, setpoint=self.x_sp)
        self.pidx.output_limits = (-0.1, 0.1)
        self.pidy = PID(1, 0.1, 0.05, setpoint=self.y_sp)       
        self.pidy.output_limits = (-0.1, 0.1)        
        # define variables    
        self.x = 0.7    
        self.y = 2.0  # y estimate

        #rostopic echo /bluerov/ground_truth/state/pose/pose/position
        
        self.xy_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                       Odometry,
                                       self.on_sub,
                                       queue_size=1)

        self.setpoint_sub = rospy.Subscriber("pose_setpoint",
                                            Point,
                                            self.on_sub_setpoint,
                                            queue_size=1)

        # Publisher
        self.thrust_pub = rospy.Publisher("lateral_thrust",
                                          Float64,
                                          queue_size=1)

        self.lateral_thrust_pub = rospy.Publisher("thrust",
                                                  Float64,
                                                  queue_size=1)
    

    def on_sub_setpoint(self, msg): 
        self.x_sp = msg.x 
        self.pidx.setpoint = self.x_sp
        self.y_sp = msg.y - 0.4
        self.pidy.setpoint = self.y_sp

    def on_sub(self, msg):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        # Publish lateral 
        thrustK = self.pidx(self.x)
        latK = self.pidy(self.y)
        q = Quaternion(msg.pose.pose.orientation.x, 0, 0, msg.pose.pose.orientation.w)
        self.yaw = q.radians
        thrust = math.cos(self.yaw) * latK + math.sin(self.yaw) * thrustK
        lat    =-math.sin(self.yaw) * latK + math.cos(self.yaw) * thrustK
        self.thrust_pub.publish(Float64(thrust))
        self.lateral_thrust_pub.publish(Float64(lat))

    def run(self):            
        while not rospy.is_shutdown():
            rospy.spin
           

def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
