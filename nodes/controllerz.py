#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID

class Control():
    def __init__(self):
        rospy.init_node("controllerz")        
        self.z_sp = -0.5   
        self.pidz = PID(0.1, 0.01, 0.005, setpoint=self.z_sp)       
        self.pidz.output_limits = (-1, 1)        
        # define variables        
        self.z = -0.7 # y estimate

        self.xy_sub = rospy.Subscriber("noisy_position",
                                        Point,
                                        self.on_sub,
                                        queue_size=1)

        self.setpoint_sub = rospy.Subscriber("pose_setpoint",
                                            Point,
                                            self.on_sub_setpointz,
                                            queue_size=1)

        # Publisher
        self.vertical_thrust_pub = rospy.Publisher("vertical_thrust",
                                                  Float64,
                                                  queue_size=1)

    def on_sub_setpointz(self, msg): 
        self.z_sp = msg.z
        self.pidz.setpoint = self.z_sp

    def on_sub(self, msg):
        self.z = float(msg.z)
        # Publish lateral thrust
        lat = self.pidz(self.z)
        self.vertical_thrust_pub.publish(Float64(lat))

    def run(self):            
        while not rospy.is_shutdown():
            rospy.spin
           

def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
