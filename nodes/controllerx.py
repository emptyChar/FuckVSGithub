#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from rospy.topics import Publisher
from std_msgs.msg import Float64
from simple_pid import PID
from std_msgs.msg       import Int32
# from nav_msgs.msg import Odometry


# subscribe: x,y,z-est; x,y,z-setpoint; yaw-est; yaw-sp
# publish: thrust, lateral thrust, vertical thrust, yaw


class Control():
    def __init__(self):
        rospy.init_node("controllerx")
        self.x_sp = 0.4
        self.no_tag_detection = Int32(0)
                        
        self.pidx = PID(1, 0.1, 0.05, setpoint=self.x_sp)

        self.pidx.output_limits = (-0.1, 0.1)

        # define variables
        self.x = 0.7  # x estimate
 

        self.xy_sub = rospy.Subscriber("position",
                                        Point,
                                        self.on_sub,
                                        queue_size=1)

        self.tag_sub = rospy.Subscriber("no_tag_detection_error",
                                        Int32,  
                                        self.on_sub_tag,                                      
                                        queue_size=1)

        self.setpoint_sub = rospy.Subscriber("pose_setpoint",
                                            Point,
                                            self.on_sub_setpointx,
                                            queue_size=1)

        # Publisher
        self.thrust_pub = rospy.Publisher("lateral_thrust",
                                          Float64,
                                          queue_size=1)


    # on every subscription, assign the topics current value to a variable
    # that has been defined in __init__

    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub_setpointx(self, msg): 
        self.x_sp = msg.x
        self.pidx.setpoint = self.x_sp

    def on_sub(self, msg):   
        self.x = float(msg.x)        
        # Publish forward thrust
        if self.no_tag_detection.data < 10:
            thrust = -self.pidx(self.x)
            self.thrust_pub.publish(Float64(thrust))



    def run(self):            
        while not rospy.is_shutdown():
            rospy.spin

            

def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
