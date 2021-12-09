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
        rospy.init_node("controllery")        
        self.y_sp = 2.0        
        self.no_tag_detection = Int32(0)
        self.pidy = PID(1, 0.1, 0.05, setpoint=self.y_sp)       
        self.pidy.output_limits = (-0.1, 0.1)        
        # define variables        
        self.y = 2.0  # y estimate

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
                                            self.on_sub_setpointy,
                                            queue_size=1)

        # Publisher
        self.lateral_thrust_pub = rospy.Publisher("thrust",
                                                  Float64,
                                                  queue_size=1)
    
    def on_sub_tag(self, msg):
        self.no_tag_detection = msg

    def on_sub_setpointy(self, msg): 
        self.y_sp = msg.y
        self.pidy.setpoint = self.y_sp

    def on_sub(self, msg):
        self.y = float(msg.y)
        # Publish lateral thrust
        if self.no_tag_detection.data < 10:
            lat = self.pidy(self.y)
            self.lateral_thrust_pub.publish(Float64(lat))

    def run(self):            
        while not rospy.is_shutdown():
            rospy.spin
           

def main():
    controller = Control()
    controller.run()


if __name__ == "__main__":
    main()
