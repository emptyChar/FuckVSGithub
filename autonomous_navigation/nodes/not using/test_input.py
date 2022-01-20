#! /usr/bin/env python

from asyncore import write
import rospy
from std_msgs.msg import Int8

class SaveData:
    def __init__(self):
        rospy.init_node("test_action")
        self.now = rospy.get_time()
        self.temp = Int8()
        self.action_publish = rospy.Publisher("test_action",
                                                   Int8,
                                                   queue_size=1)

    
    def write(self, file):
        print(file.readline)
        self.temp = file.readline
        #self.action_publish.publish(self.temp)
    
    def run(self):
        file = open("/home/sri/fav/catkin_ws/src/gesture_test/action.txt", "r")
        while not rospy.is_shutdown():
            write(file)
            rospy.spin()
        file.close()
            # else: 
            #     print(isinstance(temp, int))
            # self.action_publish.publish(temp)
    


def main():
    node = SaveData()
    node.run()


if __name__ == "__main__":
    main()

        