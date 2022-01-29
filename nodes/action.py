#!/usr/bin/env python
from itertools import count
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
# this node subscribes to the integer number of detected fingers
# it contains 5 actions and depending on the action, it will
# publish a motor setpoint (this node replaces pose_setpoint)

# A0: Tag following mode
# A1: Safe as position 1
# A2: Safe as position 2
# A3: Switch position
# A4: Safe this position to path
# A5: Follow path back to survace / go back to last position

# A0 -> robot is available to take commands
# A3,5 -> robot is active, can not take commands
# BUT once the final sp is reached it can take commands as in A0
# A1,2,4 -> does not change A0 mode of the robot


class Action():
    def __init__(self):
        rospy.init_node("action")

        # Booleans to determine current state
        # can it be achieved better with state machine?
        self.is_active = False
        self.is_following = True
        self.is_ab = False
        self.is_a = False
        self.is_down = True
        self.is_tag_visible = False

        self.already_done = False

        self.t = 0
        self.old_time = 0

        # List that counts number of times the number
        # of fingers was recognized
        self.counter = [0, 0, 0, 0, 0, 0]
        self.total_counter = 0

        # current global position
        self.x = 0.7
        self.y = 2
        self.z = -0.7

        self.setpoint = Point(0.7, 2, -0.3)
        self.setpoint_a = Point(0.7, 2, -0.3)
        self.setpoint_b = Point(0.7, 2, -0.3)
        self.sp_list = Point()
        self.sp_a = Point(0.7, 2, -0.3)
        self.sp_b = Point(0.7, 2, -0.3)

        self.sp_list = [self.sp_a, self.sp_b]
        self.sp_list.pop

        self.start_time = rospy.get_time()

        self.setpoint_pub = rospy.Publisher("position_setpoint",
                                            Point,
                                            queue_size=1)

        self.follow_pub = rospy.Publisher("is_following",
                                          Bool, queue_size=1)
        self.action_sub = rospy.Subscriber("fingercounter", Int8,
                                           self.action_callback)
        self.pose_sub = rospy.Subscriber("/bluerov/mavros/vision_pose/pose_cov",
                                         PoseWithCovarianceStamped,
                                         self.pose_callback,
                                         queue_size=1)
        self.actionstat_pub = rospy.Publisher("action_status",
                                              Int8,
                                              queue_size=1)

        # Publisher for debugging purposes:
        self.finger_pub = rospy.Publisher("debug_finger_read",
                                          Int8,
                                          queue_size=1)
        self.active_pub = rospy.Publisher("debug_is_active",
                                          Bool,
                                          queue_size=1)
        self.pubbi = rospy.Publisher("debug_sp",
                                     Point,
                                     queue_size=1)

    def run(self):
        # might need to add to the runfunction the ongoing action
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def action_callback(self, msg):
        # before taking any new action, first check wether or not there
        # is an active action
        isact = Bool()
        isact = self.is_active
        self.active_pub.publish(isact)

        self.setpoint_pub.publish(self.setpoint)

        if self.is_active is False:
            # take in a published message that contains an integer number 1-5

            # need to do the time window counting here
            # make it only possible while no other action active

            now = rospy.get_time()
            self.t += now - self.old_time
            self.old_time = now

            index = msg.data
            self.counter[index] += 1
            self.total_counter += 1

            if self.t > 3:
                self.t = 0
                self.old_time = 0

                for i in range(6):
                    if (self.counter[i] / self.total_counter) > 0.6:
                        actio = i

                self.counter = [0, 0, 0, 0, 0, 0]
                self.total_counter = 0

                # Debug Publishing of finger count:
                acti = Int8()
                acti = actio
                self.finger_pub.publish(acti)

                if actio == 1:
                    self.action_1()
                elif actio == 2:
                    self.action_2()
                elif actio == 3:
                    self.action_3()
                elif actio == 4:
                    self.action_4()
                elif actio == 5:
                    self.action_5()
                else:
                    self.action_0()

                # here it might be needed to define an ongoing setpoint
                # following after action is over
                follo = Bool(self.is_following)
                self.follow_pub.publish(follo)

    # ACTIONS #####################################################################

    def action_0(self):

        # NEEDS WORK

        # standard following procedure, keep position
        # relative to diver
        self.is_following = True
        statvar = 0
        self.actionstat_pub.publish(statvar)

    def action_1(self):
        # safe position A

        if self.is_following is True:
            self.is_active = True

            statvar = 1
            self.actionstat_pub.publish(statvar)

            isact = Bool()
            isact = self.is_active
            self.active_pub.publish(isact)

            self.setpoint_a.x = self.x
            self.setpoint_a.y = self.y
            self.setpoint_a.z = self.z

        # wait 4 secs before rdy for next action
        waiting = rospy.Rate(4000.0)
        waiting.sleep()

        self.is_active = False
        statvar = 6
        self.actionstat_pub.publish(statvar)

    def action_2(self):
        # safe position B

        if self.is_following is True:
            self.is_active = True

            statvar = 2
            self.actionstat_pub.publish(statvar)

            self.setpoint_b.x = self.x
            self.setpoint_b.y = self.y
            self.setpoint_b.z = self.z

        # wait 4 secs before rdy for next action
        waiting = rospy.Rate(4000.0)
        waiting.sleep()

        self.is_active = False
        statvar = 6
        self.actionstat_pub.publish(statvar)

    def action_3(self):
        # switch between positions A <-> B
        self.is_active = True

        statvar = 3
        self.actionstat_pub.publish(statvar)

        # following mode: switch following off switch is AB on
        if self.is_following is True:
            self.is_following = False
        if self.is_ab is False:
            self.is_ab = True
        if self.is_a is False:
            self.is_a = True
            self.setpoint = self.setpoint_a
            dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                   + (self.setpoint.y - self.y)**2
                                   + (self.setpoint.z - self.z)**2)
            while dist_to_sp > 0.1:
                # when its close to its destination it shouldnt stop
                # publishing the setpoints but it should lose the"active"status

                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)

                self.setpoint_pub.publish(self.setpoint_a)
                # How to publish same setpoints afterwards?
        else:
            self.is_a = False
            self.setpoint = self.setpoint_b
            dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                   + (self.setpoint.y - self.y)**2
                                   + (self.setpoint.z - self.z)**2)
            while dist_to_sp > 0.1:

                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)
                self.setpoint_pub.publish(self.setpoint_b)

        self.is_active = False
        statvar = 6
        self.actionstat_pub.publish(statvar)

    def action_4(self):
        # safe a position to the list
        self.is_active = True

        statvar = 4
        self.actionstat_pub.publish(statvar)

        sp = Point(self.x, self.y, self.z)
        self.sp_list.insert(0, sp)

        if len(self.sp_list) > 16:
            # keep list short in case too many sp stored
            self.sp_list.pop()
        self.setpoint = self.sp_list[0]
        waiting = rospy.Rate(1000.0)
        waiting.sleep()
        self.is_active = False
        statvar = 6
        self.actionstat_pub.publish(statvar)

    def action_5(self):
        self.is_active = True

        statvar = 5
        self.actionstat_pub.publish(statvar)

        self.is_following = False
        self.follow_pub.publish(self.is_following)
        # first delete the sp_list element, the list was initialized with
        # need to do that exactly one time

        # let bluerov go from setpoint to setpoint
        if self.is_down is True:
            for i in range(len(self.sp_list)):
                self.setpoint = self.sp_list[i]
                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)
                while dist_to_sp > 0.1:
                    self.setpoint_pub.publish(self.setpoint)
                    hold = rospy.Rate(100)
                    hold.sleep()
                    dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                           + (self.setpoint.y - self.y)**2
                                           + (self.setpoint.z - self.z)**2)
            hold = rospy.Rate(5000)
            hold.sleep()
            self.is_down = False
        else:
            for i in range(len(self.sp_list)):
                self.setpoint = self.sp_list[len(self.sp_list)-i]
                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)
                while dist_to_sp > 0.1:
                    self.setpoint_pub.publish(self.setpoint)
                    hold = rospy.Rate(100)
                    hold.sleep()
                    dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                           + (self.setpoint.y - self.y)**2
                                           + (self.setpoint.z - self.z)**2)
            hold = rospy.Rate(5000)
            hold.sleep()
            self.is_down = True
        self.is_active = False
        statvar = 6
        self.actionstat_pub.publish(statvar)


def main():
    act = Action()
    act.run()


if __name__ == "__main__":
    main()
