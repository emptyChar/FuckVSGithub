#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math

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
        self.is_following = False
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

        self.setpoint = Point
        self.setpoint.x = 0.7
        self.setpoint.y = 2
        self.setpoint.z = -0.7

        self.setpoint_a = Point
        self.setpoint_a.x = 0.7
        self.setpoint_a.y = 2
        self.setpoint_a.z = -0.7

        self.setpoint_b = Point
        self.setpoint_b.x = 0.7
        self.setpoint_b.y = 2
        self.setpoint_b.z = -0.7

        self.sp_list = Point
        self.sp_a = Point
        self.sp_a.x = 0.7
        self.sp_a.y = 2
        self.sp_a.z = -0.7
        self.sp_b = Point
        self.sp_b.x = 0.7
        self.sp_b.y = 2
        self.sp_b.z = -0.7
        self.sp_list = [self.sp_a, self.sp_b]
        self.sp_list.pop

        self.start_time = rospy.get_time()

        self.setpoint_pub = rospy.Publisher("position_setpoint",
                                            Point, queue_size=1)
        self.follow_pub = rospy.Publisher("is_following",
                                          Bool, queue_size=1)
        self.action_sub = rospy.Subscriber("action_number", Int8,
                                           self.action_callback,
                                           self.setpoint_pub)
        self.pose_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                         Odometry,
                                         self.pose_callback,
                                         queue_size=1)

    def run(self):
        # might need to add to the runfunction the ongoing action?
        rate = rospy.Rate(50.0)
        while not rospy.is_shutdown():
            rate.sleep()

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

    def action_callback(self, msg, publisher):
        # before taking any new action, first check wether or not there
        # is an active action

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

        # regardless of is_active publish if its following for follower node
        self.follow_pub.publish(self.is_following)

# ACTIONS #####################################################################

    def action_0(self):
        self.is_following = True
        # standard following procedure, keep position
        # relative to diver
        # go into that mode whenever possible
        # if not possible hold onto the last setpoint command

    def action_1(self):
        # safe position A

        if self.is_following is True:
            self.is_active = True

            self.setpoint_a.x = self.x
            self.setpoint_a.y = self.y
            self.setpoint_a.z = self.z

        # wait 4 secs before rdy for next action
        waiting = rospy.Rate(4000.0)
        waiting.sleep()

        self.is_active = False

    def action_2(self):
        # safe position B

        if self.is_following is True:
            self.is_active = True

            self.setpoint_b.x = self.x
            self.setpoint_b.y = self.y
            self.setpoint_b.z = self.z

        # wait 4 secs before rdy for next action
        waiting = rospy.Rate(4000.0)
        waiting.sleep()

        self.is_active = False

    def action_3(self):
        # switch between positions A <-> B
        self.is_active = True

        # write the whole thing more compact
        if self.is_following is True:
            self.is_following = False
        if self.is_ab is False:
            self.is_ab = True
        if self.is_a is False:
            self.is_a = True
            self.setpoint.x = self.setpoint_a.x
            self.setpoint.y = self.setpoint_a.y
            self.setpoint.z = self.setpoint_a.z
            dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                   + (self.setpoint.y - self.y)**2
                                   + (self.setpoint.z - self.z)**2)
            while dist_to_sp > 0.05:
                # when its close to its destination it shouldnt stop
                # publishing the setpoints but it should lose the"active"status

                self.setpoint_pub.publish(self.setpoint)
                # How to publish same setpoints afterwards?
        else:
            self.is_a = False
            self.setpoint.x = self.setpoint_b.x
            self.setpoint.y = self.setpoint_b.y
            self.setpoint.z = self.setpoint_b.z
            dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                   + (self.setpoint.y - self.y)**2
                                   + (self.setpoint.z - self.z)**2)
            while dist_to_sp > 0.05:
                self.setpoint_pub.publish(self.setpoint)
        self.is_active = False

    def action_4(self):
        # safe a position to the list
        self.is_active = True

        sp = Point
        sp.x = self.x
        sp.y = self.y
        sp.z = self.z
        self.sp_list.insert(0, sp)
        if len(self.sp_list) > 16:
            # keep list short in case too many sp stored
            self.sp_list.pop()
        waiting = rospy.Rate(1000.0)
        waiting.sleep()

        self.is_active = False

    def action_5(self):
        self.is_active = True
        self.is_following = False

        # first delete the sp_list element, the list was initialized with
        # need to do that exactly one time
        if self.already_done is False:
            self.already_done = True
            self.sp_list.pop()

        le = len(self.sp_list)

        # let bluerov go from setpoint to setpoint
        if self.is_down is True:
            for i in range(le):
                self.setpoint.x = self.sp_list[i].x
                self.setpoint.y = self.sp_list[i].y
                self.setpoint.z = self.sp_list[i].z
                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)
                while dist_to_sp > 0.05:
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
            for i in range(le):
                self.setpoint.x = self.sp_list[le-i].x
                self.setpoint.y = self.sp_list[le-i].y
                self.setpoint.z = self.sp_list[le-i].z
                dist_to_sp = math.sqrt((self.setpoint.x - self.x)**2
                                       + (self.setpoint.y - self.y)**2
                                       + (self.setpoint.z - self.z)**2)
                while dist_to_sp > 0.05:
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


def main():
    act = Action()
    act.run()


if __name__ == "__main__":
    main()
