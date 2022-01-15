#!/usr/bin/env python
import rospy
from std_msgs.msg import Int8
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

        self.start_time = rospy.get_time()

        self.setpoint_pub = rospy.Publisher("position_setpoint",
                                            Point, queue_size=1)
        self.action_sub = rospy.Subscriber("action_number", Int8,
                                           self.action_callback,
                                           self.setpoint_pub)
        self.pose_sub = rospy.Subscriber("/bluerov/ground_truth/state",
                                         Odometry,
                                         self.pose_callback,
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
                    self.setpoint = self.action_1
                elif actio == 2:
                    self.setpoint = self.action_2
                elif actio == 3:
                    self.setpoint = self.action_3
                elif actio == 4:
                    self.setpoint = self.action_4
                elif actio == 5:
                    self.setpoint = self.action_5
                else:
                    self.setpoint = self.action_0

                # here it might be needed to define an ongoing setpoint
                # following after action is over

                self.setpoint_pub.publish(self.setpoint)

# ACTIONS #####################################################################

    def action_0(self):
        # NEEDS WORK

        # standard following procedure, keep position
        # relative to diver

        sp = Point
        sp.x = 0.7
        sp.y = 2
        sp.z = -0.7
        self.setpoint_pub.publish(sp)

    def action_1(self):
        # safe position A

        if self.is_following is False:
            a = 0
        else:
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

        if self.is_following is False:
            a = 0
        else:
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

        # # following mode: switch following off switch is AB on
        # if self.is_following is True:
        #     self.is_following = False
        #     self.is_ab = True
        #     if self.is_a is False:
        #         self.is_a = True
        #         self.setpoint.x = self.setpoint_a.x
        #         self.setpoint.y = self.setpoint_a.y
        #         self.setpoint.z = self.setpoint_a.z
        #     else:
        #         self.is_a = False
        #         self.setpoint.x = self.setpoint_b.x
        #         self.setpoint.y = self.setpoint_b.y
        #         self.setpoint.z = self.setpoint_b.z
        # elif self.is_ab is True:
        #     # state here what should happen when bluerov is already
        #     # moving between A and B
        #     if self.is_a is False:
        #         self.is_a = True
        #         self.setpoint.x = self.setpoint_a.x
        #         self.setpoint.y = self.setpoint_a.y
        #         self.setpoint.z = self.setpoint_a.z
        #     else:
        #         self.is_a = False
        #         self.setpoint.x = self.setpoint_b.x
        #         self.setpoint.y = self.setpoint_b.y
        #         self.setpoint.z = self.setpoint_b.z
        # else:
        #     # else meaning bluerov is in up or down position
        #     self.is_ab = True
        #     if self.is_a is False:
        #         self.is_a = True
        #         self.setpoint.x = self.setpoint_a.x
        #         self.setpoint.y = self.setpoint_a.y
        #         self.setpoint.z = self.setpoint_a.z
        #     else:
        #         self.is_a = False
        #         self.setpoint.x = self.setpoint_b.x
        #         self.setpoint.y = self.setpoint_b.y
        #         self.setpoint.z = self.setpoint_b.z

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
            while dist_to_sp < 0.05:
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
            while dist_to_sp < 0.05:
                self.setpoint_pub.publish(self.setpoint)



        self.is_active = True

        sp = Point
        if self.is_a is False:
            sp.x = self.setpoint_a.x
            sp.y = self.setpoint_a.y
            sp.z = self.setpoint_a.z

            dist_to_sp = math.sqrt((sp.x - self.x)**2
                                   + (sp.y - self.y)**2
                                   + (sp.z - self.z)**2)
            while dist_to_sp < 0.15:
                # when its close to its destination it shouldnt stop
                # publishing the setpoints but it should lose the"active"status

                self.setpoint_pub.publish(sp)
        else:
            sp.x = self.setpoint_b.x
            sp.y = self.setpoint_b.y
            sp.z = self.setpoint_b.z

    def action_4(self):
        # safe a position to the list
        sp = Point
        sp.x = self.x
        sp.y = self.y
        sp.z = self.z
        self.sp_list.append(sp)

    def action_5(self):
        sp = Point
        sp.x = 0.7
        sp.y = 2
        sp.z = -0.7
        self.setpoint_pub.publish(sp)


def main():
    act = Action()
    act.run()


if __name__ == "__main__":
    main()
