#!/usr/bin/env python

import rospy
from rospy import init_node, is_shutdown

if __name__ == '__main__':
  init_node('input_test')
  while not rospy.is_shutdown():

      print("gimme something, please?")
      something = input()
      print("thanks for giving me ") + something
