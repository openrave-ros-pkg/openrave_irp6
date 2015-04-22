#!/usr/bin/env python

import rospy
import tf
import actionlib



import time

import openraveIrp6

#MAIN

if __name__ == '__main__':
	env, robot = openraveIrp6.initialize()
	print "Done"
	robot.postument.move_to_joint_position([0, -1.54, 0, 0, 4.71, -1.57])
	print "done"
	time.sleep(100)
