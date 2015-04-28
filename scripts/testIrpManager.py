#!/usr/bin/env python

import rospy
import tf
import actionlib

import time

import openraveIrp6

#MAIN

if __name__ == '__main__':
	env, robot = openraveIrp6.initialize(manageIrpos=True)
	print "Done"
	#robot.postument.move([-1.28227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624])
	
	robot.postument.moveToSynchroPosition()
	robot.track.moveToSynchroPosition()
	robot.postument.moveToJointPosition([-1.28227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624])
	robot.track.moveToCartesianPosition(Pose(Point(0.60904485399, 1.30059724452807, 1.20842285353), Quaternion(-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608)))
	robot.track.moveToCartesianPosition(Pose(Point(0.00904485399, 1.30059724452807, 1.20842285353), Quaternion(-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608)))
	robot.postument.moveRelativeToJointPosition([0, 0, 0, -0.8, 0, 0])
	robot.track.moveToCartesianPosition(Pose(Point(0.60904485399, 1.30059724452807, 1.20842285353), Quaternion(-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608)))
	robot.postument.moveToSynchroPosition()
	robot.track.moveToSynchroPosition()
	print "done"
