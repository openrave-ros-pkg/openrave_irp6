#!/usr/bin/env python

import rospy
import tf
import actionlib

import time
import numpy
import openrave_irp6

#MAIN

if __name__ == '__main__':
	env, robot = openrave_irp6.initialize(manageIrpos=False)
	print "Done"
	mugBody1 = env.ReadKinBodyXMLFile('data/cube.kinbody.xml')
	if mugBody1 is None:
		print "O kurwa"
	env.Add(mugBody1)
	robot.track.attachItem(mugBody1)
	time.sleep(100)
	robot.postument.releaseItem(mugBody1)
	robot.track.attachItem(mugBody1)
	robot.postument.moveToSynchroPosition()
	time.sleep(100)
	#robot.track.moveToSynchroPosition()
	print "done"
