#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
from operator import *

from irp6kinematic import *
from irp6Manipulator import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class Irp6Robot:
	def __init__(self,env,robot):
		self.robot=robot

		self.irp6Kinematic = Irp6Kinematic(env,robot)
		baseManipulation = interfaces.BaseManipulation(robot)
		taskprob = interfaces.TaskManipulation(robot)
		
		postument=robot.SetActiveManipulator('postument');
		track=robot.SetActiveManipulator('track');
		robot.SetActiveDOFs(postument.GetArmIndices());
		robot.SetActiveDOFs(track.GetArmIndices());
		
		self.postument = Irp6Manipulator(env,postument,baseManipulation)
