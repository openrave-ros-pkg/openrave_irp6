#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
from operator import *

from irp6kinematic import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class Irp6Manipulator:
	def __init__(self,env,manipulator,baseManipulation):
		if manipulator.GetName()!='postument' and manipulator.GetName()!='track':
			print "Unhadled manipulator!"
		self.env=env
		self.manipulator=manipulator
		self.baseManipulation=baseManipulation
	def move_to_joint_position(self,joints):
		with self.env:
			self.manipulator.GetRobot().SetActiveManipulator(self.manipulator.GetName());
			traj=self.baseManipulation.MoveManipulator(joints,outputtrajobj=True,execute=True)
		time.sleep(4)
	
