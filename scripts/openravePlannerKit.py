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

class OpenravePlannerKit:
	#
	#	env 
	#	robot
	#	irp6Kinematic			IK FK RIK for irp
	#	baseManipulation		used to manipulate robots in openrave (planning)
	#	postument				manipulate real postument
	#	track					manipulate real track
	#	oPostument				pointer to openrave postument
	#	oTrack					pointer to openrave track
	#
	def __init__(self):
		self.env = Environment()
		self.env.SetViewer('qtcoin')
		self.postument = IRPOS("IRpOS", "Irp6p", 6)
		self.track = IRPOS("IRpOS", "Irp6ot", 7)
		print "Inicjacja domyslna"
	
		module = RaveCreateModule(env, 'urdf')
		name = module.SendCommand('load package://irp6_description/robots/irp6pboth.urdf.xacro package://irp6_description/robots/irp6pboth.srdf')
		self.robot = env.GetRobot(name)
		conveyor = env.ReadKinBodyXMLFile('data/conveyor.kinbody.xml')
		env.Add(conveyor)
		
		self.irp6Kinematic = Irp6Kinematic(self.env,self.robot)
		self.baseManipulation = interfaces.BaseManipulation(robot)
		taskprob = interfaces.TaskManipulation(robot)
    
		self.oPostument = robot.SetActiveManipulator('postument');
		self.oTrack = robot.SetActiveManipulator('track');
    
		self.robot.SetActiveDOFs(postument.GetArmIndices());
		self.robot.SetActiveDOFs(track.GetArmIndices());
		
	def __init__(self,mode):
		self.env = Environment()
		self.env.SetViewer('qtcoin')
		self.postument = IRPOS("IRpOS", "Irp6p", 6)
		self.track = IRPOS("IRpOS", "Irp6ot", 7)
		
		if mode=='urdf':
			print "Inicjacja urdfowa"
			module = RaveCreateModule(env, 'urdf')
			name = module.SendCommand('load package://irp6_description/robots/irp6pboth.urdf.xacro package://irp6_description/robots/irp6pboth.srdf')
			self.robot = env.GetRobot(name)
			conveyor = env.ReadKinBodyXMLFile('data/conveyor.kinbody.xml')
			self.env.Add(conveyor)
		else:
			print "Inicjacja colladowa"
			self.env.Load('data/irp6both.env.xml')
			self.robot = env.GetRobots()[0]
		
		irp6Kinematic = Irp6Kinematic(self.env,self.robot)
		self.baseManipulation = interfaces.BaseManipulation(robot)
		taskprob = interfaces.TaskManipulation(robot)
    
		self.oPostument = robot.SetActiveManipulator('postument');
		self.oTrack = robot.SetActiveManipulator('track');
    
		self.robot.SetActiveDOFs(postument.GetArmIndices());
		self.robot.SetActiveDOFs(track.GetArmIndices());

