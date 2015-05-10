#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
from operator import *

from irp6Robot import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def initialize(mode='urdf',xmlFile='data/irp6both.env.xml',viewerEnabled=True,manageIrpos=True,planner=None,simplifier='OMPL_Simplifier'):
	env = Environment()
	if viewerEnabled==True:
		env.SetViewer('qtcoin')
		
	if mode=='urdf':
		module = RaveCreateModule(env, 'urdf')
		name = module.SendCommand('load package://irp6_description/robots/irp6pboth.urdf.xacro package://irp6_description/robots/irp6pboth.srdf')
		robot = env.GetRobot(name)
		conveyor = env.ReadKinBodyXMLFile('data/conveyor.kinbody.xml')
		env.Add(conveyor)
	else:
		env.Load(xmlFile)
		robot = env.GetRobots()[0]
		
	irp6Robot = Irp6Robot(env,robot,manageIrpos,planner)	
	
	
	return env, irp6Robot

