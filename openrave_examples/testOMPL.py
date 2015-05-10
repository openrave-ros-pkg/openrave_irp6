#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Shows how to use 5D translation+direction inverse kinematics for an arm with >= 5 joints.

.. examplepre-block:: tutorial_ik5d

.. examplepost-block:: tutorial_ik5d
"""
from __future__ import with_statement # for python 2.5
__author__ = 'Rosen Diankov'

import irp6kinematic

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def main(env):
	"Main example code."
	env.SetViewer('qtcoin')
	#env.Load('data/irp6both.env.xml')
	EPS = 1e-10
	
	module = RaveCreateModule(env, 'urdf')
	name = module.SendCommand('load package://irp6_description/robots/irp6pboth.urdf.xacro package://irp6_description/robots/irp6pboth.srdf')
	robot = env.GetRobot(name)
 
	planner = RaveCreatePlanner(env, 'OMPL_RRTConnect')
	simplifier = RaveCreatePlanner(env, 'OMPL_Simplifier')
  
	basemanip = interfaces.BaseManipulation(robot)
	taskprob = interfaces.TaskManipulation(robot)

	post=robot.SetActiveManipulator('postument');
	robot.SetActiveDOFs(post.GetArmIndices());

	# Setup the planning instance.
	params = Planner.PlannerParameters()
	params.SetRobotActiveJoints(robot)
	goal = [-1.4, -1.54, 0, 0.7, 3.75, -1.9]
	params.SetGoalConfig(goal)

	params.SetExtraParameters('<range>0.02</range>')

	planner.InitPlan(robot, params)

	# Invoke the planner.
	traj = RaveCreateTrajectory(env, '')
	result = planner.PlanPath(traj)
	assert result == PlannerStatus.HasSolution

	# Shortcut the path.
	simplifier.InitPlan(robot, Planner.PlannerParameters())
	result = simplifier.PlanPath(traj)
	assert result == PlannerStatus.HasSolution

	# Time the trajectory.
	result = planningutils.RetimeTrajectory(traj)
	assert result == PlannerStatus.HasSolution

	# Execute the trajectory.
	robot.GetController().SetPath(traj)
	while not robot.GetController().IsDone():
		time.sleep(0.01)	
	
	conf = traj.GetConfigurationSpecification();
	try:
		jointGroup = conf.GetGroupFromName("joint_values")
	except openrave_exception:
		jointGroup = None;
	try:
		velGroup = conf.GetGroupFromName("joint_velocities")
	except openrave_exception:
		velGroup = None;
	joints = []
	vels = []
	for i in range (0,traj.GetNumWaypoints()):
		w=traj.GetWaypoint(i);		
		joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5]]);
		vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5]]);
	
	for i in range(0,len(joints)):
		print joints[i]
		print "		"+str(vels[i])
		
	time.sleep(100)

env = Environment()
main(env)
