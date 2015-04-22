#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2009-2011 Rosen Diankov (rosen.diankov@gmail.com)
# 

from __future__ import with_statement # for python 2.5
__author__ = 'Przemo'

from irp6kinematic import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def moveManipulator(Tgoal,ikmodel,basemanip,robot):
    sol = ikmodel.manip.FindIKSolutions(IkParameterization(Tgoal,IkParameterization.Type.Transform6D),IkFilterOptions.CheckEnvCollisions)
    print "Jest az " + str(sol.size) + "rozwiazan!"
    if sol.size:
	print "Realizacja"
	traj=basemanip.MoveManipulator(sol[0],outputtrajobj=True,execute=False) #to dziala
	#traj=basemanip.MoveActiveJoints(sol[0],outputtrajobj=True,execute=True)# i to dziala
	#robot.SetDOFValues(sol[0],ikmodel.manip.GetArmIndices()) #ustawia chwytak, nie porusza
	#http://openrave.org/docs/latest_stable/coreapihtml/classOpenRAVE_1_1TrajectoryBase.html
	#print "Trajektoria to " + str(traj.GetNumWaypoints())

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
		joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5],w[jointGroup.offset+6]]);
		vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5],w[velGroup.offset+6]]);
	print "trajectory to " + str(vels[2]);
	robot.GetController().SetPath(traj)
	#robot.WaitForController(0)

def main(env):
	"Main example code."
	env.SetViewer('qtcoin')
	env.Load('data/irp6both.env.xml')
	EPS = 1e-10
	
    #physics = RaveCreatePhysicsEngine(env,'ode')
	# env.SetPhysicsEngine(physics)
	#handle = env.RegisterCollisionCallback(collisioncallback)

	robot = env.GetRobots()[0]
	irp6Kinematic = Irp6Kinematic(env,robot)
    
	basemanip = interfaces.BaseManipulation(robot)
	taskprob = interfaces.TaskManipulation(robot)
    
	postument = robot.SetActiveManipulator('postument');
	track = robot.SetActiveManipulator('track');
    
	robot.SetActiveDOFs(postument.GetArmIndices());
	robot.SetActiveDOFs(track.GetArmIndices());
	time.sleep(5)
	pos = [0.865904485399, 1.00059724452807, 1.00842285353]
	
	robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol = irp6Kinematic.solveIKPost([-0.30736250029, -0.787407586002, 0.0117646824742, 0.534208928387],[-0.139239407935, -0.74380811192, 1.15148372188])
		sol = [-1.38227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624]
		print sol
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=True)
	time.sleep(4)

	robot.SetActiveManipulator('track');	
	with env:
		print "jak jest"
		sol = irp6Kinematic.solveIKTrack([-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.60904485399, 1.30059724452807, 1.20842285353])
		print sol
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=True)
	time.sleep(4)
	
	robot.SetActiveManipulator('track');	
	with env:
		print "jak jest"
		sol = irp6Kinematic.solveIKTrack([-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.00904485399, 1.30059724452807, 1.20842285353])
		print sol
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=True)
	time.sleep(4)
	
	time.sleep(100)

env = Environment()
main(env)
