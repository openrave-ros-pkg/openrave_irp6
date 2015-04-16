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
	
	print robot
    
	basemanip = interfaces.BaseManipulation(robot)
	taskprob = interfaces.TaskManipulation(robot)
    
	postument = robot.SetActiveManipulator('postument');
	track = robot.SetActiveManipulator('track');
    
	robot.SetActiveDOFs(postument.GetArmIndices());
	robot.SetActiveDOFs(track.GetArmIndices());
	time.sleep(5)
    
	track = robot.SetActiveManipulator('track');
	#try:
	print "Pos 1"
	with env:
		print "jak powinno"
		sol1 = [0,0, -1.54, 0, 0, 4.71, -1.57]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.865904485399, -0.00059724452807, 1.12842285353])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	#except planning_error:
	#	print "Wrong destination point"

	print "Pos 2"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,9.219067141191173e-16, -1.5081779149982526, 0.147818277583954, -0.17964036258569438, 4.709999999999998, -1.569999999999999]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.865904485399, -0.00059724452807, 1.00842285353])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	print "Pos 3"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,-0.22505867418321274, -1.4591202279539133, 0.09617596300825593, -0.17836482982039148, 4.703188608154266, -1.7949598054768927]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.865904485399, -0.20059724452807, 1.00842285353])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)

	print "Pos 4"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[0.60625134692, -0.475428409334, 0.0512887793112, 0.635449913898],[0.615015785585, -0.280596693012, 1.14965358893])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	print "Pos 5"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,1.0, -1.57, -1.1329912702473521e-16, 2.220446049250313e-16, 2.71, -1.57]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[-0.258658489275, 0.473219697292, -0.738968267829, 0.403837595032],[0.655977762852,0.601350547649,1.50889446677])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	print "Pos 6"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,0.4, -1.5418065817051163, 0.0, 1.5, 1.57, 2.0]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[0.00981929342874, 0.692102872276, 0.290984336661, 0.660473550933],[0.866299511382,0.366481702667,1.20600302613])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	print "Pos 7"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,0.5, -1.5, 0.0, 1, 0, 2.0]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[-0.620544580564, 0.339005049421, 0.595009839529, 0.38205142437],[0.574836146073, 0.598907899683, 1.18972560703])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	sol1= [0,0.5000000000001051, -1.655934404627124, 0.2950903716181458, 0.1900477062140818, -6.195019952223217e-13, 2.670796326794247]
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	print "Pos 8"
	track = robot.SetActiveManipulator('track');
	with env:
		print "jak powinno"
		sol1 = [0,-0.3, -1.65, 0.3, 1.2, 3.14, 2.670]
		traj=basemanip.MoveManipulator(sol1,outputtrajobj=True,execute=True)
	time.sleep(4)
	postument = robot.SetActiveManipulator('postument');
	with env:
		print "jak jest"
		sol2 = irp6kinematic.solveIKPost(env,[0.707010850524, 0.000554894934739, -0.208462447667, 0.675780110129],[0.546403266155,-0.43070990562,1.07654731804])
		traj=basemanip.MoveManipulator(sol2,outputtrajobj=True,execute=True)
	sol1 = [0,-0.2999999999999599, -1.655934404656955, 0.2950903716459135, 1.190047706143288, 3.140000000000462, 2.67079632667529]
	if math.fabs(sol1[1]-sol2[0])<EPS and math.fabs(sol1[2]-sol2[1])<EPS and math.fabs(sol1[3]-sol2[2])<EPS and math.fabs(sol1[4]-sol2[3])<EPS and math.fabs(sol1[5]-sol2[4])<EPS and math.fabs(sol1[6]-sol2[5])<EPS:
		print "OK"
	else:
		print "NOT OK"
		print sol2
		print str(sol1[1]-sol2[0])+ "  " + str(sol1[2]-sol2[1])+ "  " + str(sol1[3]-sol2[2])+ "  " + str(sol1[4]-sol2[3])+ "  " + str(sol1[5]-sol2[4])+ "  " + str(sol1[6]-sol2[5])
	time.sleep(4)
	
	
	time.sleep(100)

env = Environment()
main(env)
