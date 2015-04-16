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
    env.Load('data/mojTutorial.env.xml') #TODO stworzyc nowy env file
    physics = RaveCreatePhysicsEngine(env,'ode')
    env.SetPhysicsEngine(physics)
    #handle = env.RegisterCollisionCallback(collisioncallback)

    robot = env.GetRobots()[0]

    #wrzucamy kubas na scene wazne zeby przed wrzuceniem okreslic jego parametry jesli chcemy odnosic sie do niego za pomoca mugBody1
    mugBody1 = env.ReadKinBodyXMLFile('data/mug1.kinbody.xml')
    #i go przesuwamy
    Tr = numpy.array([[1,0,0,0.1],[0,1,0,-0.1],[0,0,1,1.35],[0,0,0,1]])
    mugBody1.SetTransform(Tr)
    #pozycja kubka (macierz przeksztalcen)
    #print mugBody1.GetTransform()
    env.Add(mugBody1)

    #Sprawdzamy czy jest kolizja w TEJ CHWILI!! (bool zwraca)
    #env.CheckCollision(robot,mugBody1)

    #robot porusza aktywnym manipulatorem manipulatorRightArm ma glownie gety i IK
    manipulatorRightArm = robot.SetActiveManipulator('rightarm')

    # generate the ik solver
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Transform6D)
    if not ikmodel.load():
        ikmodel.autogenerate()
    #Pobiera przekształcenie
    #print manipulatorRightArm.GetEndEffectorTransform()
    
    basemanip = interfaces.BaseManipulation(robot)
    robot.SetActiveDOFs(manipulatorRightArm.GetArmIndices());
    taskprob = interfaces.TaskManipulation(robot)

    #print 'move the arm to the target'
    #Tgoal = array([[0,-1,0,-0.0],[-1,0,0,0.0],[0,0,-1,0.9],[0,0,0,1]])
    #res = basemanip.MoveToHandPosition(matrices=[Tgoal],seedik=16)
    #waitrobot(robot)
    time.sleep(2)
    while True:
	
	print "Position 1"
	print "Wyznaczanie"
    	Tgoal = numpy.array([[0,0,1,0.8],[0,1,0,0.3],[-1,0,0,1.6],[0,0,0,1]])
	with env:
	     moveManipulator(Tgoal,ikmodel,basemanip,robot)


	#print manipulatorRightArm.GetEndEffectorTransform()
    
	time.sleep(3)
	print "Position 2"
	print "Wyznaczanie"
	Tgoal = numpy.array([[0,-1,0,-0.0],[-1,0,0,0.0],[0,0,-1,0.9],[0,0,0,1]])
	with env: # lock environment
		moveManipulator(Tgoal,ikmodel,basemanip,robot);
		  
	#print manipulatorRightArm.GetEndEffectorTransform()
	#c = float(input('?'))

	time.sleep(3)
    #time.sleep(100)

env = Environment()
main(env)
