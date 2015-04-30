#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
import operator

from irp6kinematic import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class Irp6Manipulator:
	
	HEADERC = '\033[95m'
	OKBLUEC = '\033[94m'
	OKGREENC = '\033[92m'
	WARNINGC = '\033[93m'
	FAILC = '\033[91m'
	ENDC = '\033[0m'
	
	def __init__(self,env,manipulator,baseManipulation,kinematicSolver,irpos=None):
		self.env=env
		self.manipulator=manipulator
		self.baseManipulation=baseManipulation
		self.irpos = irpos
		self.kinematicSolver=kinematicSolver
		
		self.updateManipulatorPosition()
	
	def waitForRobot(self):
		robot = self.manipulator.GetRobot()
		while not robot.GetController().IsDone():
			time.sleep(0.01)
			
			
	def updateManipulatorPosition(self):
		robot = self.manipulator.GetRobot()
		if self.irpos!=None:
			robot.SetDOFValues(self.irpos.get_joint_position(),self.manipulator.GetArmIndices())
		else:
			print self.WARNINGC+"[OpenRAVEIrp6] Irpos not set"+self.ENDC
	
		
	def moveToJointPosition(self,dstJoints,simulate=True):
		#Check length of joints list
		if len(self.manipulator.GetArmIndices())!=len(dstJoints):
			print self.FAILC+"[OpenRAVEIrp6] Joints length error"+self.ENDC
		robot = self.manipulator.GetRobot()
	
		robot.SetActiveManipulator(self.manipulator.GetName());
		robot.SetActiveDOFs(self.manipulator.GetArmIndices());
		
		#set starting position same as real robot
		if self.irpos!=None:
			print "POSITION: "+ self.manipulator.GetName()+ " " + str(self.irpos.get_joint_position())
			print "DESTINATION: "+ self.manipulator.GetName()+ " " + str(dstJoints)
			robot.SetDOFValues(self.irpos.get_joint_position(),self.manipulator.GetArmIndices())
		#planning trajectory
		conf=None
		try:
			with self.env:
				traj=self.baseManipulation.MoveActiveJoints(dstJoints,outputtrajobj=True,execute=simulate)
			self.waitForRobot()
			robot.SetDOFValues(dstJoints,self.manipulator.GetArmIndices())
			conf = traj.GetConfigurationSpecification();
		except planning_error:
			print self.FAILC+"[OpenRAVEIrp6] Destination or start point in collision. Cannot plan trajectory"+self.ENDC
		
		#moving real robot
		if self.irpos!=None and conf!=None:
			#getting trajectory data
			try:
				jointGroup = conf.GetGroupFromName("joint_values")
			except openrave_exception:
				jointGroup = None;
			try:
				velGroup = conf.GetGroupFromName("joint_velocities")
			except openrave_exception:
				velGroup = None;
			try:
				gr_tim = conf.GetGroupFromName("deltatime")
			except openrave_exception:
				gr_tim = None
				times = None
			joints = []
			vels = []
			accs = []
			times = []
			for i in range (0,traj.GetNumWaypoints()):
				w=traj.GetWaypoint(i);
				if self.manipulator.GetName()=='postument':
					joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5]]);
					vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5]]);
				elif self.manipulator.GetName()=='track':
					joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5],w[jointGroup.offset+6]]);
					vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5],w[velGroup.offset+6]]);					
				times.append(w[gr_tim.offset])
			#building path
			points = []
			delay=0;
			for i in range(1,len(joints)):
				print "points: " + str(joints[i])
				print "vels: " + str(vels[i])
				delay = delay+times[i]
				print "delay: " +str(delay)
				points.append( JointTrajectoryPoint(joints[i], vels[i], [], [], rospy.Duration(delay)) )
			#moving
			if len(points)!=0:
				self.irpos.move_along_joint_trajectory(points)
			else:
				print self.OKBLUEC+"[OpenRAVEIrp6] Manipulator already in position"+self.ENDC
				
				
	def moveRelativeToJointPosition(self,joints,simulate=True):
		#Check length of joints list
		if len(self.manipulator.GetArmIndices())!=len(joints):
			print self.FAILC+"[OpenRAVEIrp6] Joints length error"+self.ENDC
		robot = self.manipulator.GetRobot()
		
		#set starting position same as real robot
		if self.irpos!=None:
			robot.SetDOFValues(self.irpos.get_joint_position(),self.manipulator.GetArmIndices())
		recentJoints=self.irpos.get_joint_position()
		dstJoints = map(operator.add, recentJoints,joints)
		self.moveToJointPosition(dstJoints,simulate)
	

	def moveToCartesianPosition(self,position,simulate=True):
		if self.manipulator.GetName()=='postument':
			solution = self.kinematicSolver.solveIKPost([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w ],[position.position.x, position.position.y, position.position.z])
		elif self.manipulator.GetName()=='track':
			solution = self.kinematicSolver.solveIKTrack([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w ],[position.position.x, position.position.y, position.position.z])
		else:
			print self.FAILC+"[OpenRAVEIrp6] IK for "+ str(self.manipulator.GetName())+ " unhandled" +self.ENDC
			solution=None
			
		if solution is not None:
			self.moveToJointPosition(solution);
		else:
			print self.FAILC+"[OpenRAVEIrp6] Could not find IK solution"+self.ENDC
	
	def moveRelativeToCartesianPosition(self,x_tran=0,y_tran=0,z_tran=0,x_rot=0,y_rot=0,z_rot=0,simulate=True):
		if self.manipulator.GetName()=='postument':
			solution = self.kinematicSolver.solveRelativeIKPost([x_rot,y_rot,z_rot],[x_tran,y_tran,z_tran])
		elif self.manipulator.GetName()=='track':
			solution = self.kinematicSolver.solveRelativeIKTrack([x_rot,y_rot,z_rot],[x_tran,y_tran,z_tran])
		else:
			print self.FAILC+"[OpenRAVEIrp6] IK for "+ str(self.manipulator.GetName())+ " unhandled" +self.ENDC
			solution=None
		if solution is not None:
			self.moveToJointPosition(solution);
		else:
			print self.FAILC+"[OpenRAVEIrp6] Could not find IK solution"+self.ENDC

	def moveToSynchroPosition(self,simulate=True):
		if self.manipulator.GetName()=='postument':
			pos=[-0.10443037974683544, -1.5476547584053457, 0.012313341484619551, 1.2106388401258297, 4.08203125, 0]
		elif self.manipulator.GetName()=='track':
			pos=[0.0, -0.10443037974683544, -1.5476547584053457, 0.012313341484619551, 1.2106388401258297, 4.08203125, 0]
		self.moveToJointPosition(pos,simulate)
