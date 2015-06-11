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
	
	def __init__(self,env,manipulator,gripper,baseManipulation,kinematicSolver,irpos=None,planner=None,simplifier='OMPL_Simplifier'):
		self.env = env
		self.manipulator = manipulator
		self.gripper = gripper
		self.baseManipulation = baseManipulation
		self.irpos = irpos
		self.kinematicSolver = kinematicSolver
		self.updateManipulatorPosition()
		self.planner=planner
		self.simplifier=simplifier
	#
	#
	# Misc methods
	#
	#
	def isIKSolutionExist(self,position):
		if self.manipulator.GetName()=='postument':
			solution = self.kinematicSolver.solveIKPost([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w ],[position.position.x, position.position.y, position.position.z])
		elif self.manipulator.GetName()=='track':
			solution = self.kinematicSolver.solveIKTrack([position.orientation.x, position.orientation.y, position.orientation.z, position.orientation.w ],[position.position.x, position.position.y, position.position.z])
		else:
			print self.FAILC+"[OpenRAVEIrp6] IK for "+ str(self.manipulator.GetName())+ " unhandled" +self.ENDC
			solution=None
		return solution!=None

	def waitForRobot(self):
		robot = self.manipulator.GetRobot()
		while not robot.GetController().IsDone():
			time.sleep(0.01)	
			
	def updateManipulatorPosition(self):
		robot = self.manipulator.GetRobot()
		
		robot.SetActiveManipulator(self.manipulator.GetName());
		robot.SetActiveDOFs(self.manipulator.GetArmIndices());
		
		if self.irpos!=None:
			robot.SetDOFValues(self.irpos.get_joint_position(),self.manipulator.GetArmIndices())
			robot.SetDOFValues(self.irpos.get_tfg_joint_position(),self.gripper.GetArmIndices())
		else:
			print self.WARNINGC+"[OpenRAVEIrp6] Irpos not set"+self.ENDC

	#
	#
	#Move arm methods
	#
	#
	def moveToJointPosition(self,dstJoints,simulate=True):
		#Check length of joints list
		if len(self.manipulator.GetArmIndices())!=len(dstJoints):
			print self.FAILC+"[OpenRAVEIrp6] Joints length error"+self.ENDC
		robot = self.manipulator.GetRobot()
	
		robot.SetActiveManipulator(self.manipulator.GetName());
		robot.SetActiveDOFs(self.manipulator.GetArmIndices());
		
		#set starting position same as real robot
		if self.irpos!=None:
			robot.SetDOFValues(self.irpos.get_joint_position(),self.manipulator.GetArmIndices())
		
		traj = None
		conf = None
		if self.planner!=None:
			#preparing for planning
			planner = RaveCreatePlanner(self.env, self.planner)
			simplifier = RaveCreatePlanner(self.env, self.simplifier)
			simplifier.InitPlan(robot, Planner.PlannerParameters())
		
			params = Planner.PlannerParameters()
			params.SetRobotActiveJoints(robot)
			params.SetGoalConfig(dstJoints)
			#params.SetExtraParameters('<range>0.02</range>')
			planner.InitPlan(robot, params)

	
			#planning trajectory
			traj = RaveCreateTrajectory(self.env, '')
			result = planner.PlanPath(traj)
			assert result == PlannerStatus.HasSolution
			result = simplifier.PlanPath(traj)
			assert result == PlannerStatus.HasSolution	
			result = planningutils.RetimeTrajectory(traj)
			assert result == PlannerStatus.HasSolution
		
			if simulate:
				robot.GetController().SetPath(traj)
				self.waitForRobot()
			robot.SetDOFValues(dstJoints,self.manipulator.GetArmIndices())
			conf = traj.GetConfigurationSpecification();
		else:
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
				delay = delay+times[i]
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
			self.moveToJointPosition(solution,simulate);
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
			self.moveToJointPosition(solution,simulate);
		else:
			print self.FAILC+"[OpenRAVEIrp6] Could not find IK solution"+self.ENDC

	def moveToSynchroPosition(self,simulate=True):
		if self.manipulator.GetName()=='postument':
			pos=[-0.10443037974683544, -1.5476547584053457, 0.012313341484619551, 1.2106388401258297, 4.08203125, 0]
		elif self.manipulator.GetName()=='track':
			pos=[0.0, -0.10443037974683544, -1.5476547584053457, 0.012313341484619551, 1.2106388401258297, 4.08203125, 0]
		self.moveToJointPosition(pos,simulate)
		self.tfgToJointPosition(0.073,simulate);
		
	#
	#
	#Move gripper methods
	#
	#
	def tfgToJointPosition(self,position,simulate=True):
		robot = self.gripper.GetRobot()
		
		#set starting position same as real robot
		self.updateManipulatorPosition()
		
		robot.SetActiveManipulator(self.gripper.GetName());
		robot.SetActiveDOFs(self.gripper.GetArmIndices());
		
		print 
		traj = None
		conf = None
		try:
			with self.env:
				traj=self.baseManipulation.MoveActiveJoints([position],outputtrajobj=True,execute=simulate)
			self.waitForRobot()
			robot.SetDOFValues([position],self.gripper.GetArmIndices())
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
				joints.append(w[jointGroup.offset]);
				vels.append(w[velGroup.offset]);
				times.append(w[gr_tim.offset])
			time = 0;
			for i in times:
				time = time+i
			#moving
			if len(times)!=0:
				self.irpos.tfg_to_joint_position(position,time)
			else:
				print self.OKBLUEC+"[OpenRAVEIrp6] Manipulator already in position"+self.ENDC
			
	#
	#
	# Force movement methods
	#
	#
	def startForceControl(self,tran_x=False,tran_y=False,tran_z=False,rot_x=False,rot_y=False,rot_z=False,mov_z=0,value=0.0025):
		self.irpos.set_tool_physical_params(10.8, Vector3(0.004, 0.0, 0.156))
		tx=ty=tz=rx=ry=rz=0;
		if tran_x:
			tx=value
		if tran_y:
			ty=value
		if tran_z:
			tz=value
		if rot_x:
			tx=value
		if tran_y:
			ty=value
		if tran_z:
			tz=value
		self.irpos.start_force_controller(Inertia(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), ReciprocalDamping(Vector3(tx, ty, tz), Vector3(rx, ry, rz)), Wrench(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)), Twist(Vector3(0.0, 0.0, mov_z), Vector3(0.0, 0.0, 0.0)))
	
	def stopForceControl(self):
		self.irpos.stop_force_controller()
		robot = self.manipulator.GetRobot()
	#
	#
	#Get position methods
	#
	#
	def getCartesianPosition(self):
		if self.manipulator.GetName()=='postument':
			solution = self.kinematicSolver.solveFKPost()
		elif self.manipulator.GetName()=='track':
			solution = self.kinematicSolver.solveFKTrack()
		else:
			print self.FAILC+"[OpenRAVEIrp6] IK for "+ str(self.manipulator.GetName())+ " unhandled" +self.ENDC
			solution=None
		return solution
	#
	#
	#Get Force  methods
	#
	#
	def getForceReadings(self):
		return self.irpos.get_force_readings()
