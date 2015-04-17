#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
from operator import *

import irp6kinematic

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

#MAIN

if __name__ == '__main__':
	postument = IRPOS("IRpOS", "Irp6p", 6)
	track = IRPOS("IRpOS", "Irp6ot", 7)

	env = Environment()
	#env.SetViewer('qtcoin')
	#env.Load('data/irp6both.env.xml')
	EPS = 1e-10

	#robot = env.GetRobots()[0]
 
	module = RaveCreateModule(env, 'urdf')
	name = module.SendCommand('load package://irp6_description/robots/irp6pboth.urdf.xacro package://irp6_description/robots/irp6pboth.srdf')
	robot = env.GetRobot(name)
	conveyor = env.ReadKinBodyXMLFile('data/conveyor.kinbody.xml')
	env.Add(conveyor)
 
	basemanip = interfaces.BaseManipulation(robot)
	taskprob = interfaces.TaskManipulation(robot)
    
	postument_o=robot.SetActiveManipulator('postument');
	track_o=robot.SetActiveManipulator('track');
    
	robot.SetActiveDOFs(postument_o.GetArmIndices());
	robot.SetActiveDOFs(track_o.GetArmIndices());
	
	print 'openrave ok'
	
	time.sleep(5)

	track.move_to_synchro_position(10.0)
	postument.move_to_synchro_position(10.0)
	
	print "OpenRAVE move on!"
	robot.SetActiveManipulator('postument');
	robot.SetDOFValues(postument.get_joint_position(),postument_o.GetArmIndices())
	with env:
		sol = [-1.28227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624]
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=False)
	while not robot.GetController().IsDone():
		time.sleep(0.01)	
	robot.SetDOFValues(sol,postument_o.GetArmIndices())

	postument.move_to_joint_position([-1.28227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624], 3.0)

	"""print "IRPOS postument move on!"
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
	goal.trajectory.points.append(JointTrajectoryPoint([-1.38227752354415, -1.541861095576026, 5.504115800705756e-05, 1.0007174886590251, 4.754815971689398, -1.91731301362624], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], [], [], rospy.Duration(3.0)))
	goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(2.2)
	client.send_goal(goal)
	client.wait_for_result()
	command_result = client.get_result()"""



	print "OpenRAVE move on!"
	track_o=robot.SetActiveManipulator('track');
	robot.SetDOFValues(track.get_joint_position(),track_o.GetArmIndices())
	print "position: " + str(pos)
	with env:
		sol = irp6kinematic.solveIKTrack(env,[-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.60904485399, 1.30059724452807, 1.20842285353])
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=False)
	while not robot.GetController().IsDone():
		time.sleep(0.01)	
	robot.SetDOFValues(sol,track_o.GetArmIndices())
	
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
	accs = []
	for i in range (0,traj.GetNumWaypoints()):
		w=traj.GetWaypoint(i);		
		joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5]]);
		vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5]]);
	points = []
	time=1;
	for i in range(1,len(joints)):
		time = time+ (joints[i][0]-joints[i-1][0]) / (  vels[i-1][0]+  (vels[i][0]-vels[i-1][0])/2 )
		if (joints[i][0]-joints[i-1][0]) / (  vels[i-1][0]+  (vels[i][0]-vels[i-1][0])/2 ) < 1.0:
			continue
		print str(time)
		points.append( JointTrajectoryPoint(joints[i], vels[i], [], [], rospy.Duration(time)) )
	track.move_along_joint_trajectory(points)
	#track.move_to_joint_position([0.1, 0.0, -1.5418065817051163, 0.0, 1.5, 1.57, -1.57], 3.0)







	print "OpenRAVE move on!"
	track_o=robot.SetActiveManipulator('track');
	robot.SetDOFValues(track.get_joint_position(),track_o.GetArmIndices())
	with env:
		sol = irp6kinematic.solveIKTrack(env,[-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.00904485399, 1.30059724452807, 1.20842285353])
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=False)
	while not robot.GetController().IsDone():
		time.sleep(0.01)	
	robot.SetDOFValues(sol,track_o.GetArmIndices())
	
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
	accs = []
	for i in range (0,traj.GetNumWaypoints()):
		w=traj.GetWaypoint(i);		
		joints.append([w[jointGroup.offset],w[jointGroup.offset+1],w[jointGroup.offset+2],w[jointGroup.offset+3],w[jointGroup.offset+4],w[jointGroup.offset+5]]);
		vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5]]);
	points = []
	time=1;
	for i in range(1,len(joints)):
		time = time+ (joints[i][0]-joints[i-1][0]) / (  vels[i-1][0]+  (vels[i][0]-vels[i-1][0])/2 )
		if (joints[i][0]-joints[i-1][0]) / (  vels[i-1][0]+  (vels[i][0]-vels[i-1][0])/2 ) < 1.0:
			continue
		print str(time)
		points.append( JointTrajectoryPoint(joints[i], vels[i], [], [], rospy.Duration(time)) )
	track.move_along_joint_trajectory(points)






	
	"""print "OpenRAVE, give me your powers!"
	postument = robot.SetActiveManipulator('postument');
	with env:
		sol = [-1.4, -1.54, 0, 0.7, 3.75, -1.9]
		traj=basemanip.MoveManipulator(sol,outputtrajobj=True,execute=True)
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
		vels.append([w[velGroup.offset],w[velGroup.offset+1],w[velGroup.offset+2],w[velGroup.offset+3],w[velGroup.offset+4],w[velGroup.offset+5]]);"""
	
	"""print "Go IRPOS!"
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
	for i in range(1,len(joints)):
		print joints[i]
		print vels[i]
		for j in range(0,len(vels)):
			(vels[i])[j]=(vels[i])[j]/10;
		print vels[i]
		goal.trajectory.points.append(JointTrajectoryPoint(joints[i],vels[i], [], [], rospy.Duration(i*5.0)))
	goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(2.2)
	client.send_goal(goal)
	client.wait_for_result()
	command_result = client.get_result()

	conmanSwitch([], ['Irp6pmSplineTrajectoryGeneratorJoint'], True)"""

