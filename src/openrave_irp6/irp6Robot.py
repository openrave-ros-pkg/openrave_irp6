#!/usr/bin/env python

import rospy
import tf
import actionlib

from irpos import *
from operator import *

try:
	from irp6kinematic import *
except ImportError:
	print '\033[91m'+"[OpenRAVEIrp6] A openrave'a source'owales ciulu?"+'\033[0m'
	print '\033[91m'+"[OpenRAVEIrp6] source [openrave_workspace_dir]/underlay_isolated/install_isolated/share/openrave-0.9/openrave.bash"+'\033[0m'

from irp6Manipulator import *

import time
import openravepy
if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

class Irp6Robot:
	def __init__(self,env,robot,manageIrpos=True,planner=None,simplifier='OMPL_Simplifier'):

		irp6Kinematic = Irp6Kinematic(env,robot)
		
		baseManipulation = interfaces.BaseManipulation(robot)
		taskprob = interfaces.TaskManipulation(robot)

		postument=robot.SetActiveManipulator('postument');
		track=robot.SetActiveManipulator('track');
		
		robot.SetActiveDOFs(postument.GetArmIndices());
		robot.SetActiveDOFs(track.GetArmIndices());

		accel_limits = robot.GetDOFAccelerationLimits()
		accel_limits[postument.GetArmIndices()] = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		accel_limits[track.GetArmIndices()] = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		robot.SetDOFAccelerationLimits(accel_limits)
		
		if manageIrpos==True:
			#postumentIrpos=IRPOS("IRpOS", "Irp6p", 6,'irp6p_manager')
			#trackIrpos=IRPOS("IRpOS", "Irp6ot", 7,'irp6ot_manager')
			postumentIrpos=IRPOS("IRpOS", "Irp6p", 6)
			trackIrpos=IRPOS("IRpOS", "Irp6ot", 7)
			robot.SetDOFValues(trackIrpos.get_joint_position(),track.GetArmIndices())
			robot.SetDOFValues(postumentIrpos.get_joint_position(),postument.GetArmIndices())	
		else:
			robot.SetDOFValues([0,-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984],track.GetArmIndices())
			robot.SetDOFValues([-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984],postument.GetArmIndices())
		
		self.postument = Irp6Manipulator(env,postument,baseManipulation,irp6Kinematic,postumentIrpos,planner)
		self.track = Irp6Manipulator(env,track,baseManipulation,irp6Kinematic,trackIrpos,planner)
		
	def move(self):
		track = self.robot.SetActiveManipulator('track');
		#try:
		print "Pos 1"
		with self.env:
			print "jak powinno"
			sol1 = [0,0, -1.54, 0, 0, 4.71, -1.57]
			traj=self.baseManipulation.MoveManipulator(sol1,outputtrajobj=True,execute=True)
		while not self.robot.GetController().IsDone():
			time.sleep(0.01)	
		postument = self.robot.SetActiveManipulator('postument');
		with self.env:
			print "jak jest"
			sol2 = self.irp6Kinematic.solveIKPost([-0.000379723678393, -0.999880665371, 0.00120047894583, 0.0153970671608],[0.865904485399, -0.00059724452807, 1.12842285353])
			traj=self.baseManipulation.MoveManipulator(sol2,outputtrajobj=True,execute=True)
		while not self.robot.GetController().IsDone():
			time.sleep(0.01)
