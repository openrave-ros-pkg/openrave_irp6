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
	def __init__(self,env,robot,manageIrpos=True,planner=None,simplifier='OMPL_Simplifier',csn=False):

		irp6Kinematic = Irp6Kinematic(env,robot)
		
		baseManipulation = interfaces.BaseManipulation(robot)
		taskprob = interfaces.TaskManipulation(robot)

		postument=robot.SetActiveManipulator('postument');
		postument_tfg=robot.SetActiveManipulator('postument_tfg');
		track=robot.SetActiveManipulator('track');
		track_tfg=robot.SetActiveManipulator('track_tfg');
		
		robot.SetActiveDOFs(postument.GetArmIndices());
		robot.SetActiveDOFs(postument_tfg.GetArmIndices());
		robot.SetActiveDOFs(track.GetArmIndices());

		accel_limits = robot.GetDOFAccelerationLimits()
		accel_limits[postument.GetArmIndices()] = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		accel_limits[track.GetArmIndices()] = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
		robot.SetDOFAccelerationLimits(accel_limits)
		
		if manageIrpos==True:
			if csn:
				postumentIrpos=IRPOS("IRpOS", "Irp6p", 6,'irp6p_manager')
				trackIrpos=IRPOS("IRpOS", "Irp6ot", 7,'irp6ot_manager')
			else:
				postumentIrpos=IRPOS("IRpOS", "Irp6p", 6)
				trackIrpos=IRPOS("IRpOS", "Irp6ot", 7)
			robot.SetDOFValues(trackIrpos.get_joint_position(),track.GetArmIndices())
			robot.SetDOFValues(postumentIrpos.get_joint_position(),postument.GetArmIndices())
			robot.SetDOFValues(postumentIrpos.get_tfg_joint_position(),postument_tfg.GetArmIndices())
			robot.SetDOFValues(trackIrpos.get_tfg_joint_position(),track_tfg.GetArmIndices())
		else:
			postumentIrpos=None
			trackIrpos=None
			robot.SetDOFValues([0,-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984],track.GetArmIndices())
			robot.SetDOFValues([-0.10063291139240507, -1.5419428654532268, 0.019737556833721442, 1.1335183568246088, 3.658072916666667, -2.7381185214159984],postument.GetArmIndices())
			robot.SetDOFValues([0.72],postument_tfg.GetArmIndices())
			robot.SetDOFValues([0.72],track_tfg.GetArmIndices())
		
		self.postument = Irp6Manipulator(env,postument,postument_tfg,baseManipulation,irp6Kinematic,postumentIrpos,planner,simplifier)
		self.track = Irp6Manipulator(env,track,track_tfg,baseManipulation,irp6Kinematic,trackIrpos,planner,simplifier)
