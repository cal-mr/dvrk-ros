#!/usr/bin/env python

# Adithya Murali 
# 7/3/2014

import roslib
roslib.load_manifest('dvrk_robot')
roslib.load_manifest('tfx')
import tfx
import rospy
import math
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import IPython

frameTransform = tfx.transform([[-1,0,0],[0,-1,0],[0,0,1]])

class rosTeleop:
	def __init__(self, name):
		rospy.init_node(name, anonymous=True)
		self.prevMasterPose = None
		self.currMasterPose = None
		self.currSlavePose = None
		self.scale = 0.2
		self.pub = rospy.Publisher('/dvrk_psm1/set_cartesian_pose', PoseStamped, queue_size=10)
		self.rate = rospy.Rate(500)  # 500 hz
		self.slave_sub = rospy.Subscriber('/dvrk_psm1/joint_position_cartesian', PoseStamped, self.slave_callback)
		self.master_sub = rospy.Subscriber('/dvrk_mtmr/joint_position_cartesian', PoseStamped, self.master_callback)
		self.flagMasterCallback = True
	def master_callback(self, data):
		if self.prevMasterPose is None:
			self.prevMasterPose = tfx.pose(data)
			return
		if self.currMasterPose is None:
			self.currMasterPose = tfx.pose(data)
			return
		if self.flagMasterCallback:
			print 'master callback'
			self.prevMasterPose = tfx.pose(self.currMasterPose, copy = True)
			self.currMasterPose = tfx.pose(data)

	def slave_callback(self, data):
		if self.flagMasterCallback:
			print 'slave callback'
			self.currSlavePose = tfx.pose(data)	

		

	def run(self):
		file = open("data2/poseDeviationROS.txt", "w")
		while not rospy.is_shutdown():
			if self.prevMasterPose is not None and self.currMasterPose is not None and self.currSlavePose is not None:
				self.flagMasterCallback = False
				print 'start'

				p_Master_Pose = tfx.pose(self.prevMasterPose, copy = True)

				c_Master_Pose = tfx.pose(self.currMasterPose, copy = True)

				c_Slave_Pose = tfx.pose(self.currSlavePose, copy = True)

				# IPython.embed()
				print 'end'
				self.flagMasterCallback = True

				delta = p_Master_Pose.as_tf().inverse() * c_Master_Pose

				deltaPositionInSlaveFrame = frameTransform * delta.translation
 
				orientationInSlaveFrame = frameTransform * c_Master_Pose.orientation

				newSlavePose = tfx.pose((self.scale * deltaPositionInSlaveFrame) + c_Slave_Pose.translation, orientationInSlaveFrame)

				trans = newSlavePose.translation.list
				rot = newSlavePose.rotation.tb_angles

				trans1 = c_Slave_Pose.translation.list
				rot1 = c_Slave_Pose.rotation.tb_angles

				# file.write(str(trans[0]) +" "+ str(trans[1]) + " "+ str(trans[2]) + " " + str(rot.yaw_deg) + " "+ str(rot.pitch_deg) + " " + str(rot.roll_deg) + '\n') 

				file.write(str(trans[0]) +" "+ str(trans[1]) + " "+ str(trans[2]) + " " + str(rot.yaw_deg) + " "+ str(rot.pitch_deg) + " " + str(rot.roll_deg) + " " 
					+ str(trans1[0]) +" "+ str(trans1[1]) + " "+ str(trans1[2]) + " " + str(rot1.yaw_deg) + " "+ str(rot1.pitch_deg) + " " + str(rot1.roll_deg) + " ")
				file.write(str(newSlavePose) + " "+ '\n')

				self.rate.sleep()
		file.close()
		pass

if __name__ == '__main__':
    try:
        rosTeleop = rosTeleop("rosTeleOp")
        rosTeleop.run()
    except rospy.ROSInterruptException:
        pass