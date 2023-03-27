#!/usr/bin/env python2

from __future__ import division
from __future__ import print_function
from __future__ import absolute_import
from asyncore import loop

PKG = 'px4'

import time
import rospy
import numpy as np

from six.moves import xrange
from threading import Thread
from pymavlink import mavutil

## Importing rosmsg helper functions
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mavros_test_common import MavrosTestCommon
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import ParamValue, AttitudeTarget
from geometry_msgs.msg import Quaternion, Vector3, PoseStamped, Twist
# from transforms3d.euler import euler2quat as quaternion_from_euler
# from transforms3d.euler import quat2euler as quaternion_to_euler

## Importing helper functions
import flcTC as flc
from ceilingEffect import thrustCE
from motorThrust import motorThrust
from matplotlib import pyplot as plt


class MavrosOffboardUAVctl(MavrosTestCommon):
	def setUp(self):
		super(MavrosOffboardUAVctl, self).setUp()
		self.xp = 0.0
		self.yp = 0.0
		self.zp = 1.5

		self.x = 0
		self.y = 0
		self.z = 0

		self.radius = 1
		self.thrust = 0.438
		# self.thrust = 0.378
		# # self.thrust = 0.6

		self.initPerch_atti = AttitudeTarget()
		self.initPerch_atti.orientation.x = self.x
		self.initPerch_atti.orientation.y = self.y
		self.initPerch_atti.orientation.z = self.z
		self.initPerch_atti.orientation.w = 1.0
		self.initPerch_atti.thrust = self.thrust

		self.desired_position.pose.position.x = self.xp
		self.desired_position.pose.position.y = self.yp
		self.desired_position.pose.position.z = self.zp

		# self.desired_atti.orientation.x = self.x
		# self.desired_atti.orientation.y = self.y
		# self.desired_atti.orientation.z = self.z
		# self.desired_atti.thrust = self.thrust

		# Send setpoints in separate thread to prevent failsafe
		self.pos = PoseStamped()
		self.pos_setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		
		self.pos_thread = Thread(target=self.send_pos, args=())
		self.pos_thread.daemon = True
		self.pos_thread.start()

		self.att = AttitudeTarget()
		self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)


	def tearDown(self):
		super(MavrosOffboardUAVctl, self).tearDown()

	##  Helper methods
	def send_pos(self):
		rate = rospy.Rate(10)
		self.pos.header = Header()
		self.pos.header.frame_id = "base_footprint"
		
		while not rospy.is_shutdown():
			yaw_deg = 0
			yaw = np.deg2rad(yaw_deg)
			quaternion = quaternion_from_euler(0, 0, yaw)
			self.pos.pose.orientation = Quaternion(*quaternion)
			self.pos.header.stamp = rospy.Time.now()
			self.pos_setpoint_pub.publish(self.pos)
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass
			
	## Function() for sending pos ##
	def is_at_position(self, x, y, z, offset):
		"""offset: meters"""
		rospy.logdebug(
				"current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
						self.local_position.pose.position.x, self.local_position.pose.
						position.y, self.local_position.pose.position.z))

		desired = np.array((x, y, z))
		pos = np.array((self.local_position.pose.position.x,
										self.local_position.pose.position.y,
										self.local_position.pose.position.z))
		return np.linalg.norm(desired - pos) < offset
	

	def reach_position(self, x, y, timeout):
			"""timeout(int): seconds"""
			# set a position setpoint
			self.pos.pose.position.x = self.desired_position.pose.position.x
			self.pos.pose.position.y = self.desired_position.pose.position.y
			self.pos.pose.position.z = self.desired_position.pose.position.z
			rospy.loginfo(
					"attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
					format(self.desired_position.pose.position.x, self.desired_position.pose.position.y, 
									self.desired_position.pose.position.z, self.local_position.pose.position.x,
									self.local_position.pose.position.y,
									self.local_position.pose.position.z))

			# For demo purposes we will lock yaw/heading to north.
			yaw_degrees = 0  # North
			yaw = np.deg2rad(yaw_degrees)
			quaternion = quaternion_from_euler(0, 0, yaw)
			self.pos.pose.orientation = Quaternion(*quaternion)

			# does it reach the position in 'timeout' seconds?
			loop_freq = 2  # Hz
			rate = rospy.Rate(loop_freq)
			reached = False
			for i in xrange(timeout * loop_freq):
					if self.is_at_position(self.pos.pose.position.x,
																	self.pos.pose.position.y,
																	self.pos.pose.position.z, self.radius):
							rospy.loginfo("position reached | seconds: {0} of {1}".format(
									i / loop_freq, timeout))
							reached = True
							break

					try:
							rate.sleep()
					except rospy.ROSException as e:
							self.fail(e)

			self.assertTrue(reached, (
					"took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
					format(self.local_position.pose.position.x,
									self.local_position.pose.position.y,
									self.local_position.pose.position.z, timeout)))



	def send_att(self):
		rate = rospy.Rate(100)
		self.att.body_rate = Vector3()
		self.att.header = Header()
		self.att.header.frame_id = "map"
		self.att.orientation.w = self.initPerch_atti.orientation.w
		self.att.orientation.x = self.initPerch_atti.orientation.x
		self.att.orientation.y = self.initPerch_atti.orientation.y
		self.att.orientation.z = self.initPerch_atti.orientation.z
		print(self.att.orientation)
		self.att.thrust = self.desired_atti.thrust
		self.att.type_mask = 7		# ignoring body rates

		while not rospy.is_shutdown():
			self.att.header.stamp = rospy.Time.now()
			self.att_setpoint_pub.publish(self.att)
			try:
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	def resend_att(self):
		self.att.header = Header()
		self.att.header.frame_id = "map"
		self.att.type_mask = 7		# ignoring body rates
		self.att.orientation.w = 1.0
		self.att.orientation.x = self.initPerch_atti.orientation.x
		self.att.orientation.y = self.initPerch_atti.orientation.y
		self.att.orientation.z = self.initPerch_atti.orientation.z
		self.att.thrust = self.initPerch_atti.thrust
		self.att.header.stamp = rospy.Time.now()
		self.att_setpoint_pub.publish(self.att)
		rospy.loginfo("sending new orientation | x: {0}, y: {1}, z:{2}, thrust:{3}".
									format(self.att.orientation.x, self.att.orientation.y, self.att.orientation.z, self.att.thrust))


	def test_attctrl(self):
		self.wait_for_topics(60)
		self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 10, -1)

		self.log_topic_vars()
		rcl_except = ParamValue(1<<2, 0.0)
		self.set_param("COM_RCL_EXCEPT", rcl_except, 5)
		self.set_mode("OFFBOARD", 5)
		self.set_arm(True, 5)

		global thrustControl
		thrustControl = flc.flc()
		self.desired_atti.thrust = round(thrustControl.update(), 4)

		# thrustSP_list = []
		# time_list = []
		# poseZ = []
		# velZ = []
		# ce_list = []
		# cd_list = []
		# time_start = time.time()

		while not rospy.is_shutdown():
			if (not self.mission_done.data):
				# ## For plotting
				# # thrustSP_list.append(self.initPerch_atti.thrust)
				# thrustSP_list.append(self.local_atti.thrust)
				# time_list.append(time.time()-time_start)
				# poseZ.append(thrustControl.zuav)
				# velZ.append(thrustControl._vel_uav)
				# ce_list.append(thrustControl.thrustCE*9.81)
				# cd_list.append(thrustControl._ceiling_dist)

				self.reach_position(0,0,20)
				if (np.linalg.norm(self.desired_position.pose.position.x-self.local_position.pose.position.x)<=0.03) and (np.linalg.norm(self.desired_position.pose.position.z-self.local_position.pose.position.z)<=0.03):
					# Send setpoints in separate thread to prevent failsafe
					time.sleep(3)
					rospy.loginfo("thrust val: %f", self.initPerch_atti.thrust)
					self.att_thread = Thread(target=self.send_att, args=())
					self.att_thread.daemon = True
					self.att_thread.start()
					while not rospy.is_shutdown():
						if not self.mission_done.data:
							## To insert decision node here for perch pose control
							if (self.desired_atti.orientation.x != 0) or (self.desired_atti.orientation.y != 0) or (self.desired_atti.orientation.z != 0):
								self.initPerch_atti.orientation.x = self.desired_atti.orientation.x
								self.initPerch_atti.orientation.y = self.desired_atti.orientation.y
								self.initPerch_atti.orientation.z = self.desired_atti.orientation.z
								self.initPerch_atti.thrust = self.desired_atti.thrust
								self.resend_att()

								# ## For plotting
								# thrustSP_list.append(self.initPerch_atti.thrust)
								# time_list.append(time.time()-time_start)
								# poseZ.append(thrustControl.zuav)
								# velZ.append(thrustControl._vel_uav)
								# ce_list.append(thrustControl.thrustCE*9.81)
								# cd_list.append(thrustControl._ceiling_dist)

							else:
								self.initPerch_atti.orientation.x = self.x
								self.initPerch_atti.orientation.y = self.y
								self.initPerch_atti.orientation.z = self.z
								self.initPerch_atti.thrust = round(thrustControl.update(),4)
								self.resend_att()

								# ## For plotting
								# thrustSP_list.append(self.initPerch_atti.thrust)
								# time_list.append(time.time()-time_start)
								# poseZ.append(thrustControl.zuav)
								# velZ.append(thrustControl._vel_uav)
								# ce_list.append(thrustControl.thrustCE*9.81)
								# cd_list.append(thrustControl._ceiling_dist)

								# rospy.loginfo("resend_atti \n")
						else: break
			else:
				break

		self.set_mode("AUTO.LOITER", 5)
		t = 3
		while t:
			mins, secs = divmod(t, 60)
			rospy.loginfo("Timer: {:02d}:{:02d}" .format(mins, secs))
			time.sleep(1)
			t -= 1

		self.set_mode("AUTO.LAND", 5)
		self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND, 90, 0)
		self.set_arm(False, 5)
		
		# plt.figure()
		# plt.plot(time_list, thrustSP_list, label="thrustSP")
		# plt.plot(time_list, poseZ, label="poseZ (m)")
		# plt.plot(time_list, velZ, label="velZ (m/s)")
		# plt.plot(time_list, ce_list, label="thrust_ce (N)")
		# plt.plot(time_list, cd_list, label="ceiling_dist (m)")
		# plt.legend(loc='upper left')

		# plt.figure()
		# plt.plot(time_list, thrustSP_list, label="thrustSP")
		
		# plt.legend(loc='upper left')
		# plt.show()


if __name__ == "__main__":
	import rostest
	rospy.init_node('test_node', anonymous=True)
	rostest.rosrun(PKG, 'mavros_offboard_UAVctl_test', MavrosOffboardUAVctl)

