#!/usr/bin/env python2

from __future__ import division
from asyncore import loop

PKG = 'px4'

import time
import rospy

from six.moves import xrange
from threading import Thread
from pymavlink import mavutil
from std_msgs.msg import Header
from mavros_msgs.msg import ParamValue, AttitudeTarget
from mavros_test_common import MavrosTestCommon
from geometry_msgs.msg import Quaternion, Vector3
# # from tf.transformations import quaternion_from_euler
# from transforms3d.euler import euler2quat as quaternion_from_euler
# from transforms3d.euler import quat2euler as quaternion_to_euler


class MavrosOffboardAttctlTest(MavrosTestCommon):
	def setUp(self):
		super(MavrosOffboardAttctlTest, self).setUp()
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.thrust = 0.378
		# self.thrust = 0.6
		self.desired_atti.orientation.x = self.x
		self.desired_atti.orientation.y = self.y
		self.desired_atti.orientation.z = self.z
		self.desired_atti.thrust = self.thrust

		self.att = AttitudeTarget()
		
		self.att_setpoint_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=100)

		self.att_thread = Thread(target=self.send_att, args=())
		self.att_thread.daemon = True
		self.att_thread.start()

	def tearDown(self):
		super(MavrosOffboardAttctlTest, self).tearDown()

	def send_att(self):
		rate = rospy.Rate(100)
		self.att.body_rate = Vector3()
		self.att.header = Header()
		self.att.header.frame_id = "map"
		self.att.orientation.w = self.desired_atti.orientation.w
		self.att.orientation.x = self.desired_atti.orientation.x
		self.att.orientation.y = self.desired_atti.orientation.y
		self.att.orientation.z = self.desired_atti.orientation.z
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
		self.att.orientation.w = self.desired_atti.orientation.w
		self.att.orientation.x = self.desired_atti.orientation.x
		self.att.orientation.y = self.desired_atti.orientation.y
		self.att.orientation.z = self.desired_atti.orientation.z
		self.att.thrust = self.desired_atti.thrust
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

		while not rospy.is_shutdown():
			if not self.mission_done.data:
				self.resend_att()
				# rospy.loginfo("resend_atti \n")
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



if __name__ == "__main__":
	import rostest
	rospy.init_node('test_node', anonymous=True)
	rostest.rosrun(PKG, 'mavros_offboard_attctl_test', MavrosOffboardAttctlTest)

