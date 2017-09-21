#!/usr/bin/env python
#/***********-*****************************************************************
# Frobit lidar obstacle node 
# Copyright (c) 2015-2017, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
LaserScan tutorial:
http://wiki.ros.org/laser_pipeline/Tutorials/IntroductionToWorkingWithLaserScannerData

Revision
2015-09-17 KJ First version
2017-09-21 KJ Implemented a better algorithm
"""

import rospy
from sensor_msgs.msg import LaserScan
from msgs.msg import IntArrayStamped
from obstacle_detect import obstacle_detect
from math import pi

node_name = 'obstacle'
	
class ros_node():
	def __init__(self):
		self.update_rate = 20 # [Hz]
		self.scans_skipped_cnt = 0
		self.first_scan = True

		# read parameters
		self.ahead_warn = rospy.get_param("~ahead_threshold_warning", 3.0) # [m]
		self.lateral_warn = rospy.get_param("~lateral_threshold_warning", 0.5) # [m]
		self.ahead_alarm = rospy.get_param("~ahead_threshold_alarm", 1.5) # [m]
		self.lateral_alarm = rospy.get_param("~lateral_threshold_alarm", 0.5) # [m]
		self.ang_res = rospy.get_param("~angular_resolution", 1.0) * pi/180.0 # [rad]
		self.min_range = rospy.get_param("~minimum_range", 0.05) # [m]
		self.scans_skip = rospy.get_param("~scans_skip", 0) # [m]

		# initialize wall finding algorithm
		self.od = obstacle_detect(self.ahead_warn, self.lateral_warn, self.ahead_alarm, self.lateral_alarm, self.min_range)

		# get topic names
		scan_topic = rospy.get_param("~scan_sub", "/base_scan")
		obstacle_topic = rospy.get_param("~obstacle_pub", "/fmKnowledge/obstacle")

		# setup wall pose publish topic
		self.obstacle_msg = IntArrayStamped()
		self.obstacle_msg.data = [0,0]
		self.obstacle_pub = rospy.Publisher(obstacle_topic, IntArrayStamped, queue_size=1)

		# setup subscription topic callbacks
		rospy.Subscriber(scan_topic, LaserScan, self.on_scan_topic)

		# sall updater function
		self.r = rospy.Rate(self.update_rate)
		self.updater()

	def on_scan_topic(self, msg):
		if self.scans_skipped_cnt == self.scans_skip:
			if self.first_scan == True:
				self.first_scan = False
				#print msg.angle_min*180/pi, msg.angle_max*180/pi, msg.angle_increment*180/pi, msg.range_min, msg.range_max, len(msg.ranges)

				self.od.set_params(self.ang_res, len(msg.ranges))
			self.scans_skipped_cnt = 0
			self.obstacle_msg.data = self.od.new_scan(msg.ranges)
			self.publish_obstacle_message()
		else:
			self.scans_skipped_cnt += 1

	def publish_obstacle_message(self):
		self.obstacle_msg.header.stamp = rospy.get_rostime()
		self.obstacle_pub.publish(self.obstacle_msg)

	def updater(self):
		while not rospy.is_shutdown():
			self.od.update()
			self.r.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node(node_name)

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = ros_node()
    except rospy.ROSInterruptException: pass


