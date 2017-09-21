#!/usr/bin/env python
#/****************************************************************************
# Frobit lidar obstacle detect
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

from math import pi, sin, cos, atan2

class obstacle_detect():
	def __init__(self, warn_ahead, warn_lateral, alarm_ahead, alarm_lateral, minimum_range):
		self.warn_ahead = warn_ahead
		self.warn_lateral = warn_lateral
		self.alarm_ahead = alarm_ahead
		self.alarm_lateral = alarm_lateral
		self.minimum_range = minimum_range
		self.params_set = False

	def set_params(self, angular_res, num_ranges):

		# calculate additional parameters
		self.center = int(num_ranges/2.0)
		deg90 = int(pi/(2.0*angular_res))

		self.begin = self.center - deg90		
		self.end = self.center + deg90 

		angle_warn = atan2 (self.warn_lateral, self.warn_ahead)
		angle_alarm = atan2 (self.alarm_lateral, self.alarm_ahead)

		steps_warn = int(angle_warn/angular_res)
		warn_r = self.center - steps_warn
		warn_l = self.center + steps_warn

		steps_alarm = int(angle_alarm/angular_res)
		alarm_l = self.center + steps_alarm
		alarm_r = self.center - steps_alarm

		# build list of warning ranges
		# (calculating the length of the hypothenuse (ray) length at a given angle)
		# (the lenght of the leg is defined to be the dist_warn parameter)  
		self.lidar_warn_r = []
		for i in range(self.begin, warn_r):
			self.lidar_warn_r.append(self.warn_lateral / cos((i-self.begin) * angular_res))	
		for i in range(warn_r, self.center):
			self.lidar_warn_r.append(self.warn_ahead / cos((self.center-i) * angular_res))	

		self.lidar_warn_l = []
		for i in range(self.center, warn_l):
			self.lidar_warn_l.append(self.warn_ahead / cos((self.center-i) * angular_res))	
		for i in range(warn_l, self.end):
			self.lidar_warn_l.append(self.warn_lateral / cos((self.end - i) * angular_res))	

		self.lidar_alarm_r = []
		for i in range(self.begin, alarm_r):
			self.lidar_alarm_r.append(self.alarm_lateral / cos((i - self.begin)* angular_res))	
		for i in range(alarm_r, self.center):
			self.lidar_alarm_r.append(self.alarm_ahead / cos((self.center - i) * angular_res))	

		self.lidar_alarm_l = []
		for i in range(self.center, alarm_l):
			self.lidar_alarm_l.append(self.alarm_ahead / cos((self.center - i) * angular_res))	
		for i in range(alarm_l, self.end):
			self.lidar_alarm_l.append(self.alarm_lateral / cos((self.end - i)* angular_res))	

		self.params_set = True

	def new_scan(self, scan):
		if self.params_set == True:
			warn_r_breach = False
			alarm_r_breach = False
			warn_l_breach = False
			alarm_l_breach = False

			for i in range (self.begin, self.center):
				if scan[i] > self.minimum_range:
					if scan[i] < self.lidar_warn_r[i-self.begin]:
						warn_r_breach = True
					if scan[i] < self.lidar_alarm_r[i-self.begin]:
						alarm_r_breach = True

			for i in range (self.center, self.end):
				if scan[i] > self.minimum_range:
					if scan[i] < self.lidar_warn_l[i-self.center]:
						warn_l_breach = True
					if scan[i] < self.lidar_alarm_l[i-self.center]:
						alarm_l_breach = True

			if alarm_r_breach:
				status_r = 2
			elif warn_r_breach:
				status_r = 1
			else:
				status_r = 0
				
			if alarm_l_breach:
				status_l = 2
			elif warn_l_breach:
				status_l = 1
			else:
				status_l = 0
				
			return [status_l, status_r]

	def export_params (self, file_name):
		# write header file
		file = open('%s.h' % file_name, 'w')
		file.write('/* Data generated using the script generate_lidar_params.py */\n/* DO NOT CHANGE! */\n\n')

		file.write('#define lidar_begin %d\n' % self.begin)
		file.write('#define lidar_center %d\n' % self.center)
		file.write('#define lidar_end %d\n\n' % self.end)

		file.write('extern int lidar_warn_r[%d];\n' % len(self.lidar_warn_r))
		file.write('extern int lidar_warn_l[%d];\n' % len(self.lidar_warn_l))
		file.write('extern int lidar_alarm_r[%d];\n' % len(self.lidar_alarm_r))
		file.write('extern int lidar_alarm_l[%d];\n' % len(self.lidar_alarm_l))
		file.close()

		# write source file
		file = open('%s.c' % file_name, 'w')
		file.write('/* Data generated using the script generate_lidar_params.py */\n/* DO NOT CHANGE! */\n\n')

		file.write('int lidar_warn_r[%d] = {' % len(self.lidar_warn_r))
		for i in range(len(self.lidar_warn_r)):
			if i != 0:
				file.write(', ')
			file.write('%d' % (self.lidar_warn_r[i]*1000))
		file.write('};\n\n')

		file.write('int lidar_warn_l[%d] = {' % len(self.lidar_warn_l))
		for i in range(len(self.lidar_warn_l)):
			if i != 0:
				file.write(', ')
			file.write('%d' % (self.lidar_warn_l[i]*1000))
		file.write('};\n\n')

		file.write('int lidar_alarm_r[%d] = {' % len(self.lidar_alarm_r))
		for i in range(len(self.lidar_alarm_r)):
			if i != 0:
				file.write(', ')
			file.write('%d' % (self.lidar_alarm_r[i]*1000))
		file.write('};\n\n')

		file.write('int lidar_alarm_l[%d] = {' % len(self.lidar_alarm_l))
		for i in range(len(self.lidar_alarm_l)):
			if i != 0:
				file.write(', ')
			file.write('%d' % (self.lidar_alarm_l[i]*1000))
		file.write('};\n\n')
		file.close()

	def update(self):
		pass

