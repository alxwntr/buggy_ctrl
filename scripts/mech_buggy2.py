#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from gpiozero import Motor
import math, time

class MechBuggy2:
	
	def __init__(self, connex): # connex = ((a,b), (c,d), (e,f), (g,h))
		self._io = []
		for io in connex:
			self._io.append(Motor(io[0], io[1]))
		self.commands = {'w': (1, 1, 1, 1),		\
						 's': (-1, -1, -1, -1),	\
						 'a': (-1, 1, 1, -1),	\
						 'd': (1, -1, -1, 1),	\
						 'q': (-1, 1, -1, 1),	\
						 'e': (1, -1, 1, -1),	\
						 'r': (0, 0, 0, 0)}


	def cmd_handler(self, command, duty):
		if command == ' ':
			self.spin(duty)
		elif command in self.commands.keys():
			levels = self.commands[command]
			self.drive(levels, duty)
	
	
	def drive(self, levels, duty):
		for motor in range(4):
			if levels[motor] < 0:
				self._io[motor].backward(-levels[motor]*duty)
			else:
				self._io[motor].forward(levels[motor]*duty)


	def spin_levels(self, fw, sd, tn):
		f = [fw*x for x in self.commands['w']]
		s = [sd*x for x in self.commands['a']]
		t = [tn*x for x in self.commands['e']]
		return [f[x]+s[x]+t[x] for x in range(4)]


	def spin(self, duty): # Assumes travelling forward at 'duty'
		for deg in range(0,361):
			rad = deg*math.pi/180
			res_sp = 0.5/(abs(math.sin(rad))+abs(math.cos(rad)))
			fw = res_sp*math.cos(rad)
			sd = res_sp*math.sin(rad)
			tn = 0.3
			levels = self.spin_levels(fw, sd, tn)
			self.drive(levels, 1)
			time.sleep(0.005)
		self.drive(self.commands['w'], duty)


	def cleanup(self):
		for motor in range(4):
			self._io[motor].close()

