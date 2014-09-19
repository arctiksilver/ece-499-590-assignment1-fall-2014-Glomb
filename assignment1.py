#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */

# ******************************************************

# Chris Glomb
# In Class Assignment 1
# 9/19/14

# this code uses robot-view-serial.py as the base

# ******************************************************

# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

import actuator_sim as ser
#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------

# Add imports here




#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0

ref.ref[0] = 0
ref.ref[1] = 0

# Problem 3
def checksum(packet):
	j=0
	c=0
	l=5
	for byte in packet:
		if j == 3:
			l = byte
		if j > 1 and j < l+3:
			c += byte
		if j == (l + 3):
			c = ~c & 0xFF
			packet[j] = c
		j += 1
	return packet

# Problem 4
def pack(motor,d,v):
	# motor - left = 1, right = 0
	# direction - forward = 1, reverse = 0
	#    0xFF, 0xFF, ID  , Length, Instruction   , Speed(L),        Speed(H)       , Checksum 
	p = [255 , 255 ,motor,   4   ,     0x20      ,  v&0xFF , ((v>>8)&0xFF)|(d<<2),     0    ]
	return checksum(p)

# Problem 5 - loop at 20Hz
def simSleepTwentyHz(duration):
	i = 0
	[status, framesize] = t.get(tim, wait=True, last=True)
	next = tim.sim[0] + 0.05
	while i < duration:
		[status, framesize] = t.get(tim, wait=True, last=True)
		while tim.sim[0] < next:
			[status, framesize] = t.get(tim, wait=True, last=True)
			if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
				pass
			else:
				raise ach.AchException( v.result_string(status) )
		next = tim.sim[0] + 0.05
		[status, framesize] = t.get(tim, wait=True, last=True)
		i += 1

# Problem 6.a
def clockwiseSpin():
	ref = dd.H_REF()
	buff = pack(1,1,1023)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,0,1023)
	ref = ser.serial_sim(r,ref,buff)
	
	simSleepTwentyHz(650)
	
	buff = pack(1,1,0)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,0,0)
	ref = ser.serial_sim(r,ref,buff)

# Problem 6.b
def counterClockSpin():
	ref = dd.H_REF()
	buff = pack(1,0,511)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,1,511)
	ref = ser.serial_sim(r,ref,buff)
	
	simSleepTwentyHz(1300)
	
	buff = pack(1,1,0)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,0,0)
	ref = ser.serial_sim(r,ref,buff)

# Function for problem 6.c and 6.d
def turnNinty(direction):
	# Direction - Left = 1, Right = 0
	ref = dd.H_REF()
	if direction: # left turn
		i = 1
		while i < 10:
			buff = pack(1,1,1023/i)
			ref = ser.serial_sim(r,ref,buff)
			buff = pack(0,1,1023)
			ref = ser.serial_sim(r,ref,buff)
			simSleepTwentyHz(11)
			i += 1
		i = 7
		while i > 0:
			buff = pack(1,1,1023/i)
			ref = ser.serial_sim(r,ref,buff)
			buff = pack(0,1,1023)
			ref = ser.serial_sim(r,ref,buff)
			simSleepTwentyHz(3)
			i -= 1
	else: # right turn
		i = 1
		while i < 10:
			buff = pack(1,1,1023)
			ref = ser.serial_sim(r,ref,buff)
			buff = pack(0,1,1023/i)
			ref = ser.serial_sim(r,ref,buff)
			simSleepTwentyHz(11)
			i += 1
		i = 7
		while i > 0:
			buff = pack(1,1,1023)
			ref = ser.serial_sim(r,ref,buff)
			buff = pack(0,1,1023/i)
			ref = ser.serial_sim(r,ref,buff)
			simSleepTwentyHz(3)
			i -= 1
	# forward
	buff = pack(1,1,1023)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,1,1023)
	ref = ser.serial_sim(r,ref,buff)

def square(direction):
	# direction - Counterclockwise = 1, Clockwise = 0
	ref = dd.H_REF()
	buff = pack(1,1,1023)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,1,1023)
	ref = ser.serial_sim(r,ref,buff)
	simSleepTwentyHz(140)
	c = 0
	while c < 15:
		turnNinty(direction)
		simSleepTwentyHz(140)
		c += 1

	buff = pack(1,1,0)
	ref = ser.serial_sim(r,ref,buff)
	buff = pack(0,1,0)
	ref = ser.serial_sim(r,ref,buff)

# Uncomment to run the code for each problem

'''
# Problem 3
p = []
checksum(p)
'''

'''
# Problem 4
pack(1,1,1023)
'''

'''
# Problem 5
simSleepTwentyHz(20)
'''

'''
# Problem 6.a
clockwiseSpin()
'''

'''
# Problem 6.b
counterClockSpin()
'''

'''
# Problem 6.c - Counterclockwise
square(1) 
'''

'''
# Problem 6.d - Clockwise
square(0)
'''
