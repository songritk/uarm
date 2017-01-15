#!/usr/bin/env python

# Released by rdb under the Unlicense (unlicense.org)
# Based on information from:
# https://www.kernel.org/doc/Documentation/input/joystick-api.txt
# 
# Modified by Songrit Kitisriworapan
# Rev1.0 : 28 Dec 2015
#      - Support USB Joystick + Analog Control
#      - Smooth moving (remove outliner)
#      - Slow control key 
#      - Read analog position 

#from UArmForPython.uarm_python import Uarm
import pyuarm
import time
import sys
import signal
from pixy import *
from ctypes import *
import os, struct, array
from fcntl import ioctl
from pyfirmata import Arduino, util
import pygame
import sys, traceback
from pykalman import KalmanFilter

K_BASED='x'
K_UP1='y'
K_UP2='trottle'
K_Y='rz'
K_SLOW='tr'
K_POS='c'

board = Arduino('/dev/ttyAR')
clock = pygame.time.Clock()
it = util.Iterator(board)
it.start()
board.analog[0].enable_reporting()
board.analog[1].enable_reporting()
board.analog[2].enable_reporting()
board.analog[3].enable_reporting()

#pin10 K_UP2
#pin11 K_UP1
#pin12 K_BASED

pin4 = board.get_pin('d:4:o')
pin6 = board.get_pin('d:6:o')
pin9 = board.get_pin('d:9:s')
pin10 = board.get_pin('d:10:s')
pin11 = board.get_pin('d:11:s')
pin12 = board.get_pin('d:12:s')
pin13 = board.get_pin('d:13:s')

pin14 = board.get_pin('a:0:o')
pin15 = board.get_pin('a:1:o')
pin16 = board.get_pin('a:2:o')
pin17 = board.get_pin('a:3:o')


base_state=1
base2_state=1
base4_state=0
slowrate=1

current_based=90
current_end=90
current_z=120
current_rz=90

pin9.write(current_end)
pin10.write(current_rz)
pin11.write(current_z)
pin12.write(current_based)

offset_min_y=0
offset_min_z=0
offset_min_rz=0

offset_max_y=1
offset_max_z=1
offset_max_rz=1

# Iterate over the joystick devices.
print('Available devices:')

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}

axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js0'
#print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('c', ['\0'] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tostring()
#print('Device name: %s' % js_name)

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

print '%d axes found: %s' % (num_axes, ', '.join(axis_map))
print '%d buttons found: %s' % (num_buttons, ', '.join(button_map))

# Main event loop

class UARMServo:
	def Calibration(self):
#           pin11.write(0)
           time.sleep(5.0)
           offset_min_z=pin16.read()
#           pin11.write(180)
           time.sleep(5.0)
           offset_max_z=pin16.read()
           print "offset Z min %s"%(offset_min_z)
           print "offset Z max %s"%(offset_max_z)
          
	def AnalogToDegree(rz,y,z):
	 print 'RZ%s,Y%s,Z%s'%(pin14.read(),pin15.read(),pin16.read()) 
	def Position(self):
	  print "OK"

while True:
    evbuf = jsdev.read(8)
    if evbuf:
        times, value, type, number = struct.unpack('IhBB', evbuf)

#        if type & 0x80:
#             print "(initial)",

        if type & 0x01:
	 button = button_map[number]
	 if button:
# A button
                print "%s pressed1" % (button)
                button_states[button] = value
                if (button is 'trigger' and value):
		  s=UARMServo()
                  s.Calibration()
                if (button is K_POS and value):
                  print "UP1 Z(%s) pin11 %s"%(current_z,pin11.read())
                  print "UP2 RZ(%s) pin10 %s"%(current_rz,pin10.read())
                  print "BASED X pin12 %s"%(pin12.read())
                  
                if (button is K_SLOW and value):
                  slowrate=0.1
                if (button is K_SLOW and not value):
                  slowrate=1
                if ( button is 'tl2' and value):
                  board.exit()
		  sys.exit(0)

#                if ( button is 'base4' and (not base4_state) and value):
#		    pin11.write(0)
#		    time.sleep(1.0)
#                    offset_min_z=pin16.read()
#		    pin11.write(180)
#		    time.sleep(1.0)
#                    offset_max_z=pin16.read()
#		    print "offset Z min %s"%(offset_min_z)
#		    print "offset Z max %s"%(offset_max_z)
#
#                    pin10.write(90)
#                    pin11.write(90)
#                    pin12.write(45)
#                    pin13.write(100)
#                    base2_state=0

               	base4_state=value
		if (button is 'base' and value):
                 print 'base %s' %(value)
                 if (base_state is 1):
                   pin6.write(1)
                   base_state=0
                 else:
                   pin6.write(0)
                   base_state=1
	 else:
		print "%s released" % (button)
        if type & 0x02:
            axis = axis_map[number]
            if axis:
                fvalue = value*(slowrate) / 32767.0
                axis_states[axis] = fvalue
                button = button_map[number]
		if axis==K_BASED:
                 current_based=current_based-(fvalue*5)
                 if (current_based > 180):
                  current_based=180
                 if (current_based < 0):
                  current_based=0
                 pin12.write(current_based)
                 print "pin12 %s: %.3f" % (axis, current_based)

		if axis==K_Y:
                 current_end=current_end-(fvalue*10)
                 if (current_end > 180):
                  current_end=180
                 if (current_end < 0):
                  current_end=0
                 pin9.write(current_end)
                 print "pin9 %s: %.3f" % (axis, current_end)

		if axis==K_UP2:
		 current_rz=current_rz-fvalue
                 if (current_rz > 145):
	          current_rz=145
                 if (current_rz < 0):
	          current_rz=0
		 pin10.write(current_rz)
              	 print "pin10 %s: %.3f" % (axis, current_rz)
		if axis==K_UP1:
		 current_z=current_z-(fvalue*-1)
		 if (current_z > 180):
		  current_z=180
		 if (current_z <0):
		  current_z=0
		 pin11.write(current_z)
                 print "pin11: %s %.3f"%(pin11.read(),current_z)
