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

import os, struct, array
from fcntl import ioctl
from pyfirmata import Arduino, util
import pygame
import sys, traceback
board = Arduino('/dev/ttyUARM')
#board = Arduino('/dev/ttyUARM_W')
clock = pygame.time.Clock()
it = util.Iterator(board)
it.start()
board.analog[0].enable_reporting()
board.analog[1].enable_reporting()
board.analog[2].enable_reporting()
board.analog[3].enable_reporting()

pin4 = board.get_pin('d:4:o')
pin6 = board.get_pin('d:6:o')
pin9 = board.get_pin('d:9:o')
pin10 = board.get_pin('d:10:s')
pin11 = board.get_pin('d:11:s')
pin12 = board.get_pin('d:12:s')
pin13 = board.get_pin('d:13:s')

pin14 = board.get_pin('a:0:o')
pin15 = board.get_pin('a:1:o')
pin16 = board.get_pin('a:2:o')
pin17 = board.get_pin('a:3:o')

pin9.write(0)
pin10.write(45)
pin11.write(90)
pin12.write(45)
pin13.write(90)

base_state=1
base2_state=1
base4_state=0
slowrate=1

current_y=45
current_z=90
current_rz=90

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

#print '%d axes found: %s' % (num_axes, ', '.join(axis_map))
#print '%d buttons found: %s' % (num_buttons, ', '.join(button_map))

# Main event loop
while True:
    evbuf = jsdev.read(8)
    if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)

#        if type & 0x80:
#             print "(initial)",

        if type & 0x01:
            button = button_map[number]
            if button:
                print "%s pressed" % (button)
                button_states[button] = value
                if (button is 'pinkie' and value):
                  slowrate=0.1
                if (button is 'pinkie' and not value):
                  slowrate=1
                if ( button is 'base3' and value):
                  board.exit()
		  sys.exit(0)

                if ( button is 'base4' and (not base4_state) and value):
#		    print ("Press") 
		    pin10.write(90)
                    pin11.write(100)
                    pin12.write(45)
                    pin13.write(100)
		    print "pin14 %s"%(pin14.read())
                    print "pin15 %s"%(pin15.read())
                    print "pin16 %s"%(pin16.read())
                    print "pin17 %s"%(pin17.read())
               	base4_state=value
		if (button is 'base' and value):
                 print 'base %s' %(value)
                 if (base_state is 1):
                   pin6.write(1)
                   base_state=0
                 else:
                   pin6.write(0)
                   base_state=1
		if (button is 'base2' and value):
#                 print 'base2 %s' %(value)
                 if (base2_state is 1):
                   pin9.write(1)
                   base2_state=0
                 else:
                   pin9.write(0)
                   base2_state=1
		    
#            else:
#                    print "%s released" % (button)
        if type & 0x02:
            axis = axis_map[number]
            if axis:
                fvalue = value*(slowrate) / 32767.0
                axis_states[axis] = fvalue
		if axis=='x':
		   pin10.write(90-fvalue*90)
                   print "x14 : %s"%(pin14.read())
                button = button_map[number]
	        if axis=='y':
                 if (current_y > 90):
                  current_y=90
                 else:
                  current_y=current_y+(fvalue)
                 if (current_y < 0):
                  current_y=0
#                 print "%s: %.3f=%.3f*%.3f" % (axis, current_y,fvalue,slowrate)
                 pin12.write(current_y)		
		 print "y15 : %s"%(pin15.read())
	
		if axis=='rz':
		 current_rz=current_rz-fvalue
                 if (current_rz > 145):
	          current_rz=145
                 if (current_rz < 0):
	          current_rz=0
		 pin13.write(current_rz)
		 print "rz14 : %s"%(pin14.read())
#              	 print "%s: %.3f" % (axis, current_rz)
		if axis=='z':
		 current_z=current_z-fvalue
		 if (current_z > 180):
		  current_z=180
		 if (current_z <0):
		  current_z=0
		 pin11.write(current_z)
                 print "z: %s %s"%(pin16.read(),pin17.read())
		 print "%s: %.3f" %(axis,current_z)
		 clock.tick(30)
