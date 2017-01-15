#!/usr/bin/env python
#
# begin license header
#
# This file is part of Pixy CMUcam5 or "Pixy" for short
#
# All Pixy source code is provided under the terms of the
# GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
# Those wishing to use Pixy source code, software and/or
# technologies under different licensing terms should contact us at
# cmucam@cs.cmu.edu. Such licensing terms are available for
# all portions of the Pixy codebase presented here.
#
# end license header
#

# Pixy Tracking Demo - Python Version #
#!/usr/bin/env python

import pyuarm
import time
import sys
import signal
import pygame
from fcntl import ioctl

from pyfirmata import Arduino, util
import sys, traceback
from pykalman import KalmanFilter

import sys
import signal
from pixy import *
from ctypes import *
import os, struct, array
from scipy.interpolate import interp1d

K_BASED='x'
K_UP1='y'
K_UP2='trottle'
K_END='rz'
K_SLOW='tr'
K_POS='c'
K_SELECT='tl2'

board = Arduino('/dev/ttyUARM')
it = util.Iterator(board)
it.start()
board.analog[0].enable_reporting()
board.analog[1].enable_reporting()
board.analog[2].enable_reporting()
board.analog[3].enable_reporting()

#pin10 K_END
#pin11 K_UP1
#pin12 K_BASED
#pin13 K_UP2

pin4 = board.get_pin('d:4:o')
pin6 = board.get_pin('d:6:o')
pin9 = board.get_pin('d:9:o')
m_end = board.get_pin('d:10:s')
m_up1 = board.get_pin('d:11:s')
m_based = board.get_pin('d:12:s')
m_up2 = board.get_pin('d:13:s')
#pin9 = board.get_pin('a:9:o')

pin14 = board.get_pin('a:0:o')
pin15 = board.get_pin('a:1:o')
pin16 = board.get_pin('a:2:o')
pin17 = board.get_pin('a:3:o')

base_state=1
base2_state=1
base4_state=0
slowrate=1

current_z=37
current_rz=101
current_based=90

pos_based=180

#pin9.write(0)
m_end.write(90)
m_up1.write(current_z)
m_up2.write(current_rz)
m_based.write(current_based)
offset_min_y=0
offset_min_z=0
offset_min_rz=0

offset_max_y=1
offset_max_z=1
offset_max_rz=1

for fn in os.listdir('/dev/input'):
    if fn.startswith('js'):
        print('  /dev/input/%s' % (fn))
# We'll store the states here.
axis_states = {}
button_states = {}
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


class UARMServo:
        def Calibration(self):
           m_up1.write(0)
           time.sleep(5.0)
           offset_min_z=pin16.read()
           m_up1.write(180)
           time.sleep(5.0)
           offset_max_z=pin16.read()
           print "offset Z min %s"%(offset_min_z)
           print "offset Z max %s"%(offset_max_z)

        def AnalogToDegree(rz,y,z):
         print 'RZ%s,Y%s,Z%s'%(pin14.read(),pin15.read(),pin16.read())
        def Position(self):
          print "OK"

PIXY_MIN_X             =    0
PIXY_MAX_X             =  319
PIXY_MIN_Y             =    0
PIXY_MAX_Y             =  199

PIXY_X_CENTER          =  ((PIXY_MAX_X-PIXY_MIN_X) / 2)
PIXY_Y_CENTER          =  ((PIXY_MAX_Y-PIXY_MIN_Y) / 2)
PIXY_RCS_MIN_POS       =    0
PIXY_RCS_MAX_POS       = 1000
PIXY_RCS_CENTER_POS    =  ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS) / 2)

PIXY_RCS_PAN_CHANNEL   =    0
PIXY_RCS_TILT_CHANNEL  =    1

PAN_PROPORTIONAL_GAIN  =  400
PAN_DERIVATIVE_GAIN    =  300
TILT_PROPORTIONAL_GAIN =  500
TILT_DERIVATIVE_GAIN   =  400

BLOCK_BUFFER_SIZE      =    1


# Globals #

run_flag = True


class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]

class Gimbal ():
  _fields_ = [ ("position", c_uint),
               ("first_update", bool),
               ("previous_error", c_uint),
               ("proportional_gain", c_uint),
               ("derivative_gain", c_uint) ]

  def __init__(self, start_position, proportional_gain, derivative_gain):
    self.position          = start_position
    self.proportional_gain = proportional_gain
    self.derivative_gain   = derivative_gain
    self.previous_error    = 0
    self.first_update      = True

  def update(self, error):
    if self.first_update == False:
      error_delta = error - self.previous_error
      P_gain      = self.proportional_gain;
      D_gain      = self.derivative_gain;

      # Using the proportional and derivative gain for the gimbal #
      # calculate the change to the position                      #
      velocity = (error * P_gain + error_delta * D_gain) / 1024;

      self.position += velocity;

      if self.position > PIXY_RCS_MAX_POS:
        self.position = PIXY_RCS_MAX_POS
      elif self.position < PIXY_RCS_MIN_POS:
        self.position = PIXY_RCS_MIN_POS
    else:
      self.first_update = False

    self.previous_error = error

def handle_SIGINT(signal, frame):
  global run_flag
  run_flag = False

def main():
  global run_flag

  print '+ UARM Pixy Tracking Started +'

  # Initialize Pixy Interpreter thread #
  pixy_init_status = pixy_init()

  if pixy_init_status != 0:
    print 'Error: pixy_init() [%d] ' % pixy_init_status
    pixy_error(pixy_init_status)
    return

  #  Initialize Gimbals #
  pan_gimbal  = Gimbal(PIXY_RCS_CENTER_POS, PAN_PROPORTIONAL_GAIN, PAN_DERIVATIVE_GAIN)
  tilt_gimbal = Gimbal(PIXY_RCS_CENTER_POS, TILT_PROPORTIONAL_GAIN, TILT_DERIVATIVE_GAIN)

  # Initialize block #
  block       = Block()
  frame_index = 0

  signal.signal(signal.SIGINT, handle_SIGINT)

  # Run until we receive the INTERRUPT signal #
  swap=1
  while (run_flag) or (not KeyboardInterrupt):
#    evbuf = jsdev.read(8)
#    if evbuf:
#        times, value, type, number = struct.unpack('IhBB', evbuf)
#	if type & 0x01:
#         button = button_map[number]
#         if button:
#            if ( button is K_SELECT and value):
#               board.exit()
#               sys.exit(0)

    # Do nothing until a new block is available #
    while not pixy_blocks_are_new() and run_flag:
      pass

    # Grab a block #
    count = pixy_get_blocks(BLOCK_BUFFER_SIZE, block)

    # Was there an error? #
    if count < 0:
      print 'Error: pixy_get_blocks() [%d] ' % count
      pixy_error(count)
      sys.exit(1)

    if count > 0:
      # We found a block #

      # Calculate the difference between Pixy's center of focus #
      # and the target.                                         #
      pan_error  = PIXY_X_CENTER - block.x
      tilt_error = block.y - PIXY_Y_CENTER

      # Apply corrections to the pan/tilt gimbals with the goal #
      # of putting the target in the center of Pixy's focus.    #
      pan_gimbal.update(pan_error)
      tilt_gimbal.update(tilt_error)
      
      set_position_result = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, pan_gimbal.position)

      if set_position_result < 0:
        print 'Error: pixy_rcs_set_position() [%d] ' % result
        pixy_error(result)
        sys.exit(2)

      set_position_result = pixy_rcs_set_position(PIXY_RCS_TILT_CHANNEL, tilt_gimbal.position)

      if set_position_result < 0:
        print 'Error: pixy_rcs_set_position() [%d] ' % result
        pixy_error(result)
        sys.exit(2)

    if (frame_index % 2) == 0:
      if count == 1:
#        print '  sig:%2d x:%4d y:%4d width:%4d height:%4d' % (block.signature, block.x, block.y, block.width, block.height)
#	global pos_based
#	if (pos_based > 200) or (pos_based < 0):
#		print "pos_based overrange : %f"%(pos_based)
#  		pixy_close()
#  		board.exit()
#  		sys.exit(0)
           
	m = interp1d([-160,160],[10,-10])
        print "pan = %d , m = %f"%(pan_error,m(pan_error))
#	print "pos_based : %f"%(pos_based)
        m_based.write(current_based+m(pan_error))
#	pos_based+=m(pan_error)
        time.sleep(0.2)
        m_based.write(current_based)
      
	if (swap == 1):
#	 m = interp1d([-150,150],[-3,3])
	 m = interp1d([-150,150],[5,85])
         global current_z
         if (current_z > 90):
           current_z=90
         if (current_z < 5):
           current_z=5
         current_z = m(tilt_error)
         m_up1.write(current_z)

         global current_rz
         if (current_rz > 120):
           current_rz=120
         if (current_rz < 50):
           current_rz=50
	 m = interp1d([-120,120],[3,-3])
         current_rz += m(tilt_error)
	 print "UP2 : %f"%(current_rz)
         m_up2.write(current_rz)

    frame_index = frame_index + 1

  pixy_close()
  board.exit()
  sys.exit(0)

if __name__ == "__main__":
  main()

# LEIMON 2015 #
