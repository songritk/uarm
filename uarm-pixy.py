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

from pyfirmata import Arduino, util
import sys, traceback
from pykalman import KalmanFilter

import sys
import signal
from pixy import *
from ctypes import *
from scipy.interpolate import interp1d

K_BASED='x'
K_UP1='y'
K_UP2='trottle'
K_END='rz'
K_SLOW='tr'
K_POS='c'

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

pin9.write(0)
m_end.write(45)
m_up1.write(current_z)
m_up2.write(current_rz)
m_based.write(current_based)
offset_min_y=0
offset_min_z=0
offset_min_rz=0

offset_max_y=1
offset_max_z=1
offset_max_rz=1

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

  print '+ Pixy Tracking Demo Started +'

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
	m = interp1d([-160,160],[10,-10])
#        print "pan = %d , m = %f"%(pan_error,m(pan_error))
        m_based.write(current_based+m(pan_error))
        time.sleep(0.2)
        m_based.write(current_based)
      
	if (swap == 1):
#	 m = interp1d([-150,150],[-3,3])
	 m = interp1d([-150,150],[5,80])
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
	 m = interp1d([-100,100],[5,-5])
         current_rz += m(tilt_error)
         m_up2.write(current_rz)

    frame_index = frame_index + 1

  pixy_close()
  board.exit()
  sys.exit(0)

if __name__ == "__main__":
  main()

# LEIMON 2015 #
