#!/usr/bin/env python

# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16,c_char


SLAM_CHANNEL_NAME = 'slam'

# only need to send a char representing user input
class SLAM_TYPE(Structure):
	_pack_ = 1
	_fields_ = [("xSLAM",    c_double),
                    ("ySLAM",  c_double),
		    ("yawSLAM", c_double)]
