#!/usr/bin/env python

# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16,c_char


LOCALIZER_CHANNEL_NAME = 'localizer'

# only need to send a char representing user input
class CHAN_TYPE(Structure):
	_pack_ = 1
	_fields_ = [("valid", c_uint32),
		    ("x",    c_double),
		    ("y",    c_double),
		    ("z",    c_uint32),
		    ("THETA", c_double)]
