#!/usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken), 
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

THIS_PACKAGE = "rosserial_stm32"

__usage__ = """
make_libraries.py generates the STM32 rosserial library files for SW4STM32.
It requires the family of your STM32 and location of your SWSTM32 project folder.

rosrun rosserial_stm32 make_libraries.py F3 <output_path>
"""

import rospkg
import rosserial_client
from rosserial_client.make_library import *

# for copying files
import shutil

ROS_TO_EMBEDDED_TYPES = {
    'bool'    :   ('bool',              1, PrimitiveDataType, []),
    'byte'    :   ('int8_t',            1, PrimitiveDataType, []),
    'int8'    :   ('int8_t',            1, PrimitiveDataType, []),
    'char'    :   ('uint8_t',           1, PrimitiveDataType, []),
    'uint8'   :   ('uint8_t',           1, PrimitiveDataType, []),
    'int16'   :   ('int16_t',           2, PrimitiveDataType, []),
    'uint16'  :   ('uint16_t',          2, PrimitiveDataType, []),
    'int32'   :   ('int32_t',           4, PrimitiveDataType, []),
    'uint32'  :   ('uint32_t',          4, PrimitiveDataType, []),
    'int64'   :   ('int64_t',           8, PrimitiveDataType, []),
    'uint64'  :   ('uint64_t',          8, PrimitiveDataType, []),
    'float32' :   ('float',             4, PrimitiveDataType, []),
    'float64' :   ('float',             4, AVR_Float64DataType, []),
    'time'    :   ('ros::Time',         8, TimeDataType, ['ros/time']),
    'duration':   ('ros::Duration',     8, TimeDataType, ['ros/duration']),
    'string'  :   ('char*',             0, StringDataType, []),
    'Header'  :   ('std_msgs::Header',  0, MessageDataType, ['std_msgs/Header'])
}

# need correct inputs
if (len(sys.argv) < 3):
    print __usage__
    exit()

# parse family
family = sys.argv[1]
if (family[0] != "F") or (len(family) < 2) or (len(family) > 4) or (not family[1:].isdigit()):
    
    if family[0] != "F":
        print "Error 1"
    if len(family) < 2:
        print "Error 2"
    if len(family) > 4:
        print "Error 3"
    if not family[1:].isdigit():
        print "Error 4"
    print "Family = " + family
    
    print ("Wrong family!")
    print __usage__
    exit()
while len(family) < 4:
    family += "X"
header_filename = "STM32%s.h"
family = header_filename % (family);

rospack = rospkg.RosPack()
rosserial_stm32_dir = rospack.get_path(THIS_PACKAGE)
family_dir = rosserial_stm32_dir + "/src/rosserial_stm32/family/"
files = os.listdir(family_dir[:-1])
have_family = False
for f in files:
    if f == family:
        have_family = True
        break
if not have_family:
    print "There is not header file for " + family[:-2]
    exit()
    
# get output path
path = sys.argv[2]
if path[-1] == "/":
    path = path[0:-1]
path += "/ROS/"
print "\nExporting to %s" % path

# copy ros_lib stuff in
src_lib_dir = rosserial_stm32_dir + "/src/ros_lib/"
shutil.copytree(src_lib_dir, path)
shutil.copy(family_dir + family, path + header_filename % ("FXXX"))

# generate messages
rosserial_generate(rospack, path, ROS_TO_EMBEDDED_TYPES)

