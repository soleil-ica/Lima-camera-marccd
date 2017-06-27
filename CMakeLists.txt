###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(marccd_srcs src/MarccdCamera.cpp  src/MarccdInterface.cpp
	 src/MarccdDetInfoCtrlObj.cpp src/MarccdSyncCtrlObj.cpp
	 src/MarccdBinCtrlObj.cpp src/MarccdBufferCtrlObj.cpp
	 src/MarccdRoiCtrlObj.cpp src/MarccdReader.cpp)

file(STRINGS "VERSION" marccd_vers)
add_library(limamarccd SHARED ${marccd_srcs})

target_include_directories(limamarccd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(limamarccd PUBLIC "/usr/local/yat/include")

find_library(MARCCD_LIB_YAT yat /usr/local/yat/lib64)
find_library(MARCCD_LIB_RT rt)
if("${MARCCD_LIB_RT}" STREQUAL "MARCCD_LIB_RT-NOTFOUND") 
  message(FATAL_ERROR "marccd : librt not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if("${MARCCD_LIB_YAT}" STREQUAL "MARCCD_LIB_YAT-NOTFOUND")
  message(FATAL_ERROR "marccd : libyat not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()

target_link_libraries(limamarccd limacore)
target_link_libraries(limamarccd ${MARCCD_LIB_YAT})
target_link_libraries(limamarccd ${MARCCD_LIB_RT})

set_target_properties(limamarccd PROPERTIES VERSION ${marccd_vers} SOVERSION "1")

install(TARGETS limamarccd LIBRARY DESTINATION lib)

if(COMPILE_SIP)
    set(NAME "marccd")
    include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Marccd")
endif()
