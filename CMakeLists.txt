############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2017
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
# CMakeLists.txt for Marccd camera

SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")
#ADD_DEFINITIONS(-std=c++0x -Wall -Wextra)

SET(marccd_srcs src/MarccdCamera.cpp  src/MarccdInterface.cpp
	 src/MarccdDetInfoCtrlObj.cpp src/MarccdSyncCtrlObj.cpp
	 src/MarccdBinCtrlObj.cpp src/MarccdBufferCtrlObj.cpp
	 src/MarccdRoiCtrlObj.cpp src/MarccdReader.cpp)

FILE(STRINGS "VERSION" marccd_vers)
ADD_LIBRARY(limamarccd SHARED ${marccd_srcs})

TARGET_INCLUDE_DIRECTORIES(limamarccd PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
TARGET_INCLUDE_DIRECTORIES(limamarccd PUBLIC "${YAT_ROOT}/include")

FIND_LIBRARY(MARCCD_LIB_YAT yat ${YAT_ROOT}/lib64)
FIND_LIBRARY(MARCCD_LIB_RT rt)
IF("${MARCCD_LIB_RT}" STREQUAL "MARCCD_LIB_RT-NOTFOUND") 
  MESSAGE(FATAL_ERROR "marccd : librt not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()
IF("${MARCCD_LIB_YAT}" STREQUAL "MARCCD_LIB_YAT-NOTFOUND")
  MESSAGE(FATAL_ERROR "marccd : libyat not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()

TARGET_LINK_LIBRARIES(limamarccd limacore)
TARGET_LINK_LIBRARIES(limamarccd ${MARCCD_LIB_YAT})
TARGET_LINK_LIBRARIES(limamarccd ${MARCCD_LIB_RT})

SET_TARGET_PROPERTIES(limamarccd PROPERTIES VERSION ${marccd_vers} SOVERSION "1")

INSTALL(TARGETS limamarccd LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "marccd")
    include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Marccd")
ENDIF()
