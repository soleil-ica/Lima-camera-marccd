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
set(NAME "marccd")

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(${NAME}_srcs src/MarccdCamera.cpp  src/MarccdInterface.cpp
	 src/MarccdDetInfoCtrlObj.cpp src/MarccdSyncCtrlObj.cpp
	 src/MarccdBinCtrlObj.cpp src/MarccdBufferCtrlObj.cpp
	 src/MarccdRoiCtrlObj.cpp src/MarccdReader.cpp)

add_library(lima${NAME} SHARED ${${NAME}_srcs})

target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(lima${NAME} PUBLIC "/usr/local/yat/include")

find_library(MARCCD_YAT_LIB yat /usr/local/yat/lib64)
find_library(MARCCD_RT_LIB rt)
if("${MARCCD_RT_LIB}" STREQUAL "MARCCD_RT_LIB-NOTFOUND") 
  message(FATAL_ERROR "${NAME} : librt not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if("${MARCCD_YAT_LIB}" STREQUAL "MARCCD_YAT_LIB-NOTFOUND")
  message(FATAL_ERROR "${NAME} : libyat not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()

target_link_libraries(lima${NAME} limacore)
target_link_libraries(lima${NAME} ${MARCCD_YAT_LIB})
target_link_libraries(lima${NAME} ${MARCCD_RT_LIB})

include(CameraVERSION)
install(TARGETS lima${NAME} LIBRARY DESTINATION lib)

if(LIMA_ENABLE_PYTHON)
    include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Marccd")
endif()
