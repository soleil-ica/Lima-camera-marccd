//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2011
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################

#include "MarccdBufferCtrlObj.h"
#include "MarccdCamera.h"

using namespace lima;
using namespace lima::Marccd;

/*******************************************************************
 * \brief BufferCtrlObj constructor
 *******************************************************************/
BufferCtrlObj::BufferCtrlObj(Camera& cam)
: m_buffer_cb_mgr(m_buffer_alloc_mgr),
m_buffer_ctrl_mgr(m_buffer_cb_mgr),
m_cam(cam)
{
    DEB_CONSTRUCTOR();
    m_reader = new Reader(cam, *this);
    m_reader->go(2000);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
BufferCtrlObj::~BufferCtrlObj()
{
    DEB_DESTRUCTOR();
    m_reader->reset();
    m_reader->exit();
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setFrameDim(const FrameDim& frame_dim)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.setFrameDim(frame_dim);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getFrameDim(FrameDim& frame_dim)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getFrameDim(frame_dim);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setNbBuffers(int nb_buffers)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.setNbBuffers(nb_buffers);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getNbBuffers(int& nb_buffers)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getNbBuffers(nb_buffers);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setNbConcatFrames(int nb_concat_frames)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.setNbConcatFrames(nb_concat_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getNbConcatFrames(int& nb_concat_frames)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getNbConcatFrames(nb_concat_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void BufferCtrlObj::getMaxNbBuffers(int& max_nb_buffers)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getMaxNbBuffers(max_nb_buffers);
}
//-----------------------------------------------------
//
//-----------------------------------------------------
void *BufferCtrlObj::getBufferPtr(int buffer_nb, int concat_frame_nb)
{
    DEB_MEMBER_FUNCT();
    return m_buffer_ctrl_mgr.getBufferPtr(buffer_nb, concat_frame_nb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void *BufferCtrlObj::getFramePtr(int acq_frame_nb)
{
    DEB_MEMBER_FUNCT();
    return m_buffer_ctrl_mgr.getFramePtr(acq_frame_nb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getStartTimestamp(Timestamp& start_ts)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getStartTimestamp(start_ts);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::getFrameInfo(int acq_frame_nb, HwFrameInfoType& info)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.getFrameInfo(acq_frame_nb, info);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::registerFrameCallback(HwFrameCallback& frame_cb)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.registerFrameCallback(frame_cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::unregisterFrameCallback(HwFrameCallback& frame_cb)
{
    DEB_MEMBER_FUNCT();
    m_buffer_ctrl_mgr.unregisterFrameCallback(frame_cb);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::start()
{
    DEB_MEMBER_FUNCT();
    //- start thread which will _read the new image from file
    m_reader->start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::reset()
{
    DEB_MEMBER_FUNCT();
    m_reader->reset();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int BufferCtrlObj::getLastAcquiredFrame()
{
    return m_reader->getLastAcquiredFrame();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool BufferCtrlObj::isTimeoutSignaled()
{
    return m_reader->isTimeoutSignaled();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
bool BufferCtrlObj::isRunning()
{
    return m_reader->isRunning();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void BufferCtrlObj::setTimeout(int TO)
{
    DEB_MEMBER_FUNCT();
    m_reader->setTimeout(TO);
}


//-----------------------------------------------------
//
//-----------------------------------------------------

void BufferCtrlObj::setWaitFileOnDiskTime(double value)
{
    DEB_MEMBER_FUNCT();
    m_reader->setWaitFileOnDiskTime(value);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

double BufferCtrlObj::getWaitFileOnDiskTime(void)
{
    DEB_MEMBER_FUNCT();
    return m_reader->getWaitFileOnDiskTime();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int* BufferCtrlObj::getHeader()
{
    DEB_MEMBER_FUNCT();
    return m_reader->getHeader();
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void BufferCtrlObj::setNbRetry(int value)
{
    DEB_MEMBER_FUNCT();
    m_reader->setNbRetry(value);
}
