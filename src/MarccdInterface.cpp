#include "MarccdInterface.h"
#include <algorithm>

using namespace lima;
using namespace lima::Marccd;
using namespace std;


/*******************************************************************
 * \brief Hw Interface constructor
 *******************************************************************/
Interface::Interface(Camera& cam)
: m_cam(cam), m_det_info(cam), m_buffer(cam), m_sync(cam), m_roi(cam), m_bin(cam)
{
    DEB_CONSTRUCTOR();

    HwDetInfoCtrlObj *det_info = &m_det_info;
    m_cap_list.push_back(HwCap(det_info));

    HwBufferCtrlObj *buffer = &m_buffer;
    m_cap_list.push_back(HwCap(buffer));

    HwSyncCtrlObj *sync = &m_sync;
    m_cap_list.push_back(HwCap(sync));

    HwRoiCtrlObj *roi = &m_roi;
    m_cap_list.push_back(HwCap(roi));

    HwBinCtrlObj *bin = &m_bin;
    m_cap_list.push_back(HwCap(bin));
    
}

//-----------------------------------------------------
//
//-----------------------------------------------------
Interface::~Interface()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
    DEB_MEMBER_FUNCT();
    cap_list = m_cap_list;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(reset_level);

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    m_buffer.reset();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::startAcq()
{
    DEB_MEMBER_FUNCT();
    m_cam.start();
    m_buffer.start();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::stopAcq()
{
    DEB_MEMBER_FUNCT();
    m_cam.stop();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{
    Camera::Status marccd_status = Camera::Unknown;

    int nb_frames;
    m_cam.getNbFrames(nb_frames);

    if (this->m_buffer.isRunning())
    {
        status.acq = AcqRunning;
        status.det = DetExposure;
    }
    else if (m_buffer.isTimeoutSignaled())
    {
        status.acq = AcqFault;
        status.det = DetFault;
    }
    else
    {
        m_cam.getStatus(marccd_status);
        switch (marccd_status)
        {
            case Camera::Ready:
                status.acq = AcqReady;
                status.det = DetIdle;
                break;

            case Camera::Exposure:
                status.acq = AcqRunning;
                status.det = DetExposure;
                break;

            case Camera::Readout:
                status.acq = AcqRunning;
                status.det = DetReadout;
                break;

            case Camera::Latency:
                status.acq = AcqRunning;
                status.det = DetLatency;
                break;

            case Camera::Config:
                status.acq = AcqConfig;
                status.det = DetExposure;
                break;

            case Camera::Unknown:
            case Camera::Fault:
                status.acq = AcqFault;
                status.det = DetFault;
                break;
        }
    }
    status.det_mask = DetExposure | DetReadout | DetLatency | DetFault ;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
    DEB_MEMBER_FUNCT();
    int acq_frames = m_buffer.getLastAcquiredFrame();
    return acq_frames;
}

//-----------------------------------------------------
// get last buffer header
//-----------------------------------------------------
int* Interface::getHeader()
{
    DEB_MEMBER_FUNCT();
    return m_buffer.getHeader();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setTimeout(int TO)
{
    DEB_MEMBER_FUNCT();
    m_buffer.setTimeout(TO);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Interface::setWaitFileOnDiskTime(double value)
{
    DEB_MEMBER_FUNCT();
    m_buffer.setWaitFileOnDiskTime(value);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
double Interface::getWaitFileOnDiskTime(void)
{
    DEB_MEMBER_FUNCT();
    return m_buffer.getWaitFileOnDiskTime();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Interface::setNbRetry(int value)
{
    DEB_MEMBER_FUNCT();
    m_buffer.setNbRetry(value);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

