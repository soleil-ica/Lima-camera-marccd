#include "MarccdCamera.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <time.h>
#include "Timer.h"


using namespace lima;
using namespace lima::Marccd;

const double PixelSize = 40 * 1e-06; //40µm for binning 1x1

//polling delay of the marccd state
const size_t MARCCD_POLL_DELAY = 0.01;

//- These are the STEP of acquisition process in the state machine
const size_t PREPARE_STEP = 0;
const size_t WAIT_ACQUIRE_IDLE_STEP = 1;
const size_t SEND_START_STEP = 2;
const size_t WAIT_ACQUIRE_EXECUTING_STEP = 3;
const size_t WAIT_EXPOSURE_STEP = 4;
const size_t SEND_READOUT_STEP = 5;
const size_t WAIT_WRITING_END_STEP = 6;
const size_t WAIT_LATENCY_STEP = 7;

//- User task messages
const size_t START_MSG = (yat::FIRST_USER_MSG + 100);
const size_t STOP_MSG = (yat::FIRST_USER_MSG + 101);
const size_t BACKGROUND_FRAME_MSG = (yat::FIRST_USER_MSG + 103);

//- task period (in ms)
const size_t kPERIODIC_MSG_PERIOD = 500;

//- Task numbers
const size_t TASK_ACQUIRE = 0;
const size_t TASK_READ = 1;
const size_t TASK_CORRECT = 2;
const size_t TASK_WRITE = 3;
const size_t TASK_DEZINGER = 4;


//- Task Status bits
const size_t TASK_STATUS_QUEUED = 0x1;
const size_t TASK_STATUS_EXECUTING = 0x2;
const size_t TASK_STATUS_ERROR = 0x4;
const size_t TASK_STATUS_RESERVED = 0x8;

//- These are the "old" states from version 0, but BUSY is also used in version 1
const size_t TASK_STATE_IDLE = 0;
const size_t TASK_STATE_ACQUIRE = 1;
const size_t TASK_STATE_READOUT = 2;
const size_t TASK_STATE_CORRECT = 3;
const size_t TASK_STATE_WRITING = 4;
const size_t TASK_STATE_ABORTING = 5;
const size_t TASK_STATE_UNAVAILABLE = 6;
const size_t TASK_STATE_ERROR = 7; //- command not understood
const size_t TASK_STATE_BUSY = 8; //- interpreting command

//- These are the definitions of masks for looking at task state bits
#define STATE_MASK                  0xf
#define STATUS_MASK                 0xf
#define TASK_STATUS_MASK(task)      (STATUS_MASK << (4*((task)+1)))

//- These are some convenient macros for checking and setting the state of each task
//- They are used in the marccd code and can be used in the client code
#define TASK_STATE(current_status)                      ((current_status) & STATE_MASK)
#define TASK_STATUS(current_status, task)               (((current_status) & TASK_STATUS_MASK(task)) >> (4*((task) + 1)))
#define TEST_TASK_STATUS(current_status, task, status)  (TASK_STATUS(current_status, task) & (status))

#define THROW_ON_STATUS_ERROR(current_status, origin)                       \
if (TEST_TASK_STATUS(current_status, TASK_ACQUIRE, TASK_STATUS_ERROR) ||    \
    TEST_TASK_STATUS(current_status, TASK_READ, TASK_STATUS_ERROR) ||       \
    TEST_TASK_STATUS(current_status, TASK_WRITE, TASK_STATUS_ERROR))        \
{                                                                           \
    throw LIMA_HW_EXC(Error, origin) << DEB_HEX(current_status);            \
}                                                                           \

//---------------------------
//- Ctor
//---------------------------

Camera::Camera(const std::string& camera_ip,
               size_t port_num,
               const std::string& img_path) :
m_nb_frames(0),
m_detector_size(4096, 4096),
m_image_path(img_path),
m_image_name("buffer"),
m_exp_time(0),
m_lat_time(0),
m_binning_x(1),
m_binning_y(1),
m_marccd_state(TASK_STATE_IDLE),
m_status(Camera::Unknown),
m_image_number(0),
m_first_image(0),
m_camera_ip(camera_ip),
m_port_num(port_num),
m_detector_model(""),
m_detector_type(""),
m_source_beamX(0),
m_source_beamY(0),
m_source_distance(0),
m_source_wavelength(0),
m_is_stop_in_progress(false),
m_is_bg_acquisition_in_progress(false),
m_is_bg_saving_requested(false),
m_error(""),
m_exposure_cb(),
m_latency_cb(),
m_exposure_timer(&m_exposure_cb),
m_latency_timer(&m_latency_cb)
{
    DEB_CONSTRUCTOR();

    m_status = Camera::Ready;
    m_detector_type = "MARCCD";
    m_detector_model = "SX 165";

    //prepare & connect the socket to detector
    try
    {

        //- internal YAT stuff cooking
        yat::Socket::init();
        DEB_TRACE() << "Init yat::socket";
        //- connect to MARCCD device
        DEB_TRACE() << "Connect to MARCCD socket server";
        m_sock.set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
        m_sock.set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
        m_sock.set_option(yat::Socket::SOCK_OPT_OTIMEOUT, 0);
        m_sock.set_option(yat::Socket::SOCK_OPT_ITIMEOUT, 0);
        yat::Address addr(m_camera_ip, m_port_num);
        m_sock.connect(addr);
        //- no error
        m_error.clear();
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "Unknown Exception : Unable to establish a communication with MARCCD socket server!";
    }

    //socket is OK and connected, prepare & start the task
    enable_timeout_msg(false);
    set_periodic_msg_period(kPERIODIC_MSG_PERIOD);
    enable_periodic_msg(false);
    go(2000);
}

//---------------------------
//- Dtor
//---------------------------

Camera::~Camera()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::handle_message(yat::Message& msg) throw ( yat::Exception)
{
    DEB_MEMBER_FUNCT();
    try
    {
        switch (msg.type())
        {

                //-----------------------------------------------------
            case yat::TASK_INIT:
            {
                DEB_TRACE() << "Camera::->TASK_INIT";
                //NOP
            }
                break;

                //-----------------------------------------------------
            case yat::TASK_EXIT:
            {
                DEB_TRACE() << "Camera::->TASK_EXIT";
                try
                {
                    //- abort acquisition
                    DEB_TRACE() << "Send command abort";
                    _sendCmdAndReceiveAnswer("abort");

                    //-  Delete device allocated objects
                    DEB_TRACE() << "Disconnect from MARCCD socket server";
                    m_sock.disconnect();

                    //- yat internal cooking
                    DEB_TRACE() << "Terminate yat::socket";
                    yat::Socket::terminate();
                }
                catch (yat::SocketException & ySe)
                {
                    std::stringstream ssError;
                    for (unsigned i = 0; i < ySe.errors.size(); i++)
                    {
                        ssError << ySe.errors[i].desc << std::endl;
                    }

                    THROW_HW_ERROR(Error) << ssError.str();
                }
                catch (...)
                {
                    THROW_HW_ERROR(Error) << "Camera::TASK_EXIT : Unknown Exception";
                }
            }
                break;

                //-----------------------------------------------------
            case yat::TASK_TIMEOUT:
            {
                DEB_TRACE() << "Camera::->TASK_TIMEOUT";
                //NOP
            }
                break;

                //------------------------------------------------------
            case yat::TASK_PERIODIC:
            {
                DEB_TRACE() << "Camera::->TASK_PERIODIC";
                //NOP
            }
                break;

                //------------------------------------------------------
            case START_MSG:
            {
                DEB_TRACE() << "Camera::->START_MSG";

                try
                {
                    DEB_TRACE() << "Perform acquisition sequence ...";
                    _performAcquisitionSequence();
                }
                catch (yat::SocketException & ySe)
                {
                    m_nb_frames = 0; // This will stop the reader                    
                    std::stringstream ssError;
                    for (unsigned i = 0; i < ySe.errors.size(); i++)
                    {
                        ssError << ySe.errors[i].desc << std::endl;
                    }

                    THROW_HW_ERROR(Error) << ssError.str();
                }
                catch (...)
                {
                    THROW_HW_ERROR(Error) << "Camera::START_MSG : Unknown Exception";
                }
            }
                break;

                //-----------------------------------------------------
            case STOP_MSG:
            {
                DEB_TRACE() << "Camera::->STOP_MSG";

                try
                {
                    DEB_TRACE() << "Perform stop sequence ...";
                    _performStopSequence();
                }
                catch (yat::SocketException & ySe)
                {
                    m_nb_frames = 0; // This will stop the reader                    
                    std::stringstream ssError;
                    for (unsigned i = 0; i < ySe.errors.size(); i++)
                    {
                        ssError << ySe.errors[i].desc << std::endl;
                    }

                    THROW_HW_ERROR(Error) << ssError.str();
                }
                catch (...)
                {
                    THROW_HW_ERROR(Error) << "Camera::STOP_MSG : Unknown Exception";
                }
            }
                break;

                //-----------------------------------------------------
            case BACKGROUND_FRAME_MSG:
            {
                DEB_TRACE() << "Camera::->BACKGROUND_FRAME_MSG";

                try
                {
                    DEB_TRACE() << "Perform background frame ...";
                    _performBackgroundFrame();
                }
                catch (yat::SocketException & ySe)
                {
                    m_nb_frames = 0; // This will stop the reader                    
                    std::stringstream ssError;
                    for (unsigned i = 0; i < ySe.errors.size(); i++)
                    {
                        ssError << ySe.errors[i].desc << std::endl;
                    }

                    THROW_HW_ERROR(Error) << ssError.str();
                }
                catch (...)
                {
                    THROW_HW_ERROR(Error) << "Camera::BACKGROUND_FRAME_MSG : Unknown Exception";
                }
            }
                break;
        }
    }
    catch (yat::Exception& ex)
    {
        DEB_ERROR() << "Camera::handle_message : " << ex.errors[0].desc;
        THROW_HW_ERROR(Error) << "Camera::handle_message : " << ex.errors[0].desc;
    }
    catch (...)
    {
        DEB_ERROR() << "Camera::handle_message : " << "YAT Task Unknown error ! ";
        THROW_HW_ERROR(Error) << "Camera::handle_message : " << "YAT Task Unknown error ! ";
    }
}

//---------------------------
//- Camera::start()
//---------------------------

void Camera::start()
{
    DEB_MEMBER_FUNCT();

    //- post message START_MSG to the Task (asynchronous way)
    yat::Message * msg = new yat::Message(START_MSG, MAX_USER_PRIORITY);
    post(msg);
}

//---------------------------
//- Camera::stop()
//---------------------------

void Camera::stop()
{
    DEB_MEMBER_FUNCT();
    {
        yat::MutexLock scoped_lock(m_lock_flag);
        m_is_stop_in_progress = true;
    }

    //- post message STOP_MSG to the Task (asynchronous way)
    yat::Message * msg = new yat::Message(STOP_MSG, MAX_USER_PRIORITY);
    post(msg);

}


//---------------------------
//- Camera::takeBackgroundFrame()
//---------------------------

void Camera::takeBackgroundFrame()
{
    DEB_MEMBER_FUNCT();
    {
        yat::MutexLock scoped_lock(m_lock_flag);
        m_is_bg_acquisition_in_progress = true;
    }

    //- post message BACKGROUND_FRAME_MSG to the Task (asynchronous way)
    yat::Message * msg = new yat::Message(BACKGROUND_FRAME_MSG, MAX_USER_PRIORITY);
    post(msg);
}


//-----------------------------------------------------
//- getImageSize
//-----------------------------------------------------

void Camera::getImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();

    std::string resp("");
    int sizeX, sizeY;
    try
    {
        yat::MutexLock scoped_lock(m_lock);
        //- get the max image size of the detector
        resp = _sendCmdAndReceiveAnswer("get_size");

        sscanf(resp.c_str(), "%d,%d", &sizeX, &sizeY);

        size = Size(sizeX, sizeY);
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "getImageSize : Unknown Exception";
    }
}

//-----------------------------------------------------
//- getDetectorImageSize
//-----------------------------------------------------

void Camera::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    size = m_detector_size;
}

//-----------------------------------------------------
//- getPixelSize
//-----------------------------------------------------

void Camera::getPixelSize(double& x_size, double& y_size)
{
    DEB_MEMBER_FUNCT();
    x_size = y_size = PixelSize;
}

//-----------------------------------------------------
//- getImageType
//-----------------------------------------------------

void Camera::getImageType(ImageType& type)
{
    DEB_MEMBER_FUNCT();
    //-	default:
    type = Bpp16;
}

//-----------------------------------------------------
//- getDetectorType
//-----------------------------------------------------

void Camera::getDetectorType(std::string& type)
{
    DEB_MEMBER_FUNCT();
    type = m_detector_type;
}

//-----------------------------------------------------
//- getDetectorModel
//-----------------------------------------------------

void Camera::getDetectorModel(std::string& type)
{
    DEB_MEMBER_FUNCT();
    type = m_detector_model;
}

//-----------------------------------------------------
//- setTrigMode
//-----------------------------------------------------

void Camera::setTrigMode(TrigMode mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(mode);
    //TODO : Only INTERNAL for the moment
    try
    {
        if (mode == IntTrig)
        {
            //- INTERNAL
        }
        else
        {
            throw LIMA_HW_EXC(NotSupported, "setTrigMode : Trigger is not supported !");
        }
    }
    catch (...)
    {
        // Error handling
        THROW_HW_ERROR(Error) << "setTrigMode : Unknown Exception";
    }

}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getTrigMode(TrigMode& mode)
{
    DEB_MEMBER_FUNCT();
    //- TODO : Only INTERNAL for the moment !!!
    mode = IntTrig;
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setExpTime(double exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(exp_time);

    m_exp_time = exp_time;

}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getExpTime(double& exp_time)
{
    DEB_MEMBER_FUNCT();
    exp_time = m_exp_time;
    DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setLatTime(double lat_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(lat_time);

    // Check if the latency time is enough for the current binning
    // Minimal latency time is about the minimal readout time
    // and readout time is 5s / binning (2.5 for 2x2, 1.25 for 4x4 , ...)
    // NB: Marccd SX165 use a square binning i.e binning_x=binning_y and this is checked yet in checkBin()
    if (lat_time < 5. / m_binning_x)
        throw LIMA_HW_EXC(NotSupported, "setLatTime : Invalid time for this binning ") << DEB_VAR2(lat_time, m_binning_x);

    m_lat_time = lat_time;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getLatTime(double& lat_time)
{
    DEB_MEMBER_FUNCT();
    lat_time = m_lat_time;
    DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setNbFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(nb_frames);
    m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getNbFrames(int& nb_frames)
{
    DEB_MEMBER_FUNCT();
    nb_frames = m_nb_frames;
    DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    //nop
    hw_roi = set_roi;

    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    try
    {
        Point topleft, size;
        Roi hw_roi;
        getRoi(hw_roi);
        if (hw_roi == set_roi) return;

        if (set_roi.isActive())
        {
            // --- a real roi available
            hw_roi = set_roi;

            //- set the new roi values
            //- cmd format is : "set_roi,x0,y0,x1,y1"
            std::stringstream strRoi;
            strRoi << "set_roi"
                << ","
                << set_roi.getTopLeft().x
                << ","
                << set_roi.getTopLeft().y
                << ","
                << set_roi.getTopLeft().x + set_roi.getSize().getWidth()
                << ","
                << set_roi.getTopLeft().y + set_roi.getSize().getHeight();
            {
                yat::MutexLock scoped_lock(m_lock);
                _sendCmdAndReceiveAnswer(strRoi.str());
            }
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setRoi : Unknown Exception";
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    int x0 = 0, x1 = 0, y0 = 0, y1 = 0;
    try
    {
        //- get the new roi values        
        {
            std::string resp("");
            yat::MutexLock scoped_lock(m_lock);
            resp = _sendCmdAndReceiveAnswer("get_roi");
            sscanf(resp.c_str(), "%d,%d,%d,%d", &x0, &y0, &x1, &y1);
        }

        if (!x0 && !x1 && !y0 && !y1)//Roi is Off, i.e take full frame
        {
            DEB_TRACE() << "Roi is Off, Get Full frame.";

            Bin currentBin;
            getBinning(currentBin);
            Roi r(0, 0, m_detector_size.getWidth() / currentBin.getX(), m_detector_size.getHeight() / currentBin.getY());
            hw_roi = r;
        }
        else //Roi is On
        {
            DEB_TRACE() << "Roi is On, Get Roi frame.";
            Roi r(x0, y0, x0 + x1, y0 + y1);
            hw_roi = r;
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "getRoi : Unknown Exception";
    }
    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::checkBin(Bin& bin)
{
    DEB_MEMBER_FUNCT();
    int binX = bin.getX();
    int binY = bin.getY();

    //Marccd support only a square binning (1:1 , 2:2 , 4:4 , 8:8)
    if ((binX == 1 && binY == 1) ||
        (binX == 2 && binY == 2) ||
        (binX == 4 && binY == 4) ||
        (binX == 8 && binY == 8))
    {
        bin = Bin(binX, binY);
    }
    else
    {
        THROW_HW_ERROR(Error) << "checkBin : Invalid Bin = " << DEB_VAR1(bin);
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setBinning(const Bin &bin)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(bin);
    //- This is MarCCD supported bin values : "1,1"; 2,2" ; "4,4" ; "8,8" ;
    try
    {
        std::stringstream bin_values;
        m_binning_x = bin.getX();
        m_binning_y = bin.getY();
        bin_values << "set_bin," << m_binning_x << "," << m_binning_y << std::ends;
        //- set the new binning values
        {
            yat::MutexLock scoped_lock(m_lock);
            _sendCmdAndReceiveAnswer(bin_values.str());
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setBinning : Unknown Exception";
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getBinning(Bin& bin)
{
    DEB_MEMBER_FUNCT();

    int binX, binY;

    try
    {
        //- get the new binning values        
        {
            std::string resp("");
            yat::MutexLock scoped_lock(m_lock);
            resp = _sendCmdAndReceiveAnswer("get_bin");
            sscanf(resp.c_str(), "%d,%d", &binX, &binY);
        }

        bin = Bin(binX, binY);
        m_binning_x = binX;
        m_binning_y = binY;
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "getBinning : Unknown Exception";
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::getStatus(Camera::Status& status)
{
    DEB_MEMBER_FUNCT();
    size_t acquireStatus, readoutStatus, correctStatus, writingStatus, dezingerStatus;

    {
        yat::MutexLock scoped_lock_1(m_lock_flag);
        yat::MutexLock scoped_lock_2(m_lock_status);

        _updateMarccdState();

        acquireStatus = TASK_STATUS(m_marccd_state, TASK_ACQUIRE);
        readoutStatus = TASK_STATUS(m_marccd_state, TASK_READ);
        correctStatus = TASK_STATUS(m_marccd_state, TASK_CORRECT);
        writingStatus = TASK_STATUS(m_marccd_state, TASK_WRITE);
        dezingerStatus = TASK_STATUS(m_marccd_state, TASK_DEZINGER);

        if (m_is_bg_acquisition_in_progress)
            m_status = Camera::Config;
        else if (m_marccd_state == 0)
            m_status = Camera::Ready;
        else if (m_marccd_state == 7)
            m_status = Camera::Fault;
        else if (m_marccd_state == 8)
            m_status = Camera::Config; /* TODO: replace by Readout perhaps ?!!  This is really busy interpreting command but we don't have a status for that yet */
        else if (acquireStatus & (TASK_STATUS_EXECUTING))
            m_status = Camera::Exposure;
        else if (readoutStatus & (TASK_STATUS_EXECUTING))
            m_status = Camera::Readout;
        else if (correctStatus & (TASK_STATUS_EXECUTING))
            m_status = Camera::Readout; /*ADStatusCorrect*/
        else if (writingStatus & (TASK_STATUS_EXECUTING))
            m_status = Camera::Readout; /*ADStatusSaving*/

        if ((acquireStatus | readoutStatus | correctStatus | writingStatus | dezingerStatus) & TASK_STATUS_ERROR)
            m_status = Camera::Fault;

        status = m_status;
    }
    ////DEB_TRACE() << "getStatus() - status = " << status;
    ////DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}


//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setBeamX(float X)
{
    DEB_MEMBER_FUNCT();
    try
    {
        std::stringstream cmd;
        cmd << "header,beam_x=" << X << "\n";
        {
            yat::MutexLock scoped_lock(m_lock);
            _sendCmdAndReceiveAnswer(cmd.str());
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setBeamX : Unknown Exception";
    }
    m_source_beamX = X;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setBeamY(float Y)
{
    DEB_MEMBER_FUNCT();
    try
    {
        std::stringstream cmd;
        cmd << "header,beam_y=" << Y << "\n";
        {
            yat::MutexLock scoped_lock(m_lock);
            _sendCmdAndReceiveAnswer(cmd.str());
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setBeamY : Unknown Exception";
    }
    m_source_beamY = Y;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setDistance(float D)
{
    DEB_MEMBER_FUNCT();
    try
    {
        std::stringstream cmd;
        cmd << "header,detector_distance=" << D << "\n";
        {
            yat::MutexLock scoped_lock(m_lock);
            _sendCmdAndReceiveAnswer(cmd.str());
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setDistance : Unknown Exception";
    }
    m_source_distance = D;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Camera::setWavelength(float W)
{
    DEB_MEMBER_FUNCT();
    try
    {
        std::stringstream cmd;
        cmd << "header,source_wavelength=" << W << "\n";
        {
            yat::MutexLock scoped_lock(m_lock);
            _sendCmdAndReceiveAnswer(cmd.str());
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "setWavelength : Unknown Exception";
    }
    m_source_wavelength = W;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

float Camera::getBeamX()
{
    DEB_MEMBER_FUNCT();
    return m_source_beamX;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

float Camera::getBeamY()
{
    DEB_MEMBER_FUNCT();
    return m_source_beamY;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

float Camera::getDistance()
{
    DEB_MEMBER_FUNCT();
    return m_source_distance;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

float Camera::getWavelength()
{
    DEB_MEMBER_FUNCT();
    return m_source_wavelength;
}

//-----------------------------------------------------
// - _read : returns Marccd command response
//-----------------------------------------------------

std::string Camera::_read()
{
    DEB_MEMBER_FUNCT();
    std::string response("");
    try
    {
        m_sock.receive(response);
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "_read : Unknown Exception";
    }

    return response;
}

//-----------------------------------------------------
// - _write : sends Marccd command
//-----------------------------------------------------

void Camera::_write(std::string cmd_to_send)
{
    DEB_MEMBER_FUNCT();

    try
    {
        size_t data_size = cmd_to_send.size() + 1; // +1 for '\0' character
        m_sock.send(cmd_to_send.c_str(), data_size);

        if (cmd_to_send != "get_state")
            DEB_TRACE() << "_write() [" << cmd_to_send << "]";
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "_write : Unknown Exception";
    }
}

//-----------------------------------------------------
// - Sends a command and reads response
//-----------------------------------------------------

std::string Camera::_sendCmdAndReceiveAnswer(std::string cmd_to_send)
{
    DEB_MEMBER_FUNCT();

    std::string response("");
    try
    {
        //- send command
        _write(cmd_to_send);
        //- check if command needs a response
        if (cmd_to_send.find("get_") != std::string::npos)
        {
            response = _read();
            if (cmd_to_send != "get_state")
                DEB_TRACE() << "_read() [" << response << "]";
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "_sendCmdAndReceiveAnswer : Unknown Exception";
    }
    return response;
}

//-----------------------------------------------------
// - _performAcquisitionSequence : manage a Marccd acquisition
//-----------------------------------------------------

void Camera::_performAcquisitionSequence()
{
    DEB_MEMBER_FUNCT();
    static bool is_first_trace = true;
    static bool is_latency_started = false;
    static bool is_exposure_started = false;

    m_first_image = m_image_number;
    int step = 0;
    int last_step = 0;

    //FOR TRACES ONLY
    struct timespec now;

    //DO WHILE ALL NB_FRAME are NOT ACQUIRED
    while (m_image_number < (m_first_image + m_nb_frames))
    {

        //FOR TRACES ONLY
        if (last_step != step)
        {
            last_step = step;
            clock_gettime(CLOCK_MONOTONIC, &now);
            DEB_TRACE() << "Step " << step << " (time =  " << now.tv_sec << "." << now.tv_nsec << ")\n";
        }

        //------------------------------------------------------------------------------------------------------------
        // acqusition process state machine
        //------------------------------------------------------------------------------------------------------------ 
        switch (step)
        {
                //------------------------------------------------------------
                //- Prepare acquisition (STOP ALL TIMERS, RESET FLAGS, ... )
            case PREPARE_STEP:
            {
                DEB_TRACE() << "Prepare acquisition (Stop all Timers/...)";
                m_exposure_timer.stop();
                m_latency_timer.stop();
                is_exposure_started = false;
                is_latency_started = false;
                step = WAIT_ACQUIRE_IDLE_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Wait for acquisition to be IDLE
            case WAIT_ACQUIRE_IDLE_STEP:
            {
                if (is_first_trace)
                {
                    is_first_trace = false;
                    DEB_TRACE() << "Wait for acquisition to be IDLE ...";
                }
                _updateMarccdState();
                THROW_ON_STATUS_ERROR(m_marccd_state, "_performAcquisitionSequence (Wait before start) ");
                if (TEST_TASK_STATUS(m_marccd_state, TASK_ACQUIRE, TASK_STATUS_EXECUTING) ||
                    TASK_STATE(m_marccd_state) >= 8)
                    break;
                is_first_trace = true;
                step = SEND_START_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Send start command to the marccd
            case SEND_START_STEP:
            {
                DEB_TRACE() << "Send start command to the marccd";
                std::string cmd_to_send("start");
                {
                    yat::MutexLock scoped_lock(m_lock);
                    _sendCmdAndReceiveAnswer(cmd_to_send);
                }
                step = WAIT_ACQUIRE_EXECUTING_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Wait for acquisition to be started              
            case WAIT_ACQUIRE_EXECUTING_STEP:
            {
                if (is_first_trace)
                {
                    is_first_trace = false;
                    DEB_TRACE() << "Wait for acquisition to be started ...";
                }
                _updateMarccdState();
                THROW_ON_STATUS_ERROR(m_marccd_state, "_performAcquisitionSequence (Wait after start) ");
                if (!TEST_TASK_STATUS(m_marccd_state, TASK_ACQUIRE, TASK_STATUS_EXECUTING) ||
                    TASK_STATE(m_marccd_state) >= 8)
                    break;
                is_first_trace = true;
                step = WAIT_EXPOSURE_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Wait for the exposure time
            case WAIT_EXPOSURE_STEP:
            {
                if (is_first_trace)
                {
                    DEB_TRACE() << "Wait for the Exposure time ...";
                    is_first_trace = false;
                }


                //start the timer Exposure    
                if (!is_exposure_started)
                {
                    DEB_TRACE() << "Start the timer Exposure";
                    m_exposure_timer.start(m_exp_time, 1, 0);
                    is_exposure_started = true;
                }

                //if command stop is occured, then stop the exposure timer
                {
                    yat::MutexLock scoped_lock(m_lock_flag);
                    if (m_is_stop_in_progress)
                    {
                        DEB_TRACE() << "Stop the timer Exposure due to a user command Stop !";
                        m_exposure_timer.stop();
                    }
                }

                //Check if event falling edge is raised on timer exposure
                if (!m_exposure_cb.isFallingOccured())
                {
                    sleep(0.1);
                    break;
                }

                is_first_trace = true;
                step = SEND_READOUT_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Send readout command with a specific file name to the marccd
            case SEND_READOUT_STEP:
            {
                DEB_TRACE() << "Send readout command with a specific file name to the marccd";
                std::string cmd_to_send;

                {
                    yat::MutexLock scoped_lock(m_lock_flag);
                    if (!m_is_bg_saving_requested)
                    {
                        cmd_to_send = "readout,0,";
                    }
                    else
                    {
                        cmd_to_send = "readout,1,";
                    }
                }

                std::stringstream ssEntry;
                ssEntry << m_image_path << m_image_name << "_" << m_image_number;
                cmd_to_send += (ssEntry.str());

                {
                    yat::MutexLock scoped_lock_1(m_lock);
                    yat::MutexLock scoped_lock_2(m_lock_flag);
                    _sendCmdAndReceiveAnswer(cmd_to_send);
                    m_is_bg_saving_requested = false;
                }

                step = WAIT_WRITING_END_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Wait for Marccd until writing has finished                
            case WAIT_WRITING_END_STEP:
            {
                if (is_first_trace)
                {
                    is_first_trace = false;
                    DEB_TRACE() << "Wait for Marccd until writing has finished ...";
                }
                _updateMarccdState();
                THROW_ON_STATUS_ERROR(m_marccd_state, "_performAcquisitionSequence (Wait for writting) ");
                if (TEST_TASK_STATUS(m_marccd_state, TASK_WRITE, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) ||
                    TASK_STATE(m_marccd_state) >= 8)
                    break;

                is_first_trace = true;
                step = WAIT_LATENCY_STEP;
            }
                break;

                //------------------------------------------------------------
                //- Wait for the Latency time                
            case WAIT_LATENCY_STEP:
            {
                if (is_first_trace)
                {
                    DEB_TRACE() << "Wait for the Latency time ...";
                    is_first_trace = false;
                }

                //start the timer Latency    
                if (!is_latency_started)
                {
                    DEB_TRACE() << "Start the timer Latency";
                    m_latency_timer.start(m_lat_time, 1, 0);
                    is_latency_started = true;
                }

                //Check if event falling edge is raised on timer lantecy
                if (!m_latency_cb.isFallingOccured())
                {
                    sleep(0.1);
                    break;
                }
                m_image_number++;
                is_first_trace = true;
                step++;
            }
                break;

                //------------------------------------------------------------
                //- Otherwise, GO to Next frame if necessary           
            default:
                DEB_TRACE() << "-------------------------------------------------------";
                DEB_TRACE() << "Next Frame if necessary";
                step = PREPARE_STEP;
                break;

        } //end switch 
    } // end while

}

//-----------------------------------------------------
// - _performBackgroundFrame : sequence to take a background (dark) frame
//-----------------------------------------------------

void Camera::_performBackgroundFrame()
{
    DEB_MEMBER_FUNCT();
    //readout 1
    //-----------------------------
    _readoutFrame(1);

    //readout 2
    //-----------------------------
    _readoutFrame(2);

    /* DEZINGER */
    //-----------------------------
    DEB_TRACE() << "\ndezinger Management :";
    DEB_TRACE() << "----------------------";
    //- Send dezinger 1 => use and store into the current background frame
    DEB_TRACE() << "Send dezinger 1 => use and store into the current background frame";
    std::stringstream cmd_to_send("dezinger,1");
    {
        yat::MutexLock scoped_lock(m_lock);
        _sendCmdAndReceiveAnswer(cmd_to_send.str());
    }

    /* wait for DEZINGER+EXECUTING */
    DEB_TRACE() << "Wait for DEZINGER + EXECUTING";
    _updateMarccdState();
    THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait dezinger+executing) ");
    while (TEST_TASK_STATUS(m_marccd_state, TASK_DEZINGER, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) ||
        TASK_STATE(m_marccd_state) >= 8)
    {

        sleep(MARCCD_POLL_DELAY);
        _updateMarccdState();
        THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait dezinger+executing) ");
    }

    /* inform that background is done */

    {
        yat::MutexLock scoped_lock(m_lock_flag);
        m_is_bg_acquisition_in_progress = false;
    }

}

//-----------------------------------------------------
// - _performStopSequence : stop Marccd acquisition
//-----------------------------------------------------

void Camera::_performStopSequence()
{
    DEB_MEMBER_FUNCT();

    {
        yat::MutexLock scoped_lock(m_lock_flag);
        m_is_stop_in_progress = false;
    }
}


//-----------------------------------------------------
// - _readoutFrame : do readout/correction used to perform backgroud frame
//-----------------------------------------------------

void Camera::_readoutFrame(unsigned bufferNumber)
{
    DEB_MEMBER_FUNCT();
    //readout 1
    //-----------------------------
    DEB_TRACE() << "\nreadout," << bufferNumber << " Management :";
    DEB_TRACE() << "----------------------";
    /* Wait for the readout task to be done with the previous frame, if any */
    DEB_TRACE() << "Wait for the readout task to be done with the previous frame, if any";
    _updateMarccdState();
    THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait before readout) ");
    while (TEST_TASK_STATUS(m_marccd_state, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED) ||
        TASK_STATE(m_marccd_state) >= 8)
    {
        sleep(MARCCD_POLL_DELAY);
        _updateMarccdState();
        THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait before readout) ");
    }


    /* Send readout  => _read data into background frame storage */
    DEB_TRACE() << "Send readout," << bufferNumber << " => _read data into background frame storage";
    std::stringstream cmd_to_send;
    cmd_to_send << "readout" << "," << bufferNumber;
    {
        yat::MutexLock scoped_lock(m_lock);
        _sendCmdAndReceiveAnswer(cmd_to_send.str());
    }

    /* Wait for the readout to start */
    DEB_TRACE() << "Wait for the readout to start";
    _updateMarccdState();
    THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait start readout) ");
    while (!TEST_TASK_STATUS(m_marccd_state, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED))
    {
        sleep(MARCCD_POLL_DELAY);
        _updateMarccdState();
        THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait start readout) ");
    }

    /* Wait for the readout to complete */
    DEB_TRACE() << "Wait for the readout to complete";
    _updateMarccdState();
    THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait complete readout) ");
    while (TEST_TASK_STATUS(m_marccd_state, TASK_READ, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED))
    {
        sleep(MARCCD_POLL_DELAY);
        _updateMarccdState();
        THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait complete readout) ");
    }

    /* Wait for the correction complete */
    DEB_TRACE() << "Wait for the correction complete";
    _updateMarccdState();
    THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait correction readout) ");
    while (TEST_TASK_STATUS(m_marccd_state, TASK_CORRECT, TASK_STATUS_EXECUTING | TASK_STATUS_QUEUED))
    {
        sleep(MARCCD_POLL_DELAY);
        _updateMarccdState();
        THROW_ON_STATUS_ERROR(m_marccd_state, "_performBackgroundFrame (Wait correction readout) ");
    }

}

//-----------------------------------------------------
// - _updateMarccdState : _read the state from the detector
//-----------------------------------------------------

void Camera::_updateMarccdState()
{
    DEB_MEMBER_FUNCT();
    std::string stateStr("");
    //- get detector state string value
    try
    {
        _sendCmdAndReceiveAnswer("get_state");
        stateStr = _sendCmdAndReceiveAnswer("get_state");
        //- convert state string to numeric val
        char data_to_conv[stateStr.size() + 1];
        ::strncpy(data_to_conv, stateStr.c_str(), stateStr.size());

        {
            yat::MutexLock scoped_lock(m_lock);
            m_marccd_state = _convertStringToInt(data_to_conv);
        }
    }
    catch (yat::SocketException & ySe)
    {
        std::stringstream ssError;
        for (unsigned i = 0; i < ySe.errors.size(); i++)
        {
            ssError << ySe.errors[i].desc << std::endl;
        }

        THROW_HW_ERROR(Error) << ssError.str();
    }
    catch (...)
    {
        THROW_HW_ERROR(Error) << "_updateMarccdState : Unknown Exception";
    }
}

//-----------------------------------------------------
// - setImagePath
//-----------------------------------------------------

void Camera::setImagePath(const std::string& imgPath)
{
    m_image_path = imgPath;
}

//-----------------------------------------------------
// - getImagePath
//-----------------------------------------------------

const std::string& Camera::getImagePath(void)
{
    return m_image_path;
}

//-----------------------------------------------------
// - setImageFileName
//-----------------------------------------------------

void Camera::setImageFileName(const std::string& imgName)
{
    m_image_name = imgName;
}

//-----------------------------------------------------
// - getImageFileName
//-----------------------------------------------------

const std::string& Camera::getImageFileName(void)
{
    return m_image_name;
}


//-----------------------------------------------------
// - setImageIndex
//-----------------------------------------------------

void Camera::setImageIndex(int newImgIdx)
{
    m_image_number = newImgIdx;
}

//-----------------------------------------------------
// - getImageIndex
//-----------------------------------------------------

int Camera::getImageIndex()
{
    return m_image_number;
}

//-----------------------------------------------------
// - getFirstImage
//-----------------------------------------------------

int Camera::getFirstImage()
{
    return m_first_image;
}

//-----------------------------------------------------
// - saveBGFrame
//-----------------------------------------------------

void Camera::saveBGFrame(bool BG)
{
    {
        yat::MutexLock scoped_lock(m_lock_flag);
        m_is_bg_saving_requested = BG;
    }
}

//-----------------------------------------------------
// - _convertStringToInt
//-----------------------------------------------------

int Camera::_convertStringToInt(char* text)
{
    int intval = 0;

    if (sscanf(text, "0x%x", &intval) > 0)
        return strtoul(text, NULL, 16);
    else
        return atoi(text);
}

//-----------------------------------------------------
