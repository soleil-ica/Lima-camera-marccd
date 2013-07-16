#include <yat/threading/Mutex.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <time.h>
#include <sys/stat.h>
#include "Debug.h"
#include "Constants.h"
#include "Data.h"
#include "MarccdReader.h"
#include "MarccdInterface.h"
#include "errno.h"

#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

const size_t kTASK_PERIODIC_TIMEOUT_MS = 1000;
const double kDEFAULT_READER_TIMEOUT_SEC = 10.;
const size_t MARCCD_START_MSG = (yat::FIRST_USER_MSG + 300);
const size_t MARCCD_RESET_MSG = (yat::FIRST_USER_MSG + 302);

//---------------------------
//- Ctor
//---------------------------

Reader::Reader(Camera& cam, HwBufferCtrlObj& buffer_ctrl)
: yat::Task(Config(false, //- disable timeout msg
                   1000, //- every second (i.e. 1000 msecs)
                   true, //- enable periodic msgs
                   1000, //- every second (i.e. 1000 msecs)
                   false, //- don't lock the internal mutex while handling a msg (recommended setting)
                   kDEFAULT_LO_WATER_MARK, //- msgQ low watermark value
                   kDEFAULT_HI_WATER_MARK, //- msgQ high watermark value
                   false, //- do not throw exception on post msg timeout (msqQ saturated)
                   0)), //- user data (same for all msgs) - we don't use it here
m_cam(cam),
m_buffer(buffer_ctrl),
m_image_number(0),
m_current_image_file_name(""),
m_timeout_value(kDEFAULT_READER_TIMEOUT_SEC)
{
    DEB_CONSTRUCTOR();
}

//---------------------------
//- Dtor
//---------------------------

Reader::~Reader()
{
    DEB_DESTRUCTOR();
}

//---------------------------
//- Reader::start()
//---------------------------

void Reader::start()
{
    DEB_MEMBER_FUNCT();

    try
    {
        double eTime, lTime;
        m_cam.getExpTime(eTime);
        m_cam.getLatTime(lTime);
        
        yat::MutexLock scoped_lock(m_lock);
        m_timeout.set_value(m_timeout_value + eTime + lTime);        

        post(new yat::Message(MARCCD_START_MSG), kPOST_MSG_TMO);
    }
    catch (Exception &e)
    {
        // Error handling
        DEB_ERROR() << e.getErrMsg();
        throw LIMA_HW_EXC(Error, e.getErrMsg());
    }
}

//---------------------------
//- Reader::reset()
//---------------------------

void Reader::reset()
{
    DEB_MEMBER_FUNCT();
    try
    {
        //- disable timeout
        m_timeout.disable();
        post(new yat::Message(MARCCD_RESET_MSG), kPOST_MSG_TMO);
    }
    catch (Exception &e)
    {
        // Error handling
        DEB_ERROR() << e.getErrMsg();
        throw LIMA_HW_EXC(Error, e.getErrMsg());
    }

}

//---------------------------
//- Reader::getLastAcquiredFrame()
//---------------------------

int Reader::getLastAcquiredFrame(void)
{
    yat::MutexLock scoped_lock(m_lock);
    // m_image_number corresponds to the image the reader looks for.
    return (m_image_number - m_cam.getFirstImage());
}

//---------------------------
//- Reader::isTimeoutSignaled()
//---------------------------

bool Reader::isTimeoutSignaled()
{
    DEB_MEMBER_FUNCT();
    yat::MutexLock scoped_lock(m_lock);
    return m_timeout.expired();
}

//---------------------------
//- Reader::isRunning()
//---------------------------

bool Reader::isRunning(void)
{
    DEB_MEMBER_FUNCT();
    yat::MutexLock scoped_lock(m_lock);
    return periodic_msg_enabled();
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Reader::setTimeout(double newTimeOutVal)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(newTimeOutVal);
    yat::MutexLock scoped_lock(m_lock);
    m_timeout_value = newTimeOutVal;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

int* Reader::getHeader(void)
{
    DEB_MEMBER_FUNCT();
    return hccd.data;
}

//-----------------------------------------------------
//
//-----------------------------------------------------

void Reader::handle_message(yat::Message& msg) throw ( yat::Exception)
{
    DEB_MEMBER_FUNCT();

    try
    {
        switch (msg.type())
        {
                //-----------------------------------------------------    
            case yat::TASK_INIT:
            {
                DEB_TRACE() << "Reader::->TASK_INIT";
                //- set unit in seconds
                m_timeout.set_unit(yat::Timeout::TMO_UNIT_SEC);
                //- set default timeout value
                m_timeout.set_value(kDEFAULT_READER_TIMEOUT_SEC);
            }
                break;
                //-----------------------------------------------------    
            case yat::TASK_EXIT:
            {
                DEB_TRACE() << "Reader::->TASK_EXIT";
            }
                break;
                //-----------------------------------------------------    
            case yat::TASK_TIMEOUT:
            {
                DEB_TRACE() << "Reader::->TASK_TIMEOUT";
            }
                break;
                //-----------------------------------------------------    
            case yat::TASK_PERIODIC:
            {
                DEB_TRACE() << "Reader::->TASK_PERIODIC";

                //- check if timeout expired
                if (m_timeout.expired())
                {
                    DEB_TRACE() << "FATAL::Failed to load image : timeout expired !";

                    //- disable periodic msg
                    enable_periodic_msg(false);
                    return;
                }

                //- get full image name as full/path/imgName_idx
                std::stringstream newFileName;
                newFileName << m_cam.getImagePath() << m_cam.getImageFileName() << "_" << m_image_number;

                // Force nfs file system to refresh
                std::stringstream lsCommand;
                lsCommand << "ls " << m_cam.getImagePath();
                system(lsCommand.str().c_str());

                //- check if file exist
                std::ifstream imgFile(newFileName.str().c_str());

                if (imgFile && m_current_image_file_name != newFileName.str())
                {

                    m_current_image_file_name = newFileName.str();
                    DEB_TRACE() << "Reader: Found File [" << newFileName.str() << "]";
					//- disable timeout
                    m_timeout.disable();
                    
                    //@@TODO : Check why we need this, otherwise unable to open the file !!
                    clock_t wait = 5. /1* CLOCKS_PER_SEC + clock();
                    while (wait > clock());
                    //

                    //- _read image file
                    getImageFromFile(m_current_image_file_name);

                    int nb_frames;
                    m_cam.getNbFrames(nb_frames);

                    // Wait for more frames? 
                    if (++m_image_number < (m_cam.getFirstImage() + nb_frames))
                    {
                        DEB_TRACE()<<"Reader: Wait for more frames";  
                        m_timeout.restart();
                    }
                    else
                    {
                        DEB_TRACE()<<"Reader: No more frames";                        
                        //- disable periodic msg
                        m_timeout.disable();
                        enable_periodic_msg(false);
                    }
                }
                else
                {
                    // Fix possible m_image_number de-synchronization at START
                    if (m_cam.getFirstImage() > m_image_number)
                        m_image_number = m_cam.getFirstImage();
                    DEB_TRACE() << "Reader: Image File Not Found [" << newFileName.str() << "]";
                }
            }
                break;
                //-----------------------------------------------------    
            case MARCCD_START_MSG:
            {
                DEB_TRACE() << "Reader::->MARCCD_START_MSG";
                //- enable periodic msg
                enable_periodic_msg(true);
                // Next image we are waiting for
                m_image_number = m_cam.getFirstImage();
                //- re-arm timeout
                m_timeout.restart();
                m_current_image_file_name = string("");
            }
                break;
                //-----------------------------------------------------
            case MARCCD_RESET_MSG:
            {
                DEB_TRACE() << "Reader::->MARCCD_RESET_MSG";
                enable_periodic_msg(false);
                m_image_number = m_cam.getImageIndex();
                // Clean buffer
                std::stringstream rmCommand;
                rmCommand << "rm " << m_cam.getImagePath() << m_cam.getImageFileName() << "_* >& /dev/null"; // avoid print out
                DEB_TRACE()<<rmCommand.str(); 
                system(rmCommand.str().c_str());
            }
                break;
                //-----------------------------------------------------
        }
    }
    catch (yat::Exception& ex)
    {
        DEB_ERROR() << "Error : " << ex.errors[0].desc;
        throw;
    }
}
//-----------------------------------------------------

bool Reader::getImageFromFile(const std::string& fileName)
{
    DEB_MEMBER_FUNCT();
    StdBufferCbMgr& buffer_mgr = ((reinterpret_cast<BufferCtrlObj&> (m_buffer)).getBufferCbMgr());

    // Buffer manager is very strict with the data pointer, 
    // it will accept only its own pointer address
    int frameNumber = m_image_number - m_cam.getFirstImage();
    void *ptr = buffer_mgr.getFrameBufferPtr(frameNumber);

    buffer_mgr.setStartTimestamp(Timestamp::now());

    HwFrameInfoType frame_info;
    frame_info.acq_frame_nb = frameNumber;

    // Open the Image file
    DEB_TRACE() << "Reader: Open the Image File";
    FILE * file;
    file = fopen(fileName.c_str(), "rb");
    if (file != NULL)
    {
        Size frameSize = buffer_mgr.getFrameDim().getSize();
		DEB_TRACE()<<"Frame Width = "<<frameSize.getWidth();
		DEB_TRACE()<<"Frame Height = "<<frameSize.getHeight();
        int frameMemSize = buffer_mgr.getFrameDim().getMemSize();

        MARCCD_HEADER header;

        /*** TODO Doesn't work
        struct stat st;
        // MARCCD format is TIFF header (1k) + MARCCD_HEADER (3k)+ RAW data (16bit depth)
        // get informations about file (stat) in order to check its validity
        if (fstat(fileno(file), &st) != -1)
        {
            fclose(file);
            DEB_ERROR()<<"Cannot access image file status ! - "<<strerror(errno);            
            throw LIMA_HW_EXC(Error, "Cannot access image file status !");
        }
        
        // Check that the image size corresponds with the expected frame size        
        if (st.st_size != (int) (sizeof (MARCCD_HEADER) + 1024 + frameMemSize))
        {
            fclose(file);
            DEB_ERROR()<<"Image size in file is different from the expected image size of this detector !";            
            throw LIMA_HW_EXC(Error, "Image size in file is different from the expected image size of this detector !");
        }
        ***/
        
        // get the Image header
        DEB_TRACE() << "Reader: get the Image Header";
        fseek(file, 1024, SEEK_SET);
        if (fread(&header, sizeof (MARCCD_HEADER), 1, file) == 1)
        {
            //if (hccd.header_byte_order < 1234 || hccd.header_byte_order > 4321)
            //  swaplong((void *) &hccd, sizeof(MARCCD_HEADER));
            hccd.header = header;
        }
        else
        {
            fclose(file);
            DEB_ERROR()<<"Error reading the MARCCD_HEADER from file !";           
            throw LIMA_HW_EXC(Error, "Error reading the MARCCD_HEADER from file !");
        }
        
        // get the Image raw data !!!
        DEB_TRACE() << "Reader: get the Image RAW Data";
        if (fread(ptr, frameMemSize, 1, file) != 1)
        {
            fclose(file);
            DEB_ERROR()<<"Error reading the RAW Data from file !";            
            throw LIMA_HW_EXC(Error, "Error reading the RAW Data from file !");
        }
        fclose(file);
    }

    DEB_TRACE() << "Reader: Publish the New Frame (FrameNb = " << frameNumber << ")";
    return buffer_mgr.newFrameReady(frame_info);
}

