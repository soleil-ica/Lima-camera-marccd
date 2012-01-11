
#include <yat/threading/Mutex.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include "Debug.h"
#include "Data.h"
#include "MarccdReader.h"
#include "MarccdInterface.h"


#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

#define kTASK_PERIODIC_TIMEOUT_MS   1000
const size_t  MARCCD_START_MSG     =   (yat::FIRST_USER_MSG + 300);
const size_t  MARCCD_RESET_MSG     =   (yat::FIRST_USER_MSG + 302);

//---------------------------
//- Ctor
//---------------------------
Reader::Reader(Camera& cam, HwBufferCtrlObj& buffer_ctrl)
      : m_cam(cam),
        m_buffer(buffer_ctrl),
				_currentImgFileName(""),
				_previousImgFileName(""),
        m_DI(0)
{
	DEB_CONSTRUCTOR();
	try
	{
		m_image_number = 0;
		enable_timeout_msg(false);
		enable_periodic_msg(false);
		set_periodic_msg_period(kTASK_PERIODIC_TIMEOUT_MS);
		m_cam.getImageSize(m_image_size);
	}
	catch (Exception &e)
	{
		// Error handling
		DEB_ERROR() << e.getErrMsg();
		throw LIMA_HW_EXC(Error, e.getErrMsg());
	}    
}

//---------------------------
//- Dtor
//---------------------------
Reader::~Reader()
{
	DEB_DESTRUCTOR();
	try
	{
		if( m_DI )
		{
			delete m_DI;
			m_DI = 0;
		}
	}
	catch (Exception &e)
	{
		// Error handling
		DEB_ERROR() << e.getErrMsg();
		throw LIMA_HW_EXC(Error, e.getErrMsg());
	}
}

//---------------------------
//- Reader::start()
//---------------------------
void Reader::start()
{
	DEB_MEMBER_FUNCT();
	try
	{
		this->post(new yat::Message(MARCCD_START_MSG), kPOST_MSG_TMO);
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
		this->post(new yat::Message(MARCCD_RESET_MSG), kPOST_MSG_TMO);
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
    {
        yat::MutexLock scoped_lock(contextual_lock_);
        return m_image_number;        
    }
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::handle_message( yat::Message& msg )  throw( yat::Exception )
{
  DEB_MEMBER_FUNCT();
  try
  {
    switch ( msg.type() )
    {
      //-----------------------------------------------------    
      case yat::TASK_INIT:
      {
        DEB_TRACE() <<"Reader::->TASK_INIT";
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_EXIT:
      {
        DEB_TRACE() <<"Reader::->TASK_EXIT";
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_TIMEOUT:
      {
        DEB_TRACE() <<"Reader::->TASK_TIMEOUT";
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_PERIODIC:
      {
				DEB_TRACE() <<"Reader::->TASK_PERIODIC";
				//- check camera state : stop must be done
				if ( m_cam.is_stop_sequence_finished() )
				{
std::cout << "\t\t\t Reader::->stop_sequence_finished ...\n"
 << "\t\t\t Reader::-> this->_currentImgFileName = " << this->_currentImgFileName << "\n"
 << "\t\t\t Reader::-> this->_previousImgFileName = "<< this->_previousImgFileName
<< std::endl;
					//- is it a new image ?
					if ( this->_currentImgFileName != this->_previousImgFileName )
					{
std::cout << "\t\t\t Reader::->new image ..." << std::endl;
						//- check if file exist
						std::ifstream imgFile(this->_currentImgFileName.c_str());
						if ( imgFile )
						{
std::cout << "\t\t\t Reader::->imgFile exist ..." << std::endl;
							//- read image file
							this->getImageFromFile();
							this->_previousImgFileName = this->_currentImgFileName;
							//- disable periodic msg
							this->enable_periodic_msg(false);
std::cout << "\t\t\t Reader::->TASK_PERIODIC ... DONE !!!" << std::endl;
						}
            else { std::cout << "\t\t\t Reader::->imgFile DOES NOT exist ..." << std::endl;}

					}
				}
      }
      break;
      //-----------------------------------------------------    
      case MARCCD_START_MSG:    
      {
        DEB_TRACE() << "Reader::->MARCCD_START_MSG";
				//- get full image name as full/path/imgName_idx
				this->_currentImgFileName = this->m_cam.getFullImgName();
				//- get image file name
//				this->_currentImgFileName += this->m_cam.getImageFileName();
				//- enable periodic msg
        this->enable_periodic_msg(true);
std::cout << "\t\t\t Reader::->MARCCD_START_MSG DONE for image name : " << this->_currentImgFileName << std::endl;
      }
      break;
      //-----------------------------------------------------
      case MARCCD_RESET_MSG:
      {
        DEB_TRACE() << "Reader::->MARCCD_RESET_MSG";
        this->enable_periodic_msg(false);
        m_image_number = 0;
      }
      break;    
      //-----------------------------------------------------
    }
  }
  catch( yat::Exception& ex )
  {
    DEB_ERROR() <<"Error : " << ex.errors[0].desc;
    throw;
  }
}
//-----------------------------------------------------
void Reader::getImageFromFile ()
{
	//- update image info
	m_cam.getImageSize(m_image_size);

	//DEB_TRACE() <<"vecNewAndChangedFiles = "<< vecNewAndChangedFiles.at(i)->FullName();
	StdBufferCbMgr& buffer_mgr = ((reinterpret_cast<BufferCtrlObj&>(m_buffer)).getBufferCbMgr());
	int buffer_nb, concat_frame_nb;        
	buffer_mgr.setStartTimestamp(Timestamp::now());
	buffer_mgr.acqFrameNb2BufferNb(m_image_number, buffer_nb, concat_frame_nb);
	std::cout << "\t\t\tReader::getImageFromFile -> m_image_number = " << m_image_number 
		<< " & buffer_nb = " << buffer_nb
		<< " & concat_frame_nb = " << concat_frame_nb
		<< std::endl;

	void *ptr = buffer_mgr.getBufferPtr(buffer_nb, concat_frame_nb);
	std::cout << "\t\t\tReader::getImageFromFile -> buffer_nb = " << buffer_nb << " & concat_frame_nb = " << concat_frame_nb << std::endl;
	//DEB_TRACE()  << "file : " << vecNewAndChangedFiles.at(i)->FullName();

	//DEB_TRACE()<<"-- Read an image using DI";
	m_DI = new DI::DiffractionImage(const_cast<char*>(this->_currentImgFileName.c_str()));
	//DEB_TRACE()<<"-- Read an image using DI -> new DI DONE";

	std::cout << "\t\t\tm_image_size.getWidth() = " << m_image_size.getWidth() << " & m_image_size.getHeight() = " << m_image_size.getHeight() << std::endl;
	std::cout << "\t\t\tm_DI->getWidth() = " << m_DI->getWidth() << " & m_DI->getHeight() = " << m_DI->getHeight() << std::endl;

	if(m_image_size.getWidth()!=m_DI->getWidth() || m_image_size.getHeight()!=m_DI->getHeight())
		throw LIMA_HW_EXC(Error, "Image size in file is different from the expected image size of this detector !");

	//DEB_TRACE()<<"-- prepare memory with image data"; 

	std::cout << "\t\t\tvoid *ptr -> done : buffer_nb = " << buffer_nb
		<< " concat_frame_nb = " << concat_frame_nb
		<< " m_image_number = " << m_image_number
		<< std::endl;
	int img_size = m_DI->getWidth()*m_DI->getHeight();
	unsigned int * image = 0;
	std::cout << "\t\t\tgetting image ..." << std::endl;
	image = m_DI->getImage();
	std::cout << "\t\t\tgetting image DONE image : " << image << std::endl;

	for(int j=0;j<img_size;j++)
	{
		((uint16_t*)ptr)[j] = (uint16_t)image[j];
	}
	std::cout << "\t\t\tloop on DI done" << std::endl;

	if ( m_DI )
	{
		delete m_DI;
		m_DI = 0;
	}
	std::cout << "\t\t\tdelete m_DI -> done " << std::endl;

	//DEB_TRACE()<<"-- newFrameReady";                      
	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = m_image_number++;
	bool continueAcq = buffer_mgr.newFrameReady(frame_info);
	std::cout << "\t\t\tnew frame ready" << std::endl;                     
	int nb_frames = 0;
	m_cam.getNbFrames(nb_frames);
}

