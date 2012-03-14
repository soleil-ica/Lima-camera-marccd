
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

const size_t kTASK_PERIODIC_TIMEOUT_MS	 =  1000;
const double kDEFAULT_READER_TIMEOUT_SEC = 60.;
const size_t MARCCD_START_MSG     =   (yat::FIRST_USER_MSG + 300);
const size_t MARCCD_RESET_MSG     =   (yat::FIRST_USER_MSG + 302);

//---------------------------
//- Ctor
//---------------------------
Reader::Reader(Camera& cam, HwBufferCtrlObj& buffer_ctrl)
      : _cam(cam),
        _buffer(buffer_ctrl),
				_image_number(0),
				_image_size(0),
				_currentImgFileName(""),
				_simulated_image(0),
				_is_reader_open_image_file(true),	//- read image from file (no simulated image)
				_tmOut(0)
{
	DEB_CONSTRUCTOR();
}

//---------------------------
//- Dtor
//---------------------------
Reader::~Reader()
{
	DEB_DESTRUCTOR();
	if ( this->_tmOut )
	{
		delete _tmOut;
		_tmOut = 0;
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
	yat::MutexLock scoped_lock(_lock);
	return this->_image_number;        
}

//---------------------------
//- Reader::isTimeoutSignaled()
//---------------------------
bool Reader::isTimeoutSignaled()
{
	bool expired = false;
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	if( this->_tmOut )
		expired = this->_tmOut->expired();
	else
		throw LIMA_HW_EXC(Error, "Reader initialisation failed : cannot get timeout state.");

	return expired;
}

//---------------------------
//- Reader::isRunning()
//---------------------------
bool Reader::isRunning(void)
{
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	return this->periodic_msg_enabled();
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::setTimeout(double newTimeOutVal)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(newTimeOutVal);
	yat::MutexLock scoped_lock(_lock);
	if ( this->_tmOut )
		this->_tmOut->set_value(newTimeOutVal);
	else
		throw LIMA_HW_EXC(Error, "Reader initialisation failed : cannot set new timeout value.");
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::enableReader(void)
{
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	this->_is_reader_open_image_file = true;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void Reader::disableReader(void)
{
	DEB_MEMBER_FUNCT();
	yat::MutexLock scoped_lock(_lock);
	this->_is_reader_open_image_file = false;
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
				//- create timeout
				this->_tmOut = new yat::Timeout();
				//- set unit in seconds
				this->_tmOut->set_unit(yat::Timeout::TMO_UNIT_SEC);
				//- set default timeout value
				this->_tmOut->set_value(kDEFAULT_READER_TIMEOUT_SEC);
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
				//- check if timeout expired
				if ( this->_tmOut->expired() )
				{
					DEB_TRACE() << "FATAL::Failed to load image : timeout expired !";
					//- disable periodic msg
					this->enable_periodic_msg(false);
					//- disable timeout
					this->_tmOut->disable();
					return;
				}
				//- check if file exist
				std::ifstream imgFile(this->_currentImgFileName.c_str());
				if ( imgFile )
				{
std::cout << "\t\t\t Reader::->imgFile exist ..." << std::endl;
					//- read image file
					this->getImageFromFile();
					//- disable periodic msg
					this->enable_periodic_msg(false);
std::cout << "\t\t\t Reader::->TASK_PERIODIC ... DONE !!!" << std::endl;
				}
				else 
				{ 
					std::cout << "\t\t\t Reader::->imgFile DOES NOT exist ..." << std::endl;
				}
			}
      break;
      //-----------------------------------------------------    
      case MARCCD_START_MSG:    
      {
        DEB_TRACE() << "Reader::->MARCCD_START_MSG";
				//- get full image name as full/path/imgName_idx
				this->_currentImgFileName = this->_cam.getFullImgName();
				//- enable periodic msg
        this->enable_periodic_msg(true);
				//- re-arm timeout
				this->_tmOut->restart();
std::cout << "\t\t\t Reader::->MARCCD_START_MSG DONE for image name : " << this->_currentImgFileName << std::endl;
      }
      break;
      //-----------------------------------------------------
      case MARCCD_RESET_MSG:
      {
        DEB_TRACE() << "Reader::->MARCCD_RESET_MSG";
        this->enable_periodic_msg(false);
        this->_image_number = 0;
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
	this->_cam.getImageSize(_image_size);

	StdBufferCbMgr& buffer_mgr = ((reinterpret_cast<BufferCtrlObj&>(this->_buffer)).getBufferCbMgr());
	int buffer_nb, concat_frame_nb;        
	
	buffer_mgr.setStartTimestamp(Timestamp::now());
	buffer_mgr.acqFrameNb2BufferNb(this->_image_number, buffer_nb, concat_frame_nb);

	void *ptr = buffer_mgr.getBufferPtr(buffer_nb, concat_frame_nb);

	DI::DiffractionImage tmpDI( const_cast<char*>(this->_currentImgFileName.c_str()) );
	//tmpDI.open(const_cast<char*>(this->_currentImgFileName.c_str()));

	//std::cout << "\t\t\tm_image_size.getWidth() = " << m_image_size.getWidth() << " & m_image_size.getHeight() = " << m_image_size.getHeight() << std::endl;
	//std::cout << "\t\t\tm_DI->getWidth() = " << m_DI->getWidth() << " & m_DI->getHeight() = " << m_DI->getHeight() << std::endl;

	if( this->_image_size.getWidth() != tmpDI.getWidth() || this->_image_size.getHeight() != tmpDI.getHeight())
		throw LIMA_HW_EXC(Error, "Image size in file is different from the expected image size of this detector !");

	int img_size = tmpDI.getWidth()*tmpDI.getHeight();
	unsigned int * image = 0;
	image = tmpDI.getImage();

	for(int j=0;j<img_size;j++)
	{
		((uint16_t*)ptr)[j] = (uint16_t)image[j];
	}

	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = this->_image_number++;
	buffer_mgr.newFrameReady(frame_info);
	std::cout << "\t\t\tReader::getImageFromFile -> new frame ready" << std::endl;                     
}

