#include "MarccdCamera.h"
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <yat/utils/XString.h>
#include <DiffractionImage.h>			//- to read back img data

using namespace lima;
using namespace lima::Marccd;
//using namespace std;

//- Define NaN value
const double _NaN_ = ::sqrt(-1.);

//- task period (in ms)
const size_t kPERIODIC_MSG_PERIOD = 500;

const size_t DATA_SIZE   = 16;			//- 16 bits image
const size_t DATA_IMAGE_OFFSET = 4096;	//- image data offset in the Marccd file
//- Task numbers
const size_t TASK_ACQUIRE  =   0;
const size_t TASK_READ     =   1;
const size_t TASK_CORRECT  =   2;
const size_t TASK_WRITE    =   3;
const size_t TASK_DEZINGER =   4;

//- The status bits for each task are:
//- Task Status bits
const size_t TASK_STATUS_QUEUED    = 0x1;
const size_t TASK_STATUS_EXECUTING = 0x2;
const size_t TASK_STATUS_ERROR     = 0x4;
const size_t TASK_STATUS_RESERVED  = 0x8;

//- These are the "old" states from version 0, but BUSY is also used in version 1
const size_t TASK_STATE_IDLE        = 0;
const size_t TASK_STATE_ACQUIRE     = 1;
const size_t TASK_STATE_READOUT     = 2;
const size_t TASK_STATE_CORRECT     = 3;
const size_t TASK_STATE_WRITING     = 4;
const size_t TASK_STATE_ABORTING    = 5;
const size_t TASK_STATE_UNAVAILABLE = 6;
const size_t TASK_STATE_ERROR       = 7;	//- command not understood
const size_t TASK_STATE_BUSY        = 8;	//- interpreting command

//- These are the definitions of masks for looking at task state bits
#define STATE_MASK        0xf
#define STATUS_MASK       0xf
#define TASK_STATUS_MASK(task) (STATUS_MASK << (4*((task)+1)))

//- These are some convenient macros for checking and setting the state of each task
//- They are used in the marccd code and can be used in the client code
#define TASK_STATE(current_status) ((current_status) & STATE_MASK)
#define TASK_STATUS(current_status, task) (((current_status) & TASK_STATUS_MASK(task)) >> (4*((task) + 1)))
#define TEST_TASK_STATUS(current_status, task, status) (TASK_STATUS(current_status, task) & (status))

//- Frame type choices
enum {
    marccdFrameNormal,
    marccdFrameBackground,
    marccdFrameRaw,
    marccdFrameDoubleCorrelation
};

// Buffers for grabbing
//static const uint32_t c_nBuffers = 1;

//---------------------------
//- Ctor
//---------------------------
MarccdCamera::MarccdCamera(const std::string& camera_ip, size_t portNum, const std::string& full_image_path_name)
: yat::Task(),
	_sock(0),
	_current_img_path(full_image_path_name),
	_previous_img_path(""),
	m_buffer_cb_mgr(m_buffer_alloc_mgr),
	m_buffer_ctrl_mgr(m_buffer_cb_mgr),
	_marccd_state(TASK_STATE_IDLE),
	m_status(MarccdCamera::Unknown),
	_image_number(0),
	_stop_already_done(false),
	_camera_ip(camera_ip),
	_port_num(portNum),
	_detector_model(""),
	_detector_type("")
{
	DEB_CONSTRUCTOR();
	std::cout << "MarccdCamera::MarccdCamera() - ENTERING ..." << std::endl;
	std::cout << "MarccdCamera::MarccdCamera() - _current_img_path INIT : " << _current_img_path << std::endl;

	_current_img_path = "/nfs/spool/xavier/imgRAW.00xx";
	_previous_img_path= "/nfs/spool/dt/xavier/imgRAW.00xx";
	std::cout << "MarccdCamera::MarccdCamera() - _current_img_path NOW : " << _current_img_path << std::endl;
	try
	{
    //this->enable_periodic_msg( true );

		_stop_already_done = true;
		m_status = MarccdCamera::Ready;		
		// Create the transport layer object needed to enumerate or
		// create a camera object of type Camera_t::DeviceClass()
		std::cout << "Create a MarccdCamera object of type Camera_t::DeviceClass()" << std::endl;

		_detector_type  = "MARCCD" ;
		_detector_model = "SX 165";

    //- internal YAT stuff cooking
    yat::Socket::init();
    try
    {
      _sock = new yat::ClientSocket ();
    }
    catch (yat::Exception &ye)
    {
			std::cout << "MarccdCamera::MarccdCamera : MarccdCamera::MarccdCamera initialization failed caught DevFailed trying to create yat::ClientSocket" << std::endl;
      //this->status_str = "MarccdCamera::MarccdCamera : device initialization failed caught DevFailed trying to create yat::Socket\n";
      std::cout << " caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
      return;
    }
    catch (...)
    {
      std::cout << "MarccdCamera::MarccdCamera : MarccdCamera initialization failed caught ... trying to create yat::ClientSocket" << std::endl;
      //this->status_str = "MarccdCamera::init_device : device initialization failed caught ... trying to create yat::Socket\n";
      return;
    }

    //- try to connect to Marccd ethernet server HW
    try
    {
      this->connect();
    }
    catch (yat::Exception &ye)
    {
      std::cout << "MarccdCamera::MarccdCamera : could not connect to " << this->_camera_ip <<  " caught DevFailed trying to connect" << std::endl;
      //this->status_str = "MarccdCamera::init_device : could not connect to " + url + " caught DevFailed trying to connect\n";
      std::cout << " caught YAT Exception [" << ye.errors[0].desc << "]" << std::endl;
      return;
    }
    catch (...)
    {
      std::cout << "MarccdCamera::MarccdCamera : could not connect to " << this->_camera_ip <<  " caught ... trying to connect" << std::endl;
      //this->status_str = "MarccdCamera::init_device : could not connect to " + url + " caught ... trying to connect\n";
      return;
    }
	
		std::cout  << "Create the MarccdCamera object attached to ip address : " << this->_camera_ip << std::endl;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCCD CTOR -> An ... exception occurred!" << std::endl;
		//Pylon::PylonTerminate( );
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}	
	std::cout <<"MarccdCamera::MarccdCamera() - DONE" << std::endl;	
}

//---------------------------
//- Dtor
//---------------------------
MarccdCamera::~MarccdCamera()
{
	DEB_DESTRUCTOR();
	std::cout <<"MarccdCamera::~MarccdCamera() - ENTERING ..." << std::endl;	
	try
	{
		//- abort acquisition
		this->write("abort");

    //-  Delete device allocated objects
    this->disconnect ();
		std::cout << "MarccdCamera::~MarccdCamera -> disconnect ... " << std::endl;
    
		//- yat internal cooking
    yat::Socket::terminate();
		std::cout << "MarccdCamera::~MarccdCamera -> terminate ... " << std::endl;
	}
	//- TODO : catch all exceptions !
	catch (...)
	{
		// Error handling
		std::cerr << "MARCCD DTOR -> An ... exception occurred!"  << std::endl;
	}

	std::cout <<"MarccdCamera::~MarccdCamera() - MARCCD DONE" << std::endl;		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::handle_message( yat::Message& msg )  throw( yat::Exception )
{
  try
  {
    switch ( msg.type() )
    {
      //-----------------------------------------------------	
      case yat::TASK_INIT:
      {
        std::cout << "MarccdCamera::->TASK_INIT" << std::endl;          
				
				this->enable_periodic_msg(true);          
				this->set_periodic_msg_period( kPERIODIC_MSG_PERIOD );

				std::cout << "MarccdCamera::->TASK_INIT DONE." << std::endl;          
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_EXIT:
      {
        std::cout << "MarccdCamera::->TASK_EXIT DONE." << std::endl;                
      }
      break;
      //-----------------------------------------------------    
      case yat::TASK_TIMEOUT:
      {
				std::cout << "MarccdCamera::->TASK_TIMEOUT DONE." << std::endl;       
      }
      break;
			//- TASK_PERIODIC ===================
			case yat::TASK_PERIODIC:
			{
				std::cout << "\n\t******MarccdCamera::handle_message entering handling of TASK_PERIODIC msg" << std::endl;

				//- code relative to the task's periodic job goes here
				//yat::Timer t;
				
				//yat::MutexLock scoped_lock(this->_lock);        
				
				this->get_marccd_state();
				
				//this->getStatus(mar_status);
				//- TODO : update Marccd state/status here !???? (period value = ?)

				//std::cout << "\t********** TASK_PERIODIC COMPLETED -> marccd status = " << this->_marccd_state << std::endl;
				//std::cout << "**********\tTIME TO COMPLETE TASK_PERIODIC : " << t.elapsed_msec() << " milliseconds \n" << std::endl;
			}
			break;
			//- USER_DEFINED_MSG ================
      case START_MSG:	
      {
				std::cout << "MarccdCamera::->START_MSG <-" << std::endl;
				
				_stop_already_done = false;
				
				this->perform_start_sequence();
        				
				std::cout << "MarccdCamera::->START_MSG DONE." << std::endl;
				//this->post(new yat::Message(DLL_GET_IMAGE_MSG), kPOST_MSG_TMO);
      }
      break;
      //-----------------------------------------------------
      case GET_IMAGE_MSG:
      {
				std::cout << "MarccdCamera::->GET_IMAGE_MSG" << std::endl;
				
				this->GetImage();
				
				std::cout << "MarccdCamera::->GET_IMAGE_MSG DONE." << std::endl;
      }
      break;	
      //-----------------------------------------------------
      case STOP_MSG:
      {
				std::cout << "MarccdCamera::->STOP_MSG <-" << std::endl;
				
				this->perform_stop_sequence();

				//this->write("abort");
				std::cout << "MarccdCamera::->STOP_MSG DONE." << std::endl;
				//FreeImage();
      }
      break;
      //-----------------------------------------------------
    }
  }
  catch(yat::Exception& ex)	//- XE : Dangerous -> 'cause cannot be caught !!!
  {
		std::cout << "\n\t****** HANDLE_MSG -> Error on YAT exception which desc is : " << ex.errors[0].desc << std::endl;
		//throw;
  }
  catch(...)
  {
		std::cout << "\n\t****** HANDLE_MSG -> Error : Generic Exception caught !!!" << std::endl;
  }
}

//---------------------------
//- MarccdCamera::start()
//---------------------------
void MarccdCamera::start()
{
	DEB_MEMBER_FUNCT();
	std::cout <<"MarccdCamera::start() - ENTERING ..." << std::endl;

	this->_image_number = 0;
	
	//- prepare msg
	yat::Message * msg = new yat::Message(START_MSG, MAX_USER_PRIORITY);
	if ( !msg )
	{
		std::cerr << "MarccdCamera::start FAILED -> yat::Message allocation FAILED!"  << std::endl;
		//- TODO : gestion erreur !???
		return;
	}
	//- don't wait till the message is processed !!
	this->post(msg);
	
	std::cout << "MarccdCamera::start() - MARCCD DONE" << std::endl;
}

//---------------------------
//- MarccdCamera::stop()
//---------------------------
void MarccdCamera::stop()
{
	DEB_MEMBER_FUNCT();
	std::cout << "MarccdCamera::stop() - ENTERING ..." << std::endl;
	
	//- prepare msg
	yat::Message * msg = new yat::Message(STOP_MSG, MAX_USER_PRIORITY);
	if ( !msg )
	{
		std::cerr << "MarccdCamera::stop -> yat::Message allocation FAILED!"  << std::endl;
		//- TODO : gestion erreur !???
		return;
	}
	
	//- don't wait till the message is processed !!
	this->post(msg);
	
	std::cout << "MarccdCamera::stop() - MARCCD DONE" << std::endl;
}

//---------------------------
//- MarccdCamera::prepare() //- TODO : make a yat msg ?
//---------------------------
void MarccdCamera::prepare()
{
	//- Method to take a backround image. This background will be substacted to each
	//-  taken images !
	//- TODO periodically ?! If so, frequence = ?

	//- SEQUENCE :
	//- wait for marccd to not be reading

	//- send readout 1

	//- wait for marccd to not be reading

	//- send readout 2

	//- wait for marccd to not be reading

	//- dezinger 1
}

//---------------------------
//- MarccdCamera::FreeImage()
//---------------------------
void MarccdCamera::FreeImage()
{
	try
	{
		//m_image_number = 0;
		//m_status = Camera::Ready;
		//if(!_stop_already_done)
		//{
		//	_stop_already_done = true;
		//	// Get the pending buffer back (You are not allowed to deregister
		//	// buffers when they are still queued)
		//	StreamGrabber_->CancelGrab();
	
		//	// Get all buffers back
		//	for (GrabResult r; StreamGrabber_->RetrieveResult(r););
		//	
		//	// Stop acquisition
		//	std::cout <<"Stop acquisition" << std::endl;
		//	Camera_->AcquisitionStop.Execute();
		//	
		//	// Clean up
		//
		//	// You must deregister the buffers before freeing the memory
			std::cout << "Must deregister the buffers before freeing the memory" << std::endl;
		//	for (vector<CGrabBuffer*>::iterator it = BufferList_.begin(); it != BufferList_.end(); it++)
		//	{
		//		StreamGrabber_->DeregisterBuffer((*it)->GetBufferHandle());
		//		delete *it;
		//		*it = NULL;
		//	}
		//	BufferList_.clear();
		//	// Free all resources used for grabbing
			std::cout << "Free all resources used for grabbing" << std::endl;
		//	StreamGrabber_->FinishGrab();
		//}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM FREE_IMAGE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}	
}
		
//---------------------------
//- MarccdCamera::GetImage()
//---------------------------
void MarccdCamera::GetImage()
{
	//- pointer which will receive the image data (from file)
  uint32_t*	pImage = 0;
	Size imgSize;
	std::ifstream filestr(this->_previous_img_path.c_str());
	std::filebuf *pbuf;

std::cout << "MarccdCamera::GetImage() <- ..." << std::endl;
	if ( !filestr )
	{
		std::cout << "MarccdCamera::GetImage() -> FAILED TO OPEN : "
							<< this->_current_img_path 
							<< " image file !!!" << std::endl;
		return;
	}

		DI::DiffractionImage* diff_img;
	try
	{
		m_nb_frames = 11;

		do
		{
			this->get_marccd_state();
			std::cout << "MarccdCamera::GetImage -> CHECKING MARCCD STATE " << std::endl;
		}while(TEST_TASK_STATUS(this->_marccd_state,TASK_WRITE,TASK_STATE_WRITING));

		//if ( this->_previous_img_path == this->_current_img_path )
		//{
		//	std::cout << "MarccdCamera::GetImage -> _previous_img_path == _current_img_path !!!" << std::endl;
		//	return;
		//}

		//- get image size
		//this->getImageSize(imgSize);

		//- allocate pImage
		//pImage = new uint32_t[imgSize.getWidth()*imgSize.getHeight() * DATA_SIZE];	//- image data size is 16bits
		
		//- Lock sur le fichier
		yat::MutexLock scoped_lock(this->_lock);

		//- TODO : check _previous_img_path is NOT NULL
		//- TODO : check _previous_img_path has not be previously read, if so return previous img

		//- read marccd file
		diff_img = new DI::DiffractionImage(this->_previous_img_path);
std::cout << "MarccdCamera::GetImage() -> diffImg has opened : " << this->_previous_img_path << std::endl;

		int img_width		= diff_img->getWidth();
		int img_heigtht = diff_img->getHeight();
		int img_size = img_width * img_heigtht;

		unsigned short *imgData = new unsigned short[img_size];
std::cout << "MarccdCamera::GetImage() -> img size : " << img_size << std::endl;

		imgData = static_cast<int *>(diff_img->getImage());
std::cout << "MarccdCamera::GetImage() -> image read OK." << std::endl;

		void *ptr = buffer_mgr.getBufferPtr(buffer_nb, concat_frame_nb);
std::cout << "MarccdCamera::GetImage() -> ptr OK" << std::endl;
		//- copy image data
		//pbuf->(pImage, size);
		//memcpy((int16_t *)(pImage), (uint16_t *)(ptr), imgSize.getWidth()*imgSize.getHeight()*2);	//- 2 bytes long
		memcpy((int16_t *)ptr, imgData, img_size);	//- 2 bytes long
std::cout << "MarccdCamera::GetImage() -> memcpy OK" << std::endl;

		//FrameBuilder& frame_builder = m_frame_builder;
		//frame_builder.resetFrameNr();

		//int nb_frames = m_nb_frames;
		//int& frame_nb = m_acq_frame_nb;

		//typedef unsigned char *BufferPtr;
		//frame_builder.getNextFrame(BufferPtr(ptr));

		HwFrameInfoType frame_info;
		frame_info.acq_frame_nb = this->_image_number;
		buffer_mgr.newFrameReady(frame_info);
		
		this->_image_number++;

		//this->_previous_img_path = this->_current_img_path;
std::cout << "MarccdCamera::GetImage() -> DONE !" << std::endl;
	}
	catch (...)
    {
        // Error handling
			std::cerr << "MARCAM GET_IMAGE -> An ... exception occurred!"  << std::endl;
        //- XE throw LIMA_HW_EXC(Error, e.GetDescription());
				std::cout << "MarccdCamera::GetImage() -> ERROR : failed to read image from file : \n"
									<< "****** $" << this->_current_img_path << "$" << std::endl;
    }			
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getImageSize(Size& size)
{
	DEB_MEMBER_FUNCT();
	
	std::string resp ("");
	int sizeX, sizeY;

	try
	{
		// get the max image size of the detector
		resp = this->write_read("get_size");

		sscanf(resp.c_str(), "%d,%d", &sizeX, &sizeY);

		size = Size(sizeX,sizeY);
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_IMAGE_SIZE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}			
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getPixelSize(double& size)
{
	DEB_MEMBER_FUNCT();
	size= PixelSize;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getImageType(ImageType& type)
{
	DEB_MEMBER_FUNCT();
	try
	{
		//PixelSizeEnums ps = Camera_->PixelSize.GetValue();
		//switch( ps )
		//{
		//	case PixelSize_Bpp8:
		//		type= Bpp8;
		//	break;
		//
		//	case PixelSize_Bpp12:
		//		type= Bpp12;
		//	break;
		//
		//	case PixelSize_Bpp16: //- this is in fact 12 bpp inside a 16bpp image
		//		type= Bpp16;
		//	break;
		//
		//	default:
				type = Bpp16;
		//	break;
		//}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_IMAGE_TYPE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		

}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getDetectorType(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_type;
	return;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getDetectorModel(std::string& type)
{
	DEB_MEMBER_FUNCT();
	type = this->_detector_model;
	return;		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::setMaxImageSizeCallbackActive(bool cb_active)
{  
	m_mis_cb_act = cb_active;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
BufferCtrlMgr& MarccdCamera::getBufferMgr()
{
	return m_buffer_ctrl_mgr;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::setTrigMode(TrigMode mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(mode);
	
		//- TODO : any trigger (should be internal !) or use an external one !??
		//- if external how & where to manage it !???
	try
	{
		if ( mode == IntTrig )
		{
		  //- INTERNAL
		  //this->Camera_->TriggerMode.SetValue( TriggerMode_Off );
		  ////@@@@ TODO later this mode is disabled !!
		  ////this->Camera_->AcquisitionFrameRateEnable.SetValue( true );
		  //////////////////////////////////////////////////////////////////
std::cout << "MarccdCamera::setTrigMode -> IntTrig" << std::endl;
		}
		else if ( mode == ExtGate )
		{
		  //- EXTERNAL - TRIGGER WIDTH
		  //this->Camera_->TriggerMode.SetValue( TriggerMode_On );
		  //this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
		  //this->Camera_->ExposureMode.SetValue( ExposureMode_TriggerWidth );
std::cout << "MarccdCamera::setTrigMode -> ExtGate" << std::endl;
		}		
		else //ExtTrigSingle
		{
		  //- EXTERNAL - TIMED
		  //this->Camera_->TriggerMode.SetValue( TriggerMode_On );
		  //this->Camera_->AcquisitionFrameRateEnable.SetValue( false );
		  //this->Camera_->ExposureMode.SetValue( ExposureMode_Timed );
std::cout << "MarccdCamera::setTrigMode -> else" << std::endl;
		}
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM SET_TRIG_MODE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getTrigMode(TrigMode& mode)
{
    DEB_MEMBER_FUNCT();
	try
	{
		//if (this->Camera_->TriggerMode.GetValue() == TriggerMode_Off)
		//- TODO : for the moment and device compatibility forced to INTERNAL !!!
		mode = IntTrig;
		//else if (this->Camera_->ExposureMode.GetValue() == ExposureMode_TriggerWidth)
		//	mode = ExtGate;
		//else //ExposureMode_Timed
		//	mode = ExtTrigSingle;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_TRIG_MODE -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::setExpTime(double exp_time_ms)
{
	DEB_MEMBER_FUNCT();
	std::cout << "MarccdCamera::setExpTime(" <<exp_time_ms<< ")" << std::endl;
	DEB_PARAM() << DEB_VAR1(exp_time_ms);

	try
	{
		//- TODO : exposure managed externally !!

		//Camera_->ExposureTimeBaseAbs.SetValue(1E3 * exp_time_ms);
		
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM SET_EXP_TIME -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getExpTime(double& exp_time_ms)
{
	DEB_MEMBER_FUNCT();
	try
	{
		//- TODO : to be deleted !????

		//- exposure time controlled externally !!!
		exp_time_ms = _NaN_;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_EXP_TIME -> An ... exception occurred!"  << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}			
	DEB_RETURN() << DEB_VAR1(exp_time_ms);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::setLatTime(double lat_time)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(lat_time);
	/////@@@@ TODO if necessary
	lat_time = 0.1;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getLatTime(double& lat_time)
{
	DEB_MEMBER_FUNCT();
	/////@@@@ TODO if necessary
	lat_time = _NaN_;
	DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::setNbFrames(int nb_frames)
{
	DEB_MEMBER_FUNCT();
	DEB_PARAM() << DEB_VAR1(nb_frames);
	m_nb_frames = nb_frames;
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getNbFrames(int& nb_frames)
{
	DEB_MEMBER_FUNCT();
	nb_frames = m_nb_frames;
	DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getStatus(MarccdCamera::Status& status)
{
	size_t acquireStatus, readoutStatus, correctStatus, writingStatus, dezingerStatus;
	
	DEB_MEMBER_FUNCT();
		
	size_t mar_status = TASK_STATE(this->_marccd_state);

	acquireStatus = TASK_STATUS(this->_marccd_state, TASK_ACQUIRE); 
	readoutStatus = TASK_STATUS(this->_marccd_state, TASK_READ); 
	correctStatus = TASK_STATUS(this->_marccd_state, TASK_CORRECT); 
	writingStatus = TASK_STATUS(this->_marccd_state, TASK_WRITE); 
	dezingerStatus= TASK_STATUS(this->_marccd_state, TASK_DEZINGER);

	if (this->_marccd_state == 0) m_status = MarccdCamera::Ready;
	else if (this->_marccd_state == 7) m_status = MarccdCamera::Fault;
	else if (this->_marccd_state == 8) m_status = MarccdCamera::Ready;  /* This is really busy interpreting command,
																										but we don't have a status for that yet */
	else if (acquireStatus & (TASK_STATUS_EXECUTING)) m_status = MarccdCamera::Exposure;
	else if (readoutStatus & (TASK_STATUS_EXECUTING)) m_status = MarccdCamera::Readout;
	else if (correctStatus & (TASK_STATUS_EXECUTING)) m_status = MarccdCamera::Readout;/*ADStatusCorrect*/
	else if (writingStatus & (TASK_STATUS_EXECUTING)) m_status = MarccdCamera::Readout;/*ADStatusSaving*/
	if ((acquireStatus | readoutStatus | correctStatus | writingStatus | dezingerStatus) & 
		TASK_STATUS_ERROR) m_status = MarccdCamera::Fault;

	status = m_status;
std::cout << "*********\tMarccdCamera::getStatus -> m_status = " << m_status << "\n" << std::endl;
std::cout << "*********\tMarccdCamera::getStatus -> m_status = " << status << "\n" << std::endl;

	DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
//
//-----------------------------------------------------
void MarccdCamera::getFrameRate(double& frame_rate)
{
	DEB_MEMBER_FUNCT();
	try
	{
		frame_rate = 123456.7;
	}
	catch (...)
	{
		// Error handling
		std::cerr << "MARCAM GET_FRAME_RATE -> An ... exception occurred!" << std::endl;
		//- XE throw LIMA_HW_EXC(Error, e.GetDescription());
	}		
	DEB_RETURN() << DEB_VAR1(frame_rate);
}

//-----------------------------------------------------
// - INTERNAL METHODS
//-----------------------------------------------------
void MarccdCamera::disconnect()
{
	try
	{
		this->_sock->disconnect();
		std::cout << "INFO::MarccdCamera::disconnect(): DONE !" << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::MarccdCamera::disconnect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::MarccdCamera::disconnect -> [...] Exception." << std::endl;
	}
	//- TODO : gestion state de la com
	//this->com_state = Tango::CLOSE;
}

void MarccdCamera::connect()
{
	try
	{
		//this->_sock->set_option(yat::Socket::SOCK_OPT_KEEP_ALIVE, 1);
		this->_sock->set_option(yat::Socket::SOCK_OPT_NO_DELAY, 1);
		//this->_sock->set_option(yat::Socket::SOCK_OPT_OTIMEOUT, 0);
		//this->_sock->set_option(yat::Socket::SOCK_OPT_ITIMEOUT, 0);

		yat::Address addr(this->_camera_ip, this->_port_num);
		std::cout << "INFO::MarccdCamera::connect(): made Address... !" << std::endl;

		this->_sock->connect(addr);
		std::cout << "INFO::MarccdCamera::connect(): connect DONE." << std::endl;
	}
	//- TODO : on error -> sock = NULL
	catch(yat::SocketException & ySe)
	{
		this->_sock = 0;
		std::cout << "ERROR::MarccdCamera::connect -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		this->_sock = 0;
		std::cout << "ERROR::MarccdCamera::connect -> [...] Exception." << std::endl;
	}
}

//-----------------------------------------------------
// - read : returns Marccd command response
//-----------------------------------------------------
std::string MarccdCamera::read()
{
	std::string response("");
	try
	{
	//- TODO : CHECK if sock = NULL
		this->_sock->receive(response);
		//std::cout << "INFO::MarccdCamera::read(): response = " << response << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::MarccdCamera::read -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::MarccdCamera::read -> [...] Exception." << std::endl;
	}

	return response;
}

//-----------------------------------------------------
// - write : sends Marccd command
//-----------------------------------------------------
void MarccdCamera::write(std::string cmd_to_send)
{
	try
	{
		size_t data_size = cmd_to_send.size() + 1; //- +1 for '\0' character
	//- TODO : CHECK if sock = NULL
		this->_sock->send(cmd_to_send.c_str(), data_size);

//std::cout << "\n\nINFO::MarccdCamera::write(): cmd_to_be_send = " << cmd_to_send << " WRITTEN." << std::endl;
	}
	catch(yat::SocketException & ySe)
	{
		std::cout << "ERROR::MarccdCamera::write -> yat::SocketException : " << ySe.errors[0].desc << std::endl;
	}
	catch(...)
	{
		std::cout << "ERROR::MarccdCamera::write -> [...] Exception." << std::endl;
	}
}

//-----------------------------------------------------
// - write_read : Sends a command and reads response
//-----------------------------------------------------
std::string MarccdCamera::write_read(std::string cmd_to_send)
{
	this->write(cmd_to_send);
	
	return this->read();
}

//-----------------------------------------------------
// - perform_start_sequence : manage a Marccd acquisition
//-----------------------------------------------------
void MarccdCamera::perform_start_sequence()
{
std::cout << "MarccdCamera::perform_start_sequence <- " << std::endl;
	//---------------------------------------
	//-		PREPARE SEQUENCE
	//---------------------------------------
	//- Wait for Marccd to not be acquiring !
	do
	{
		this->get_marccd_state();
	}while(TEST_TASK_STATUS(this->_marccd_state,TASK_ACQUIRE,TASK_STATUS_EXECUTING));
std::cout << "\t **** Wait for Marccd to not be acquiring DONE." << std::endl;

	//---------------------------------------
	//-		TELL MARCCD START ACQUIRING
	//---------------------------------------
	//- Send start cmd
	std::string cmd_to_send("start");
	this->write(cmd_to_send);
std::cout << "\t **** START SENT !!!" << std::endl;

	//- Wait for Marccd to start acquiring : WHY not working => infinite loop !!!
	//do
	//{
	//	//yat::MutexLock scoped_lock(this->_lock);        
	//	this->get_marccd_state();
	//}while( !TEST_TASK_STATUS(this->_marccd_state, TASK_ACQUIRE, TASK_STATUS_EXECUTING) );

	//---------------------------------------
	//-		SHUTTER CONTROLLED IN DS -> TODO : to be implemented !!!
	//---------------------------------------
std::cout << "MarccdCamera::perform_start_sequence -> " << std::endl;
}

//-----------------------------------------------------
// - perform_stop_sequence : manage a Marccd END acquisition
//-----------------------------------------------------
void MarccdCamera::perform_stop_sequence()
{
std::cout << "MarccdCamera::perform_stop_sequence <- " << std::endl;

	//---------------------------------------
	//-		END ACQ : by starting readout -> TODO : to be implemented in STOP cmd !!!
	//---------------------------------------
	//- Send readout cmd with a specific file name
	std::string cmd_to_send("readout,0,");
	cmd_to_send += this->_current_img_path;
	std::cout << "MarccdCamera::written file : &" << cmd_to_send << "$" << std::endl;

	this->write(cmd_to_send);
std::cout << "MarccdCamera::perform_stop_sequence -> " << std::endl;

	//- prepare msg
	yat::Message * msg = new yat::Message(GET_IMAGE_MSG, MAX_USER_PRIORITY);
	if ( !msg )
	{
		std::cerr << "MarccdCamera::perform_stop_sequence : GetImage FAILED -> yat::Message allocation FAILED!"  << std::endl;
		//- TODO : gestion erreur !???
		return;
	}
	//- don't wait till the message is processed !!
	this->post(msg);

}

void MarccdCamera::get_marccd_state()
{
	//- get detectot state string value
	std::string stateStr = this->write_read("get_state");
	
	//- convert state string to numeric val
	this->_marccd_state  = yat::XString<size_t>::to_num(stateStr);
std::cout << "\n*********\tMarccdCamera::getStatus -> mar_state = " << this->_marccd_state << std::endl;
}

//-----------------------------------------------------
// Constructor allocates the image buffer
//-----------------------------------------------------
//CGrabBuffer::CGrabBuffer(size_t ImageSize)
//{
//	m_pBuffer = new uint8_t[ ImageSize ];
//	if (NULL == m_pBuffer)
//	{
//		//GenICam::GenericException e("Not enough memory to allocate image buffer", __FILE__, __LINE__);
//		std::cout << "CGrabBuffer::CGrabBuffer -> NULL == m_pBuffer !!!" << std::endl;;
//	}
//}

//-----------------------------------------------------
// Freeing the memory
//-----------------------------------------------------
//CGrabBuffer::~CGrabBuffer()
//{
//	if (NULL != m_pBuffer)
//		delete[] m_pBuffer;
//}
//-----------------------------------------------------
