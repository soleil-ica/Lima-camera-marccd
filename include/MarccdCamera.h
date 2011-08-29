#ifndef _MARCCD_INTERFACE_H
#define _MARCCD_INTERFACE_H

///////////////////////////////////////////////////////////
// YAT::TASK 
///////////////////////////////////////////////////////////
#include <yat/threading/Task.h>
#include <yat/network/ClientSocket.h>
//#include <yat/network/Address.h>

#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

const size_t  START_MSG			=	(yat::FIRST_USER_MSG + 100);
const size_t  STOP_MSG			=	(yat::FIRST_USER_MSG + 101);
const size_t  GET_IMAGE_MSG	=	(yat::FIRST_USER_MSG + 102);

///////////////////////////////////////////////////////////

#include <stdlib.h>
#include <limits>

#include "HwMaxImageSizeCallback.h"
#include "HwBufferMgr.h"

namespace lima
{

	namespace Marccd
	{

		//--------------------------------------------------------------------
		// \class MarccdCamera
		// \brief object controlling the marccd detector through a socket
		//--------------------------------------------------------------------
		class MarccdCamera : public yat::Task
		{
			DEB_CLASS_NAMESPC(DebModCamera, "MarccdCamera", "MarCCD");

		public:

			enum Status {
				Ready, 
				Exposure,
				Readout,
				Latency,
				Fault,
				Unknown
			};

			MarccdCamera(const std::string& camera_ip, size_t port_number, const std::string& fullImagePathName);
			~MarccdCamera();

			void start();
			void stop();
			//- to be renamed "take_background_frame"
			void prepare(); //- Take a background frame (to substracted)

			// -- detector info
			void getImageSize(Size& size);
			void setPixelDepth(ImageType pixel_depth);
			void getPixelDepth(ImageType& pixel_depth);
			void getPixelSize(double& size);
			void getImageType(ImageType& type);

			void getDetectorType(std::string& type);
			void getDetectorModel(std::string& model);

			//- Buffer
			BufferCtrlMgr& getBufferMgr();

			//- Sync 
			void setTrigMode(TrigMode  mode);
			void getTrigMode(TrigMode& mode);

			void setExpTime(double  exp_time);
			void getExpTime(double& exp_time);

			void setLatTime(double  lat_time);
			void getLatTime(double& lat_time);

			void setNbFrames(int  nb_frames);
			void getNbFrames(int& nb_frames);

			void getStatus(MarccdCamera::Status& status);

			static const double PixelSize= 55.0; //- NOT PRIVATE !??
			void getFrameRate(double& frame_rate);

		protected:
			virtual void setMaxImageSizeCallbackActive(bool cb_active);	

			//- yat::Task implementation
			virtual void handle_message( yat::Message& msg )      throw (yat::Exception);
		private:
			void GetImage();
			void FreeImage();

			/**
			* Disconnect from the current socket
			*/
			void disconnect();
			/**
			* Connect to the current socket
			*/
			void connect();
			/**
			* Get detector response
			*/
			std::string read();
			/**
			* Send command to detector
			*/
			void write(std::string);
			/**
			* Send command and receive the response
			*/
			std::string write_read(std::string);
			/**
			* update marccd state
			*/
			void get_marccd_state();
			/**
			* Method to start an Marccd acquisition
			*/
			void perform_start_sequence();
			/**
			* Method to stop an Marccd acquisition
			*/
			void perform_stop_sequence();

			//- mutex to protect file against read image from device and
			//-		marccd acquisition
			yat::Mutex 	_lock;
			
			yat::ClientSocket* _sock;
			
			std::string _current_img_path;
			std::string _previous_img_path;
			

			//- lima stuff
			SoftBufferAllocMgr 	m_buffer_alloc_mgr;
			StdBufferCbMgr 			m_buffer_cb_mgr;
			BufferCtrlMgr 			m_buffer_ctrl_mgr;
			bool 								m_mis_cb_act;

			//- img stuff
			int 	m_nb_frames;	
			Size	m_image_size;
			//		IMG_TYPE		m_pixel_depth;
			unsigned short	m_trigger_type;
			unsigned				m_exp_time;

			uint16_t**	pSeqImage;
			uint16_t*	  pOneImage;

			//---------------------------------
			//- xpad stuff 
			short		m_acquisition_type;
			unsigned	m_modules_mask;
			int			m_module_number;
			unsigned	m_chip_number;
			int			m_full_image_size_in_bytes;
			unsigned	m_time_unit;
			std::vector<long>		m_all_config_g;

			//- FParameters
			unsigned		m_fparameter_deadtime;
			unsigned		m_fparameter_init;
			unsigned		m_fparameter_shutter;
			unsigned		m_fparameter_ovf;
			unsigned		m_fparameter_mode;
			unsigned		m_fparameter_n;
			unsigned		m_fparameter_p;
			unsigned		m_fparameter_GP1;
			unsigned		m_fparameter_GP2;
			unsigned		m_fparameter_GP3;
			unsigned		m_fparameter_GP4;

			size_t _marccd_state;
			MarccdCamera::Status		m_status;
			int							_image_number;
			bool						_stop_already_done;

			//- Marccd stuff 
			std::string		_camera_ip;
			size_t				_port_num;
			std::string 	_detector_model;
			std::string 	_detector_type;
		};

	} //- namespace Marccd
} // namespace lima


#endif // _MARCCD_INTERFACE_H
