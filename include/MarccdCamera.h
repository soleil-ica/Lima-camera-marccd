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
		// \class Camera
		// \brief object controlling the marccd detector through a socket
		//--------------------------------------------------------------------
		class Camera : public yat::Task
		{
			DEB_CLASS_NAMESPC(DebModCamera, "Camera", "MarCCD");

		public:

			enum Status {
				Ready, 
				Exposure,
				Readout,
				Latency,
				Fault,
				Unknown
			};

			Camera(const std::string& camera_ip, size_t port_number, const std::string& img_path, const std::string& img_name,const std::string& img_dir_watcher);
			~Camera();

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

			//- Sync 
			void setTrigMode(TrigMode  mode);
			void getTrigMode(TrigMode& mode);

			void setExpTime(double  exp_time);
			void getExpTime(double& exp_time);

			void setLatTime(double  lat_time);
			void getLatTime(double& lat_time);

			void setNbFrames(int  nb_frames);
			void getNbFrames(int& nb_frames);

			void getStatus(Camera::Status& status);
      
			void getFrameRate(double& frame_rate);
      
      const std::string& getDirectoryWatcherPath(void);
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

			
      static const double PixelSize= 39.795; //- NOT PRIVATE !??
      
			//- mutex to protect file against read image from device and
			//-		marccd acquisition
			yat::Mutex 	        _lock;
			
			yat::ClientSocket*  _sock;
			
			std::string         _image_path;
			std::string         _image_name;
			std::string         _image_dir_watcher_path;
			


			//- img stuff
			int 	m_nb_frames;	
			Size	m_image_size;
      
			unsigned short	    m_trigger_type;
			unsigned				    m_exp_time;

	
			size_t _marccd_state;
			Camera::Status		  m_status;
			int							    _image_number;
			bool						    _stop_already_done;

			//- Marccd stuff 
			std::string		      _camera_ip;
			size_t				      _port_num;
			std::string 	      _detector_model;
			std::string 	      _detector_type;
		};

	} //- namespace Marccd
} // namespace lima


#endif // _MARCCD_INTERFACE_H
