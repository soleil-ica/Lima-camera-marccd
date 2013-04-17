#ifndef _MARCCD_INTERFACE_H
#define _MARCCD_INTERFACE_H

///////////////////////////////////////////////////////////
// YAT::TASK
///////////////////////////////////////////////////////////
#include <yat/threading/Task.h>
#include <yat/network/ClientSocket.h>
#include <yat/time/Time.h>
#include <Timer.h>

#define kLO_WATER_MARK      128
#define kHI_WATER_MARK      512

#define kPOST_MSG_TMO       2

///////////////////////////////////////////////////////////

#include <stdlib.h>
#include <limits>

#include "HwBufferMgr.h"

namespace lima
{

    namespace Marccd
    {

        //--------------------------------------------------------------------
        // \class ExposureCb
        // \brief object controlling the exposure duration
        //--------------------------------------------------------------------        

        class ExposureCb : public Timer::Callback
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "MarCCD");
        public:

            void start()
            {
                DEB_MEMBER_FUNCT();                
                DEB_TRACE() << "ExposureCb::start";
                m_is_falling_occured = false;
                m_start = yat::CurrentTime().raw_value();
            }

            void risingEdge()
            {               
               m_is_falling_occured = false;                
            }

            void fallingEdge()
            {
                DEB_MEMBER_FUNCT();
                DEB_TRACE() << "ExposureCb::fallingEdge";
                m_is_falling_occured = true;
                m_end = yat::CurrentTime().raw_value();
                unsigned long millis = (m_end - m_start) / 1000;
                DEB_TRACE() << "Elapsed time  = " << millis << " (ms)\n";
            }

            void end()
            {
            }
            
            bool isFallingOccured(){return m_is_falling_occured;}
        private:
            yat::int64 m_start;
            yat::int64 m_end;
            bool m_is_falling_occured;        

        };

        //--------------------------------------------------------------------
        // \class LatencyCb
        // \brief object controlling the latency duration
        //--------------------------------------------------------------------

        class LatencyCb : public Timer::Callback
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "MarCCD");
        public:

            void start()
            {
                DEB_MEMBER_FUNCT();                
                DEB_TRACE() << "LatencyCb::start";          
                m_is_falling_occured = false;                        
                m_start = yat::CurrentTime().raw_value();
            }

            void risingEdge()
            {
               m_is_falling_occured = false;                
            }

            void fallingEdge()
            {
                DEB_MEMBER_FUNCT();
                DEB_TRACE() << "LatencyCb::fallingEdge";
                m_is_falling_occured = true;               
                m_end = yat::CurrentTime().raw_value();
                unsigned long millis = (m_end - m_start) / 1000;
                DEB_TRACE() << "Elapsed time  = " << millis << " (ms)\n";
            }

            void end()
            {         
            }
            
            bool isFallingOccured(){return m_is_falling_occured;}           
        private:
            yat::int64 m_start;
            yat::int64 m_end;
            bool m_is_falling_occured;            
        };


        //--------------------------------------------------------------------
        // \class Camera
        // \brief object controlling the marccd detector through a socket
        //--------------------------------------------------------------------

        class Camera : public yat::Task
        {
            DEB_CLASS_NAMESPC(DebModCamera, "Camera", "MarCCD");

        public:

            enum Status
            {
                Ready,
                Exposure,
                Readout,
                Latency,
                Config,
                Fault,
                Unknown
            };

            Camera(const std::string& camera_ip,
                   size_t port_number,
                   const std::string& img_path);

            ~Camera();

            void start();
            void stop();
            void take_background_frame();

            // -- detector info
            void getImageSize(Size& size);
            void getPixelSize(double& x_size, double &y_size);
            void getImageType(ImageType& type);

            void getDetectorType(std::string& type);
            void getDetectorModel(std::string& model);

            //- Sync
            void setTrigMode(TrigMode mode);
            void getTrigMode(TrigMode& mode);

            void setExpTime(double exp_time);
            void getExpTime(double& exp_time);

            void setLatTime(double lat_time);
            void getLatTime(double& lat_time);

            void setNbFrames(int nb_frames);
            void getNbFrames(int& nb_frames);

            void getStatus(Camera::Status& status);
            unsigned int getState();

            void checkRoi(const Roi& set_roi, Roi& hw_roi);
            void setRoi(const Roi& set_roi);
            void getRoi(Roi& hw_roi);

            void setBinning(const Bin&);
            void getBinning(Bin&);
            void checkBin(Bin&);

            //specific stuff 
            void getDetectorImageSize(Size& size);
            void setImagePath(const std::string& path);
            const std::string& getImagePath(void);
            void setImageFileName(const std::string& imgName);
            const std::string& getImageFileName();
            void setImageIndex(int newImgIdx);
            int getImageIndex();
            int getFirstImage();

            bool isStopSequenceFinished();
            void saveBGFrame(bool);

            void setBeamX(float);
            float getBeamX();
            void setBeamY(float);
            float getBeamY();
            void setDistance(float);
            float getDistance();

            void setWavelength(float);
            float getWavelength();

        protected:

            //- yat::Task implementation
            virtual void handle_message(yat::Message& msg) throw (yat::Exception);
        private:

            /// Get detector response
            std::string read();

            /// Send command to detector
            void write(std::string);

            /// Send command and receive the response
            std::string send_cmd_and_receive_answer(std::string);

            /// get detector state
            void get_marccd_state();

            /// Start acquisition sequence
            void perform_acquisition_sequence();

            /// Abort acquisition
            void performm_abort_sequence();

            /// start a background frame acqusistion
            void perform_background_frame();

            ///convert the detector status string to an int.
            int convert_string_to_int(char* hexa_text);


            //- socket object used to communicate with detector
            yat::ClientSocket m_sock;

            //- mutex to protect file against read image from device and detector
            yat::Mutex m_lock;

            //- image/file stuff
            int m_nb_frames;
            Size m_detector_size;
            std::string m_image_path;
            std::string m_image_name;

            //- acquisition control stuff
            double m_exp_time;
            double m_lat_time;
            unsigned m_binning_x;
            unsigned m_binning_y;

            size_t m_marccd_state;
            Camera::Status m_status;
            int m_image_number;
            int m_first_image;

            //- Marccd stuff
            std::string m_camera_ip;
            size_t m_port_num;
            std::string m_detector_model;
            std::string m_detector_type;

            // X-ray source header info
            float m_source_beamX;
            float m_source_beamY;
            float m_source_distance;
            float m_source_wavelength;

            bool m_is_stop_sequence_finished;
            bool m_is_abort_in_progress;
            bool m_is_bg_acquisition_finished;
            bool m_is_bg_saving_requested;

            std::string m_error;

            // Timers controlling exposure/latency durations
            ExposureCb m_exposure_cb;
            LatencyCb m_latency_cb;
            Timer m_exposure_timer;
            Timer m_latency_timer;
        };

    } //- namespace Marccd
} // namespace lima

#endif // _MARCCD_INTERFACE_H
