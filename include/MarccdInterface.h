#ifndef MARCCD_INTERFACE_H
#define MARCCD_INTERFACE_H

#include "HwInterface.h"
#include "MarccdCamera.h"
#include "MarccdReader.h"



namespace lima
{
namespace Marccd
{
class Interface;

//*******************************************************************
// \class DetInfoCtrlObj
// \brief Control object providing Marccd detector info interface
//*******************************************************************
class DetInfoCtrlObj : public HwDetInfoCtrlObj
{
	DEB_CLASS_NAMESPC(DebModCamera, "DetInfoCtrlObj", "Marccd");

 public:
	DetInfoCtrlObj(Camera& cam);
	virtual ~DetInfoCtrlObj();

	virtual void getMaxImageSize(Size& max_image_size);
	virtual void getDetectorImageSize(Size& det_image_size);

	virtual void getDefImageType(ImageType& def_image_type);
	virtual void getCurrImageType(ImageType& curr_image_type);
	virtual void setCurrImageType(ImageType  curr_image_type);

	virtual void getPixelSize(double& pixel_size);
	virtual void getDetectorType(std::string& det_type);
	virtual void getDetectorModel(std::string& det_model);

	virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
	virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);

 private:
	Camera& m_cam;
};


//*******************************************************************
// \class BufferCtrlObj
// \brief Control object providing Marccd buffering interface
//*******************************************************************
class BufferCtrlObj : public HwBufferCtrlObj
{
	DEB_CLASS_NAMESPC(DebModCamera, "BufferCtrlObj", "Marccd");

 public:
	BufferCtrlObj(Camera& simu);
	virtual ~BufferCtrlObj();

	virtual void setFrameDim(const FrameDim& frame_dim);
	virtual void getFrameDim(      FrameDim& frame_dim);

	virtual void setNbBuffers(int  nb_buffers);
	virtual void getNbBuffers(int& nb_buffers);

	virtual void setNbConcatFrames(int  nb_concat_frames);
	virtual void getNbConcatFrames(int& nb_concat_frames);

	virtual void getMaxNbBuffers(int& max_nb_buffers);

	virtual void *getBufferPtr(int buffer_nb, int concat_frame_nb = 0);
	virtual void *getFramePtr(int acq_frame_nb);

	virtual void getStartTimestamp(Timestamp& start_ts);
	virtual void getFrameInfo(int acq_frame_nb, HwFrameInfoType& info);

    // -- Buffer control object
    BufferCtrlMgr&      getBufferMgr(){return m_buffer_ctrl_mgr;};
    StdBufferCbMgr&     getBufferCbMgr(){return m_buffer_cb_mgr;};
    
	virtual void registerFrameCallback(HwFrameCallback& frame_cb);
	virtual void unregisterFrameCallback(HwFrameCallback& frame_cb);
  
  // Reader stuff
    void start();
    void stop();
    void reset();
    int  getLastAcquiredFrame();    

 private:
    SoftBufferAllocMgr      m_buffer_alloc_mgr;
    StdBufferCbMgr          m_buffer_cb_mgr;
    BufferCtrlMgr           m_buffer_ctrl_mgr;
    Camera&                 m_cam;
    Reader*                 m_reader;    
};

//*******************************************************************
// * \class SyncCtrlObj
// * \brief Control object providing Marccd synchronization interface
// *******************************************************************/
class SyncCtrlObj : public HwSyncCtrlObj
{
    DEB_CLASS_NAMESPC(DebModCamera, "SyncCtrlObj", "Marccd");

  public:
		SyncCtrlObj(Camera& cam);
    virtual ~SyncCtrlObj();
	
		virtual bool checkTrigMode(TrigMode trig_mode);
    virtual void setTrigMode(TrigMode  trig_mode);
    virtual void getTrigMode(TrigMode& trig_mode);

    virtual void setExpTime(double  exp_time);
    virtual void getExpTime(double& exp_time);

    virtual void setLatTime(double  lat_time);//- Not supported by Marccd
    virtual void getLatTime(double& lat_time);//- Not supported by Marccd

    virtual void setNbHwFrames(int  nb_frames);
    virtual void getNbHwFrames(int& nb_frames);

    virtual void getValidRanges(ValidRangesType& valid_ranges);

  private:
    Camera& m_cam;
};

//*******************************************************************
// * \class Interface
// * \brief Marccd hardware interface
//*******************************************************************/
class Interface : public HwInterface
{
	DEB_CLASS_NAMESPC(DebModCamera, "MarccdInterface", "Marccd");

 public:
	Interface(Camera& cam);
	virtual ~Interface();

	//- From HwInterface
	virtual void 	getCapList(CapList&) const;
	virtual void	reset(ResetLevel reset_level);
	virtual void 	prepareAcq();
	virtual void 	startAcq();
	virtual void 	stopAcq();
	virtual void 	getStatus(StatusType& status);
	virtual int 	getNbHwAcquiredFrames();

	void 			getFrameRate(double& frame_rate);
 private:
	Camera&		m_cam;
	CapList 		m_cap_list;
	DetInfoCtrlObj	m_det_info;
	BufferCtrlObj	m_buffer;
	SyncCtrlObj	m_sync;

};

} // namespace Marccd
} // namespace lima

#endif // MARCCD_INTERFACE_H