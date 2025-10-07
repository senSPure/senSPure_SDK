/*------------------------------------------------------------------*/
/// @file		EvtThread.h
/// @brief		Event Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <thread>
#include <memory>
#include <utility>
#include <functional>

#include "MsgQue.h"
#include "CameraType.h"
#include "FrameData.h"

namespace krm
{
/*------------------------------------------------------------------*/
/// @brief	Maximum number of queuing frames
/*------------------------------------------------------------------*/
static const uint8_t MAX_QUEUING_FRAMES = 3U;

/*------------------------------------------------------------------*/
/// @brief	Kind of thread processing
/*------------------------------------------------------------------*/
enum ProcKind : uint16_t {
	PROC_CAMERA = 0,	/*!< Camera control */
	PROC_RECORD,		/*!< Record */
	PROC_LENSCONV,		/*!< Lens Conversion */
	PROC_POSTFILT,		/*!< Post Filter */
	PROC_USER			/*!< Other user thread */
};

/*------------------------------------------------------------------*/
/// @brief	Callback function from pipeline
/*------------------------------------------------------------------*/
using PlCbFunc = std::function<void(uint16_t, Result)>;

/*------------------------------------------------------------------*/
/// @brief	PipeLine parameters
/*------------------------------------------------------------------*/
struct PipeLinePrm {
	uint16_t					proc_id;		/* Processing ID */
	std::shared_ptr<MsgQue>		rcv_que;		/* Message queue for receiving */
	std::shared_ptr<MsgQue>		snd_que;		/* Message queue for sending end of pipeline */
	std::shared_ptr<MsgQue>		next_que;		/* Message queue for sending next pipeline */
	uint8_t						next_que_max;	/* Maximum of queue in next pipeline */
	PlCbFunc					status_cb;		/* status callback function */
	bool						eopl;			/* End of pipeline(true: end, false:not end) */
};

/*------------------------------------------------------------------*/
/// @brief	Camera properties
/*------------------------------------------------------------------*/
struct CameraProperty {
	ModeInfo		mode_info;		/* Motion mode information */
	LensInfo		lens_info;		/* Lens conversion parameters */
	CamFov			fov;			/* Field of View */
	PostFiltInfo	post_filt_info;	/* Post filter information */
};

class EvtThread
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	EvtThread(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	virtual ~EvtThread(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set Pipeline parameter
	/// @param	[in]	param		PipeLine parameters
	/*------------------------------------------------------------------*/
	void setPipelinePrm(const PipeLinePrm& param);

	/*------------------------------------------------------------------*/
	/// @brief	Get kind of thread processing
	/// @return	Kind of thread processing
	/*------------------------------------------------------------------*/
	virtual ProcKind getKind(void) { return PROC_USER; }

	/*------------------------------------------------------------------*/
	/// @brief	Reset Property
	/// @param	[in]	property		Camera property
	/// @retval	SUCCESS		Success
	/// @retval	ERR_BAD_ARG	Invalid Argument
	/*------------------------------------------------------------------*/
	Result resetProp(const CameraProperty& property);

	/*------------------------------------------------------------------*/
	/// @brief	Wakeup thread
	/// @retval	SUCCESS			Success
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	virtual Result wakeup(void);
	/*------------------------------------------------------------------*/
	/// @brief	Shutdown thread
	/// @retval	SUCCESS			Success
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	virtual Result shutdown(void);

	/*------------------------------------------------------------------*/
	/// @brief	Get maximum number of queuing frames
	/// @return	Maximum number of queuing frames
	/*------------------------------------------------------------------*/
	virtual uint8_t getMaxQueuingFrames(void) { return MAX_QUEUING_FRAMES; }

	/* delete copy */
	EvtThread(const EvtThread&) = delete;
	EvtThread& operator=(const EvtThread&) = delete;

protected:
	uint16_t					proc_id_;		/* Processing ID */
	std::shared_ptr<MsgQue>		rcv_que_;		/* Message queue for receiving */
	std::shared_ptr<MsgQue>		snd_que_;		/* Message queue for sending */
	std::shared_ptr<MsgQue>		next_que_;		/* Message queue for sending next pipeline */
	PlCbFunc					status_cb_;		/* status callback */
	bool						eopl_;			/* End of pipeline */
	uint8_t						que_th_;		/* Threshold queueing in self */
	uint8_t						next_que_th_;	/* Threshold queueing in next pipeline */
	bool						dropped_;		/* Dropped frame when stagnant */

	/*------------------------------------------------------------------*/
	/// @brief	Thread is exist
	/*------------------------------------------------------------------*/
	inline bool isThreadExist(void) { return (thread_ != nullptr); }

	/*------------------------------------------------------------------*/
	/// @brief	Enable frame drop when stagnant
	/// @retval	true	Enable frame drop
	/// @retval	false	Disable frame drop
	/*------------------------------------------------------------------*/
	virtual bool enableFrameDrop(void) { return true; }

	/*------------------------------------------------------------------*/
	/// @brief	Thread main before function
	/*------------------------------------------------------------------*/
	virtual Result exeBefore(void) { return SUCCESS; }
	/*------------------------------------------------------------------*/
	/// @brief	Thread main function
	/*------------------------------------------------------------------*/
	virtual void execute(void);
	/*------------------------------------------------------------------*/
	/// @brief	Thread main after function
	/*------------------------------------------------------------------*/
	virtual Result exeAfter(void) { return SUCCESS; }

	/*------------------------------------------------------------------*/
	/// @brief	Send Queue
	/*------------------------------------------------------------------*/
	void sendQue(std::shared_ptr<MsgQue>& msg_que, MsgData& data);
	/*------------------------------------------------------------------*/
	/// @brief	Notify message to next pipeline processing
	///	@param	[in]	msg		Message data
	///	@param	[in]	img_ev	Event is RCV_IMG?
	/*------------------------------------------------------------------*/
	void notifyNextPl(MsgData& msg, bool img_ev = false);

	/*------------------------------------------------------------------*/
	/// @brief	Received CHG_PROP event
	///	@retval	SUCCESS		Success
	/// @retval	ERR_BAD_ARG	Invalid Argument
	/*------------------------------------------------------------------*/
	virtual Result recvChgProp(const CameraProperty& /*property*/) { return SUCCESS; }
	/*------------------------------------------------------------------*/
	/// @brief	Received CHG_FMT event
	///	@retval	SUCCESS		Success
	/// @retval	ERR_BAD_ARG	Invalid Argument
	/*------------------------------------------------------------------*/
	virtual Result recvChgFmt(const ImageFormats& /*formats*/) { return SUCCESS; }
	/*------------------------------------------------------------------*/
	/// @brief	Received RCV_IMG event
	///	@retval	SUCCESS		Success
	/// @retval	ERR_BAD_ARG	Invalid Argument
	/*------------------------------------------------------------------*/
	virtual Result recvImage(FrameData& /*frame*/) { return SUCCESS; }
	/*------------------------------------------------------------------*/
	/// @brief	Received RESET event
	/*------------------------------------------------------------------*/
	virtual void recvReset(void) {}
	/*------------------------------------------------------------------*/
	/// @brief	Received User event
	///	@retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Invalid Argument
	/// @retval	ERR_NOT_SUPPORT	Not supported
	/*------------------------------------------------------------------*/
	virtual Result recvUserEvent(uint8_t /*event*/, void* /*param*/) { return SUCCESS; }

	/*------------------------------------------------------------------*/
	/// @brief	Notify status callback
	///	@param	[in]	status		status
	/*------------------------------------------------------------------*/
	void notifyStatus(Result status);
	/*------------------------------------------------------------------*/
	/// @brief	Start to measure time
	/// @param	[in]	frame		Frame data
	/// @param	[in]	func_no		Function number
	/*------------------------------------------------------------------*/
	inline void startMeasureTime(const Frame& frame, uint8_t func_no = 0);
	/*------------------------------------------------------------------*/
	/// @brief	Stop to measure time
	/// @param	[in]	func_no		Function number
	/*------------------------------------------------------------------*/
	inline void stopMeasureTime(uint8_t func_no = 0);

private:
	static const uint32_t	RCV_TIMEOUT = 1000U;	/* timeout when receive queue in wakeup/shutdown [ms] */
	struct ThreadArg {
		std::pair<MsgQue, MsgQue>*	wake_msg;		/* Massage for wakeup */
		EvtThread*					self_cxt;		/* self instance */
	};
	enum PrivCmd : uint8_t {
		MSG_WAKEUP,			/* wakeup */
		MSG_SHUTDOWN,		/* shutdown */
	};
	std::pair<MsgQue, MsgQue>		wake_msg_;		/* Massage for wakeup */
	std::shared_ptr<std::thread>	thread_;		/* thread */
	bool							stopped_;		/* Stopped capture */
	void*							time_record_;	/* Time Record */

	/*------------------------------------------------------------------*/
	/// @brief	Thread function
	///	@param	[in]	ins		self instance (ThreadArg)
	///	@retval NULL
	/*------------------------------------------------------------------*/
	static void* thrdMain(void *ins);
	/*------------------------------------------------------------------*/
	/// @brief	Check delay proccess(self-thread)
	///	@param	[in]	frame			frame data
	///	@param	[in]	under_th		under threshold(self)
	///	@param	[in]	under_next_th	under threshold(next)
	///	@retval	true	delay
	///	@retval	false	normal
	/*------------------------------------------------------------------*/
	bool checkDelaySelf(void* frame, uint8_t under_th, uint8_t under_next_th);
	/*------------------------------------------------------------------*/
	/// @brief	Check delay proccess(next-thread)
	///	@param	[in]	frame	frame data
	///	@retval	true	delay
	///	@retval	false	normal
	/*------------------------------------------------------------------*/
	bool checkDelayNext(void* frame);
	/*------------------------------------------------------------------*/
	/// @brief	Add dropped frame in FrameInfo
	///	@param	[in]	frame_data	frame data
	/*------------------------------------------------------------------*/
	void addDroppedFrame(FrameData* frame_data);
	/*------------------------------------------------------------------*/
	/// @brief	Release frame data
	///	@param	[in]	frame	frame data
	/*------------------------------------------------------------------*/
	void releaseFrame(FrameData* frame);
};

} // namespace krm
