/*------------------------------------------------------------------*/
/// @file		PlFw.h
/// @brief		PlFw Framework class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <memory>
#include <functional>

#include "TpTofSdkDefine.h"
#include "CameraType.h"
#include "EvtThread.h"
#include "FrameData.h"

namespace krm
{

class PlFw
{
public:
	static const uint16_t PROC_PIPELINE = 0;
	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::Camera
	/*------------------------------------------------------------------*/
	PlFw(CameraType type, const void* param, Result& res);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~PlFw(void);

	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::getDeviceList
	/*------------------------------------------------------------------*/
	Result getCamDeviceList(std::vector<ConnDevice>& dev_list);

	/*------------------------------------------------------------------*/
	/// @brief	Add PipeLine processing
	///	@param	[in]	proc			Processing thread to add
	///	@param	[in]	ex_buf_size		Additional buffer size
	///	@param	[out]	proc_id			Processing ID
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_FULL		Over the limit of pipeline threads.
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result addPlProc(std::shared_ptr<EvtThread>& proc, size_t ex_buf_size, uint16_t& proc_id);

	/*------------------------------------------------------------------*/
	/// @brief	Wakeup PipeLine
	///	@param	[in]	dev_id		Device ID
	///	@param	[in]	closed_pl	Closed Pipeline
	///	@param	[in]	notify_cb	Notify callback when closed pipeline
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result wakeupPl(uint16_t dev_id, bool closed_pl = false, const PlCbFunc notify_cb = nullptr);
	/*------------------------------------------------------------------*/
	/// @brief	Shutdown PipeLine
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result shutdownPl(void);

	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::getProperty
	/*------------------------------------------------------------------*/
	Result getCamProperty(uint16_t prop_cmd, void* param);
	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::setProperty
	/*------------------------------------------------------------------*/
	Result setCamProperty(uint16_t prop_cmd, const void* param = nullptr);

	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::startCapture
	/*------------------------------------------------------------------*/
	Result startCapture(void);
	/*------------------------------------------------------------------*/
	/// @copydoc	Camera::stopCapture
	/*------------------------------------------------------------------*/
	Result stopCapture(void);

	/*------------------------------------------------------------------*/
	/// @brief	Get event
	///	@param	[out]	proc_id		Processing ID
	///	@param	[out]	frame		Frame Data
	///	@param	[in]	block		parameter for each events
	///	@retval	SUCCESS			Success
	/// @retval	CANCELED		Canceled wait state
	/// @retval	REACH_EOF		Reached end of file
	/// @retval	ERR_INVALID_PTR	Invalid pointer
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_TIMEOUT		Timeout from waiting
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result getEvent(uint16_t& proc_id, FrameData** frame, bool block = true);
	/*------------------------------------------------------------------*/
	/// @brief	Release frame data
	///	@param	[in,out]	frame	Frame Data
	/// @retval	SUCCESS				Success
	/// @retval ERR_INVALID_PTR		Unknown buffer
	/*------------------------------------------------------------------*/
	Result releaseBuf(FrameData** frame);

	/*------------------------------------------------------------------*/
	/// @brief	Notify event
	///	@param	[in]	proc_id		Processing ID
	///	@param	[in]	event		event for each threads
	///	@param	[in]	param		parameter for each events
	///	@retval	SUCCESS			Success
	/// @retval	ERR_TIMEOUT		Timeout from waiting
	/*------------------------------------------------------------------*/
	Result notifyEvent(uint16_t proc_id, uint8_t event, void* param = nullptr);

private:
	class PipeLine;
	std::unique_ptr<PipeLine> priv_;	 /* private class */
};

} // namespace krm
