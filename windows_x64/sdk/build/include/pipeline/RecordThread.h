/*------------------------------------------------------------------*/
/// @file		RecordThread.h
/// @brief		Record Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include "EvtThread.h"
#include "Record.h"
#include "RecordThreadEvent.h"

namespace krm
{

class RecordThread : public EvtThread
{
public:
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::EvtThread
	/*------------------------------------------------------------------*/
	RecordThread(void);
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::~EvtThread
	/*------------------------------------------------------------------*/
	~RecordThread(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set Camera object
	///	@param	[in]	camera	camera class object
	/*------------------------------------------------------------------*/
	inline void setCamera(std::shared_ptr<Camera>& camera) { camera_ = camera; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getKind
	/*------------------------------------------------------------------*/
	ProcKind getKind(void) override { return PROC_RECORD; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getMaxQueuingFrames
	/*------------------------------------------------------------------*/
	uint8_t getMaxQueuingFrames(void) override { return 8U; }

private:
	std::shared_ptr<Camera>	camera_;	/* Camera class object */
	Record::Record			rec_;		/* Record object */
	bool					recording_;	/* is Recording? */

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::enableFrameDrop
	/*------------------------------------------------------------------*/
	bool enableFrameDrop(void) override { return true; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvImage
	/*------------------------------------------------------------------*/
	Result recvImage(FrameData& frame) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvUserEvent
	/*------------------------------------------------------------------*/
	Result recvUserEvent(uint8_t event, void* param) override;
};

} // namespace krm
