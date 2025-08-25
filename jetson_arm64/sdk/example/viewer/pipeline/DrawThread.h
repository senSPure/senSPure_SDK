/*------------------------------------------------------------------*/
/// @file		DrawThread.h
/// @brief		Draw image Conversion Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>
#include "EvtThread.h"
#include "DrawImage.h"
#include "DrawThreadEvent.h"

namespace krm
{

static constexpr uint8_t DRAW_KINDS = (IMG_KINDS + 1U);
using DrawImages = std::array<std::shared_ptr<DrawImage>, DRAW_KINDS>;

class DrawThread : public EvtThread
{
public:
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::EvtThread
	///	@param	[in]	draws		Drawing classes
	/*------------------------------------------------------------------*/
	explicit DrawThread(DrawImages& draws);
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::~EvtThread
	/*------------------------------------------------------------------*/
	~DrawThread(void);

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getMaxQueuingFrames
	/*------------------------------------------------------------------*/
	uint8_t getMaxQueuingFrames(void) override { return 8U; }

private:
	DrawImages		draws_;			/* drawing classes */
	bool			snap_taking_;	/* is snapshot mode */

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::enableFrameDrop
	/*------------------------------------------------------------------*/
	bool enableFrameDrop(void) override { return true; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvChgProp
	/*------------------------------------------------------------------*/
	Result recvChgProp(const CameraProperty& property) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvChgFmt
	/*------------------------------------------------------------------*/
	Result recvChgFmt(const ImageFormats& formats) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvImage
	/*------------------------------------------------------------------*/
	Result recvImage(FrameData& frame) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvReset
	/*------------------------------------------------------------------*/
	void recvReset(void) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::recvUserEvent
	/*------------------------------------------------------------------*/
	Result recvUserEvent(uint8_t event, void* param) override;
};

} // namespace krm
