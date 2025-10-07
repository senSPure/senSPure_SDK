/*------------------------------------------------------------------*/
/// @file		PostFilterThread.h
/// @brief		PostFilter Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include "EvtThread.h"
#include "PostFilter.h"
#include "PostFilterType.h"
#include "PostFilterThreadEvent.h"

namespace krm
{

class PostFilterThread : public EvtThread
{
public:
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::EvtThread
	/// @param	[in]	enable_medf		Enable to median filter(default:true)
    /// @param  [in]    enable_bilf     Enable to bilateral filter(default:true)
	/// @param	[in]	enable_flypf	Enable to flying pixel filter(default:true)
	/*------------------------------------------------------------------*/
	PostFilterThread(bool enable_medf = true, bool enable_bilf = true, bool enable_flypf = true);
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::~EvtThread
	/*------------------------------------------------------------------*/
	~PostFilterThread(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set camera device median filter information
	///	@param	[in]	filt_med		median filter in device.
	/*------------------------------------------------------------------*/
	inline void setDevMedFilt(bool filt_med) { is_filt_med_ = filt_med; }

	/*------------------------------------------------------------------*/
	/// @brief	Set camera device bilateral filter information
	///	@param	[in]	filt_bil	birateral filter in device.
	/*------------------------------------------------------------------*/
	inline void setDevBilFilt(bool filt_bil) { is_filt_bil_ = filt_bil; }
	/*------------------------------------------------------------------*/
	/// @brief	Set camera device flying pixel filter information
	///	@param	[in]	filt_flyp		flying pixel filter in device.
	/*------------------------------------------------------------------*/
	inline void setDevFlyPFilt(bool filt_flyp) { is_filt_fly_p_ = filt_flyp; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getKind
	/*------------------------------------------------------------------*/
	ProcKind getKind(void) override { return PROC_POSTFILT; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getMaxQueuingFrames
	/*------------------------------------------------------------------*/
	uint8_t getMaxQueuingFrames(void) override { return 8U; }

private:
	PostFilter		post_filter_;		/* Lens Conversion object */
	bool			is_filt_med_;		/* Whether the image is a median filter applied */
	bool			is_filt_bil_;
	bool			is_filt_fly_p_;		/* Whether the image is a flying pixel filter applied */
	bool			enable_medf_;		/* Enable Median Filter */
	bool			enable_bilf_;		/* Enable Bilateral Filter */
	bool			enable_flypf_;		/* Enable Flying Pixel Filter */
	ImageData		depth_tmp1_;		/* Depth data temporary for median filter */
	ImageData		ir_tmp_;			/* IR data temporary for distortion correct */



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
	/// @copydoc	EvtThread::recvUserEvent
	/*------------------------------------------------------------------*/
	Result recvUserEvent(uint8_t event, void* param) override;
};

} // namespace krm
