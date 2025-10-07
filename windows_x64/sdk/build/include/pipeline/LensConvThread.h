/*------------------------------------------------------------------*/
/// @file		LensConvThread.h
/// @brief		Lens Conversion Thread class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include "EvtThread.h"
#include "LensConv.h"
#include "LensConvType.h"
#include "LensConvThreadEvent.h"

namespace krm
{

class LensConvThread : public EvtThread
{
public:
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::EvtThread
	/// @param	[in]	enable_pcd			Enable to output Point Cloud(default:true)
	/// @param	[in]	enable_distortion	Enable to output after Distortion correct Depth/IR images(default:true)
	/*------------------------------------------------------------------*/
	LensConvThread(bool enable_pcd = true, bool enable_distortion = true);
	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::~EvtThread
	/*------------------------------------------------------------------*/
	~LensConvThread(void); 

	/*------------------------------------------------------------------*/
	/// @brief	Set camera device distortion information
	///	@param	[in]	crct_dist		correcteddistortion in device.
	/*------------------------------------------------------------------*/
	inline void setDevDist(bool crct_dist) { is_dist_corrected_ = crct_dist; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getKind
	/*------------------------------------------------------------------*/
	ProcKind getKind(void) override { return PROC_LENSCONV; }

	/*------------------------------------------------------------------*/
	/// @brief	Point Cloud conversion is enabled?
	/// @retval	true	Enable
	/// @retval	false	Disable
	/*------------------------------------------------------------------*/
	bool getPcdEnabled(void) { return enable_pcd_; }

	/*------------------------------------------------------------------*/
	/// @copydoc	EvtThread::getMaxQueuingFrames
	/*------------------------------------------------------------------*/
	uint8_t getMaxQueuingFrames(void) override { return 8U; }

private:
	LensConv		lens_conv_;			/* Lens Conversion object */
	bool			is_dist_corrected_;	/* Whether Distortion correction of Depth/IR image is already done */
	bool			enable_distortion_;	/* Enable Distortion Correct */
	bool			enable_pcd_;		/* Enable Point Cloud Conversion */
	bool			pcd_world_;			/* Use Pcd world coordinate */
	ImageData		depth_tmp1_;		/* Depth data temporary for distortion correct */
	ImageData		ir_tmp_;			/* IR data temporary for distortion correct */
	PcdColorKind	pcd_color_;			/* Point Cloud color */

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
