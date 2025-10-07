/*------------------------------------------------------------------*/
/// @file		DrawImage.h
/// @brief		Draw image base class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <mutex>

#include "TpTofSdkDefine.h"
#include "ColorType.h"

namespace krm
{

class DrawImage
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	DrawImage(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	virtual ~DrawImage(void);

	/*------------------------------------------------------------------*/
	/// @brief	set Display scale
	///	@param	[in]	scale	display scale
	/*------------------------------------------------------------------*/
	void setScale(uint16_t scale);

	/*------------------------------------------------------------------*/
	/// @brief	Set cycle for average calculation
	///	@param	[in]	fps		image framerate [fps * 100]
	/*------------------------------------------------------------------*/
	void setAveCycle(uint16_t fps);

	/*------------------------------------------------------------------*/
	/// @brief	Set format
	///	@param	[in]	img_fmt		Image format
	/*------------------------------------------------------------------*/
	virtual void setFormat(const ImageFormat& img_fmt);

	/*------------------------------------------------------------------*/
	/// @brief	Get Draw Image
	///	@return	drawing image pointer
	/*------------------------------------------------------------------*/
	const std::vector<RgbaColor>& getDrawImage(void) { return draw_img_; }

	/*------------------------------------------------------------------*/
	/// @brief	Lock image
	/*------------------------------------------------------------------*/
	inline void lock(void) { mtx_.lock(); }
	/*------------------------------------------------------------------*/
	/// @brief	Unlock image
	/*------------------------------------------------------------------*/
	inline void unlock(void) { mtx_.unlock(); }

	/*------------------------------------------------------------------*/
	/// @brief	Get point value
	///	@param	[in]	point	Screen point
	///	@return	point value
	/*------------------------------------------------------------------*/
	virtual uint16_t getValue(const Point2d& point);

	/*------------------------------------------------------------------*/
	/// @brief	Set image data
	///	@param	[in]	image	image data
	/*------------------------------------------------------------------*/
	void setImage(const ImageData& image);

	/*------------------------------------------------------------------*/
	/// @brief	Calculate snapshot
	///	@param	[in]	image	image data
	/// @return is finished taking snapshot
	/*------------------------------------------------------------------*/
	bool setSnapShot(const ImageData& image);

	/*------------------------------------------------------------------*/
	/// @brief	Refresh drawing image
	/*------------------------------------------------------------------*/
	void refresh(void) { convImage(scale_ > 1U ? scale_img_ : org_img_); }

	/*------------------------------------------------------------------*/
	/// @brief	Clear drawing image
	/*------------------------------------------------------------------*/
	virtual void clear(void);

	/*------------------------------------------------------------------*/
	/// @brief	Start snapshot mode
	/// @param	[in]	cycle	calc cycle
	/*------------------------------------------------------------------*/
	virtual void startSnapshot(uint8_t cycle);
	/*------------------------------------------------------------------*/
	/// @brief	End snapshot mode
	/*------------------------------------------------------------------*/
	virtual void endSnapshot(void);
	/*------------------------------------------------------------------*/
	/// @brief	Get snapshot image
	/// @return	snapshot image
	/*------------------------------------------------------------------*/
	uint16_t* getSnapshotImage(void) { return org_img_.data(); };
	/*------------------------------------------------------------------*/
	/// @brief	Get snapshot std image
	/// @return	snapshot std image
	/*------------------------------------------------------------------*/
	uint16_t* getSnapshotStd(void) { return snap_std_.data(); };

protected:
	std::recursive_mutex	mtx_;
	uint32_t				frm_cnt_;				/* frame counter */
	uint32_t				frm_cycle_;				/* calc average cycle */
	ImageFormat				img_fmt_;				/* image format */
	std::vector<uint16_t>	org_img_;				/* original image */
	std::vector<uint16_t>	scale_img_;				/* resized image */
	std::vector<RgbaColor>	draw_img_;				/* drawing image */
	std::vector<uint32_t>	calc_tmp_;				/* calc average value */
	std::vector<uint16_t>	ave_val_;				/* average value */
	uint8_t					snap_cycle_;			/* calc cycle for snapshot */
	uint16_t				scale_;					/* display scale */
	std::vector<std::vector<uint16_t>> snap_buf_;	/* image buffer for snapshot */
	std::vector<uint16_t>	snap_std_;				/* counter for snapshot */

	/*------------------------------------------------------------------*/
	/// @brief	convert to draw image
	///	@param	[in]	src_img		source image
	/*------------------------------------------------------------------*/
	virtual void convImage(const std::vector<uint16_t>& src_img) = 0;
	/*------------------------------------------------------------------*/
	/// @brief	calculate average
	/*------------------------------------------------------------------*/
	virtual void calcAve(void);
	/*------------------------------------------------------------------*/
	/// @brief	calculate snapshot
	///	@param	[in]	src_img			source image
	///	@param	[in]	is_last_frame	whether this frame is last snapshot frame
	/*------------------------------------------------------------------*/
	virtual void calcSnap(const std::vector<uint16_t>& src_img, bool is_last_frame);

private:
	/*------------------------------------------------------------------*/
	/// @brief	resize to scaled image
	/*------------------------------------------------------------------*/
	void resizeImage(void);
};

} // namespace krm
