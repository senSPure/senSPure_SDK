/*------------------------------------------------------------------*/
/// @file		DrawImage.h
/// @brief		Draw image base class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <mutex>

#include "TpTofSdkDefine.h"
#include "ColorType.h"

namespace tof_camera_example
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
	virtual void setFormat(const krm::ImageFormat& img_fmt);

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
	virtual uint16_t getValue(const krm::Point2d& point);

	/*------------------------------------------------------------------*/
	/// @brief	Set image data
	///	@param	[in]	image	image data
	/*------------------------------------------------------------------*/
	void setImage(const krm::ImageData& image);

	/*------------------------------------------------------------------*/
	/// @brief	Refresh drawing image
	/*------------------------------------------------------------------*/
	void refresh(void) { convImage(scale_ > 1U ? scale_img_ : org_img_); }

	/*------------------------------------------------------------------*/
	/// @brief	Clear drawing image
	/*------------------------------------------------------------------*/
	virtual void clear(void);

protected:
	std::recursive_mutex	mtx_;
	uint32_t				frm_cnt_;				/* frame counter */
	uint32_t				frm_cycle_;				/* calc average cycle */
	krm::ImageFormat		img_fmt_;				/* image format */
	std::vector<uint16_t>	org_img_;				/* original image */
	std::vector<uint16_t>	scale_img_;				/* resized image */
	std::vector<RgbaColor>	draw_img_;				/* drawing image */
	std::vector<uint32_t>	calc_tmp_;				/* calc average value */
	std::vector<uint16_t>	ave_val_;				/* average value */
	uint16_t				scale_;					/* display scale */

	/*------------------------------------------------------------------*/
	/// @brief	convert to draw image
	///	@param	[in]	src_img		source image
	/*------------------------------------------------------------------*/
	virtual void convImage(const std::vector<uint16_t>& src_img) = 0;
	/*------------------------------------------------------------------*/
	/// @brief	calculate average
	/*------------------------------------------------------------------*/
	virtual void calcAve(void);

private:
	/*------------------------------------------------------------------*/
	/// @brief	resize to scaled image
	/*------------------------------------------------------------------*/
	void resizeImage(void);
};

} // namespace tof_camera_example
