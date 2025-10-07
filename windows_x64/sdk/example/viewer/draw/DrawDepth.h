/*------------------------------------------------------------------*/
/// @file		DrawDepth.h
/// @brief		Draw depth image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "DrawImage.h"
#include "ColorTable.h"

namespace krm
{

class DrawDepth : public DrawImage
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	///	@param	[in]	color_table		color table for drawing
	/*------------------------------------------------------------------*/
	explicit DrawDepth(const std::shared_ptr<ColorTable>& color_table);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~DrawDepth(void);

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::setFormat
	/*------------------------------------------------------------------*/
	void setFormat(const ImageFormat& img_fmt) override;

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::clear
	/*------------------------------------------------------------------*/
	void clear(void) override;

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::startSnapshot
	/*------------------------------------------------------------------*/
	void startSnapshot(uint8_t cycle) override;

private:
	std::shared_ptr<ColorTable> color_table_;		/* color table for drawing */
	std::vector<uint16_t>	ave_cnt_;				/* average counter */
	std::vector<uint8_t>	sat_cnt_;				/* saturation counter */

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::convImage
	/*------------------------------------------------------------------*/
	void convImage(const std::vector<uint16_t>& src_img) override;
	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::calcAve
	/*------------------------------------------------------------------*/
	virtual void calcAve(void) override;

	/*------------------------------------------------------------------*/
	/// @brief	calculate snapshot
	/*------------------------------------------------------------------*/
	void calcSnap(const std::vector<uint16_t>& src_img, bool is_last_frame) override;
};

} // namespace krm
