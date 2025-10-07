/*------------------------------------------------------------------*/
/// @file		DrawGray.h
/// @brief		Draw raw image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include "DrawGray.h"

namespace krm
{

class DrawRaw : public DrawGray
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	///	@param	[in]	index			RAW index(0-)
	///	@param	[in]	gray_table		Grayscale table
	/*------------------------------------------------------------------*/
	DrawRaw(uint8_t index, std::shared_ptr<GrayTable>& gray_table);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~DrawRaw(void);

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::getValue
	/*------------------------------------------------------------------*/
	uint16_t getValue(const Point2d& point) override;

	/*------------------------------------------------------------------*/
	/// @brief	Convert Screen point to image point
	///	@param	[in]	screen	Screen point
	///	@param	[out]	point	image point
	///	@retval	true	screen point is into this image
	///	@retval	false	screen point is out of this image
	/*------------------------------------------------------------------*/
	bool convPoint2d(const Point2d& screen, Point2d& point);

private:
	uint8_t		raw_idx_;	/* RAW index */
};

} // namespace krm
