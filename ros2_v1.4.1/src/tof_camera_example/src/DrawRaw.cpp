/*------------------------------------------------------------------*/
/// @file		DrawRaw.h
/// @brief		Draw raw image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include "DrawRaw.h"

namespace tof_camera_example
{

DrawRaw::DrawRaw(uint8_t index, std::shared_ptr<GrayTable>& gray_table) :
	DrawGray(gray_table), raw_idx_(index)
{
}

DrawRaw::~DrawRaw(void)
{
}

uint16_t DrawRaw::getValue(const krm::Point2d& point)
{
	krm::Point2d img_point = point;
	img_point.x += img_fmt_.active_start.x;
	img_point.y += img_fmt_.active_start.y;
	return DrawGray::getValue(img_point);
}

bool DrawRaw::convPoint2d(const krm::Point2d& screen, krm::Point2d& point)
{
	uint8_t x_idx = raw_idx_ % 2U;
	uint8_t y_idx = raw_idx_ / 2U;
	uint16_t x = screen.x / scale_;
	uint16_t y = screen.y / scale_;
	krm::Point2d offset;

	offset.x = img_fmt_.width  * x_idx;
	offset.y = img_fmt_.height * y_idx;

	if (offset.x > x) { return false; }
	if (offset.y > y) { return false; }

	x -= offset.x;
	y -= offset.y;

	if (img_fmt_.width  <= x) { return false; }
	if (img_fmt_.height <= y) { return false; }

	if (img_fmt_.active_start.x > x) { return false; }
	if (img_fmt_.active_start.y > y) { return false; }

	x -= img_fmt_.active_start.x;
	y -= img_fmt_.active_start.y;

	if (img_fmt_.active_w <= x) { return false; }
	if (img_fmt_.active_h <= y) { return false; }

	point.x = x;
	point.y = y;
	return true;
}

} // namespace tof_camera_example
