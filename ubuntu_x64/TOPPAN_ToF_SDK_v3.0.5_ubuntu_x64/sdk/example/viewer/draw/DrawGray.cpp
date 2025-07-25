/*------------------------------------------------------------------*/
/// @file		DrawGray.h
/// @brief		Draw grayscale image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include "DrawGray.h"

namespace krm
{

DrawGray::DrawGray(const std::shared_ptr<GrayTable>& gray_table) :
	gray_table_(gray_table)
{
}

DrawGray::~DrawGray(void)
{
}

void DrawGray::convImage(const std::vector<uint16_t>& src_img)
{
	gray_table_->convColor(src_img, draw_img_);
}

} // namespace krm
