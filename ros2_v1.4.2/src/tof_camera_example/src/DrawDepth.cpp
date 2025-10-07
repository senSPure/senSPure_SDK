/*------------------------------------------------------------------*/
/// @file		DrawDepth.h
/// @brief		Draw depth image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include "DrawDepth.h"

namespace tof_camera_example
{

DrawDepth::DrawDepth(const std::shared_ptr<ColorTable>& color_table) :
	color_table_(color_table)
{
	ave_cnt_.clear();
	sat_cnt_.clear();
}

DrawDepth::~DrawDepth(void)
{
	color_table_.reset();
	ave_cnt_.clear();
	sat_cnt_.clear();
}

void DrawDepth::setFormat(const krm::ImageFormat& img_fmt)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	ave_cnt_.resize(img_fmt.pixels);
	sat_cnt_.resize(img_fmt.pixels);
	DrawImage::setFormat(img_fmt);
}

void DrawDepth::clear(void)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	std::fill(ave_cnt_.begin(), ave_cnt_.end(), 0);
	std::fill(sat_cnt_.begin(), sat_cnt_.end(), 0);
	DrawImage::clear();
}

void DrawDepth::convImage(const std::vector<uint16_t>& src_img)
{
	color_table_->convColor(src_img, draw_img_);
}

void DrawDepth::calcAve(void)
{
	uint32_t i;
	frm_cnt_++;
	for (i = 0; i < img_fmt_.pixels; i++) {
		if ((org_img_[i] != krm::SATURATION_DEPTH) && (org_img_[i] != krm::INVALID_DEPTH)) {
			calc_tmp_[i] += static_cast<uint32_t>(org_img_[i]);
			ave_cnt_[i]++;
		}
	}
	if (frm_cnt_ >= frm_cycle_) {
		for (i = 0; i < img_fmt_.pixels; i++) {
			if (ave_cnt_[i] > 0) {
				ave_val_[i] = static_cast<uint16_t>(calc_tmp_[i] / ave_cnt_[i]);
			} else {
				ave_val_[i] = krm::INVALID_DEPTH;
			}
			calc_tmp_[i] = 0;
			ave_cnt_[i] = 0;
		}
		frm_cnt_ = 0;
	}
}

} // namespace tof_camera_example
