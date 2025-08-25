/*------------------------------------------------------------------*/
/// @file		DrawDepth.h
/// @brief		Draw depth image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cmath>
#include <algorithm>
#include "DrawDepth.h"

namespace krm
{

DrawDepth::DrawDepth(const std::shared_ptr<ColorTable>& color_table) :
	color_table_(color_table)
{
	ave_cnt_.clear();
	sat_cnt_.clear();
	snap_buf_.clear();
	snap_std_.clear();
}

DrawDepth::~DrawDepth(void)
{
	color_table_.reset();
	ave_cnt_.clear();
	sat_cnt_.clear();
	snap_buf_.clear();
	snap_std_.clear();
}

void DrawDepth::setFormat(const ImageFormat& img_fmt)
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
	endSnapshot();
}

void DrawDepth::startSnapshot(uint8_t cycle)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	std::fill(ave_cnt_.begin(), ave_cnt_.end(), 0);
	std::fill(sat_cnt_.begin(), sat_cnt_.end(), 0);
	DrawImage::startSnapshot(cycle);
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
		if ((org_img_[i] != SATURATION_DEPTH) && (org_img_[i] != INVALID_DEPTH)) {
			calc_tmp_[i] += static_cast<uint32_t>(org_img_[i]);
			ave_cnt_[i]++;
		}
	}
	if (frm_cnt_ >= frm_cycle_) {
		for (i = 0; i < img_fmt_.pixels; i++) {
			if (ave_cnt_[i] > 0) {
				ave_val_[i] = static_cast<uint16_t>(calc_tmp_[i] / ave_cnt_[i]);
			} else {
				ave_val_[i] = INVALID_DEPTH;
			}
			calc_tmp_[i] = 0;
			ave_cnt_[i] = 0;
		}
		frm_cnt_ = 0;
	}
}

void DrawDepth::calcSnap(const std::vector<uint16_t>& src_img, bool is_last_frame)
{
	uint32_t i;
	for (i = 0; i < img_fmt_.pixels; i++) {
		snap_buf_[frm_cnt_][i] = src_img[i];
		if (src_img[i] == SATURATION_DEPTH) {
			sat_cnt_[i]++;
		} else if (src_img[i] != INVALID_DEPTH) {
			calc_tmp_[i] += static_cast<uint32_t>(src_img[i]);
			ave_cnt_[i]++;
		}
	}
	if (is_last_frame) {
		float tmp, ave_tmp;
		double std_tmp;
		uint8_t half_snap_cycle = snap_cycle_ / 2U;
		uint32_t u32_std;
		for (i = 0; i < img_fmt_.pixels; i++) {
			if (ave_cnt_[i] > 0) {
				std_tmp = 0.0;
				ave_tmp = static_cast<float>(calc_tmp_[i]) / static_cast<float>(ave_cnt_[i]);
				org_img_[i] = ave_val_[i] = static_cast<uint16_t>((((calc_tmp_[i] * 10U) / static_cast<uint32_t>(ave_cnt_[i])) + 5U) / 10U);
				for (auto& buf : snap_buf_) {
					if ((buf[i] != SATURATION_DEPTH) && (buf[i] != INVALID_DEPTH)) {
						tmp = static_cast<float>(buf[i]) - ave_tmp;
						std_tmp += static_cast<double>(tmp) * static_cast<double>(tmp);
					}
				}
				std_tmp /= static_cast<double>(ave_cnt_[i]);
				u32_std = (static_cast<uint32_t>(std::sqrt(std_tmp) * 100.0) + 5U) / 10U;
				snap_std_[i] = (u32_std > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(u32_std);
			} else {
				org_img_[i] = (sat_cnt_[i] > half_snap_cycle) ? SATURATION_DEPTH : INVALID_DEPTH;
				ave_val_[i] = INVALID_DEPTH;
				snap_std_[i] = 0;
			}
			calc_tmp_[i] = 0;
			ave_cnt_[i] = 0;
			sat_cnt_[i] = 0;
		}
	}
}

} // namespace krm
