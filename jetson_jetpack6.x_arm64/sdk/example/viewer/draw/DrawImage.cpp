/*------------------------------------------------------------------*/
/// @file		DrawImage.cpp
/// @brief		Draw image base class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <algorithm>
#include <opencv2/opencv.hpp>
#include "DrawImage.h"
#include "CommonLog.h"

namespace krm
{

DrawImage::DrawImage(void) :
	frm_cnt_(0), frm_cycle_(1U), snap_cycle_(1U), scale_(1U)
{
	img_fmt_.set();
	org_img_.clear();
	scale_img_.clear();
	draw_img_.clear();
	calc_tmp_.clear();
	ave_val_.clear();
}

DrawImage::~DrawImage(void)
{
	org_img_.clear();
	scale_img_.clear();
	draw_img_.clear();
	calc_tmp_.clear();
	ave_val_.clear();
}

void DrawImage::setScale(uint16_t scale)
{
	if (scale > 0) { scale_ = scale; }
}

void DrawImage::setAveCycle(uint16_t fps)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	frm_cnt_ = 0;
	frm_cycle_ = fps / 200U;	// (fps / 100) / 2
	if (frm_cycle_ == 0) {
		frm_cycle_ = 1U;
	}
}

void DrawImage::setFormat(const ImageFormat& img_fmt)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	img_fmt_ = img_fmt;
	org_img_.resize(img_fmt.pixels);
	if (scale_ > 1U) {
		uint32_t pixels = img_fmt.pixels * scale_ * scale_;
		scale_img_.resize(pixels);
		draw_img_.resize(pixels);
	} else {
		scale_img_.clear();
		draw_img_.resize(img_fmt.pixels);
	}
	calc_tmp_.resize(img_fmt.pixels);
	ave_val_.resize(img_fmt.pixels);
	clear();
}

uint16_t DrawImage::getValue(const Point2d& point)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	if (point.x >= img_fmt_.width) {
		return 0;
	}
	if (point.y >= img_fmt_.height) {
		return 0;
	}
	return ave_val_[
			(static_cast<uint32_t>(point.y) * static_cast<uint32_t>(img_fmt_.width))
			 + static_cast<uint32_t>(point.x)];
}

void DrawImage::setImage(const ImageData& image)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	std::copy_n(image.data.cbegin(), org_img_.size(), org_img_.begin());
	if (scale_ > 1U) {
		resizeImage();
		convImage(scale_img_);
	} else {
		convImage(org_img_);
	}
	calcAve();
}

bool DrawImage::setSnapShot(const ImageData& image)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	bool is_last_frame = false;
	if (frm_cnt_ < static_cast<uint32_t>(snap_cycle_)) {
		if (frm_cnt_ == 0) {
			std::copy_n(image.data.cbegin(), org_img_.size(), org_img_.begin());
			std::copy_n(image.data.cbegin(), ave_val_.size(), ave_val_.begin());
		}

		is_last_frame = ((frm_cnt_ + 1U) == static_cast<uint32_t>(snap_cycle_));
		calcSnap(image.data, is_last_frame);

		if (is_last_frame) {
			if (scale_ > 1U) {
				resizeImage();
				convImage(scale_img_);
			} else {
				convImage(org_img_);
			}
		}
		frm_cnt_++;
	}
	return is_last_frame;
}

void DrawImage::clear(void)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	std::fill(org_img_.begin(), org_img_.end(), 0);
	std::fill(scale_img_.begin(), scale_img_.end(), 0);
	std::fill(draw_img_.begin(), draw_img_.end(), BLACK_COLOR);
	std::fill(calc_tmp_.begin(), calc_tmp_.end(), 0);
	std::fill(ave_val_.begin(), ave_val_.end(), 0);
	frm_cnt_ = 0;
}

void DrawImage::startSnapshot(uint8_t cycle)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	std::fill(draw_img_.begin(), draw_img_.end(), BLACK_COLOR);
	std::fill(calc_tmp_.begin(), calc_tmp_.end(), 0);
	snap_buf_.resize(cycle);
	for (auto &buf : snap_buf_) {
		buf.resize(img_fmt_.pixels);
	}
	snap_std_.resize(img_fmt_.pixels);
	std::fill(snap_std_.begin(), snap_std_.end(), 0);
	frm_cnt_ = 0;
	snap_cycle_ = cycle;
}

void DrawImage::endSnapshot(void)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	frm_cnt_ = 0;
	snap_buf_.clear();
	snap_buf_.shrink_to_fit();
	snap_std_.clear();
	snap_std_.shrink_to_fit();
}

void DrawImage::calcAve(void)
{
	uint32_t i;
	frm_cnt_++;
	for (i = 0; i < img_fmt_.pixels; i++) {
		calc_tmp_[i] += static_cast<uint32_t>(org_img_[i]);
	}
	if (frm_cnt_ >= frm_cycle_) {
		for (i = 0; i < img_fmt_.pixels; i++) {
			ave_val_[i] = static_cast<uint16_t>(calc_tmp_[i] / frm_cycle_);
			calc_tmp_[i] = 0;
		}
		frm_cnt_ = 0;
	}
}

void DrawImage::calcSnap(const std::vector<uint16_t>& src_img, bool is_last_frame)
{
	uint32_t i;
	for (i = 0; i < img_fmt_.pixels; i++) {
		snap_buf_[frm_cnt_][i] = src_img[i];
		calc_tmp_[i] += static_cast<uint32_t>(src_img[i]);
	}
	if (is_last_frame) {
		float tmp, ave_tmp;
		double std_tmp;
		uint32_t u32_std;
		for (i = 0; i < img_fmt_.pixels; i++) {
			std_tmp = 0.0;
			ave_tmp = static_cast<float>(calc_tmp_[i]) / snap_cycle_;
			org_img_[i] = ave_val_[i] = static_cast<uint16_t>((((calc_tmp_[i] * 10U) / snap_cycle_) + 5U) / 10U);
			for (auto& buf : snap_buf_) {
				tmp = static_cast<float>(buf[i]) - ave_tmp;
				std_tmp += static_cast<double>(tmp) * static_cast<double>(tmp);
			}
			std_tmp /= static_cast<double>(snap_cycle_);
			u32_std = (static_cast<uint32_t>(std::sqrt(std_tmp) * 100.0) + 5U) / 10U;
			snap_std_[i] = (u32_std > UINT16_MAX) ? UINT16_MAX : static_cast<uint16_t>(u32_std);
			calc_tmp_[i] = 0;
		}
	}
}

void DrawImage::resizeImage(void)
{
	if (img_fmt_.pixels > 0) {
		cv::Size dst_size(cv::Size(img_fmt_.width * scale_, img_fmt_.height * scale_));
		cv::Mat org_mat(cv::Size(img_fmt_.width, img_fmt_.height), CV_16UC1, org_img_.data());
		cv::Mat dst_mat(dst_size, CV_16UC1, scale_img_.data());
		cv::resize(org_mat, dst_mat, dst_size, 0, 0, cv::INTER_NEAREST);
	}
}

} // namespace krm
