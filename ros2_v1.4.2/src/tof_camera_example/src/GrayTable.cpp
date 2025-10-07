/*------------------------------------------------------------------*/
/// @file		GrayTable.cpp
/// @brief		GrayScale table class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cmath>
#include "GrayTable.h"

namespace tof_camera_example
{

GrayTable::GrayTable(void) :
	gain_(1.0F), gamma_(0.0F)
{
	uint16_t coef = UINT16_MAX / UINT8_MAX;
	RgbaColor* data = gray_table_.data();

	for (uint32_t i = 0; i < TABLE_NUM; i++) {
		data->at(0) = data->at(1) = data->at(2) = static_cast<uint8_t>(i / coef);
		data->at(3) = 0xFFU;
		data++;
	}
	setGamma(1.0F);
}

GrayTable::~GrayTable(void)
{
}

void GrayTable::setGain(float gain)
{
	std::lock_guard<std::mutex>		lock(mtx_);
	gain_ = gain;
}

void GrayTable::setGamma(float gamma)
{
	std::lock_guard<std::mutex>		lock(mtx_);
	double		gmm = 1.0 / static_cast<double>(gamma);
	uint16_t*	data = gamma_table_.data();
	double		max = static_cast<double>(UINT16_MAX);

	for (uint32_t i = 0; i < TABLE_NUM; i++) {
		*(data++) = static_cast<uint16_t>(std::pow(static_cast<double>(i) / max, gmm) * max);
	}

	gamma_ = gamma;
}

void GrayTable::convColor(const std::vector<uint16_t>& image, std::vector<RgbaColor>& color_image)
{
	std::lock_guard<std::mutex>		lock(mtx_);
	const uint16_t* org = image.data();
	const uint16_t* end = org + image.size();
	RgbaColor* dst = color_image.data();
	uint32_t tmp;

	for (; org < end; org++, dst++) {
		tmp = static_cast<uint32_t>(static_cast<float>(*org) * gain_);
		if (tmp > UINT16_MAX) {
			tmp = UINT16_MAX;
		}
		*dst = gray_table_[gamma_table_[tmp]];
	}
}

RgbaColor GrayTable::getColor(uint16_t index)
{
	uint32_t tmp = static_cast<uint32_t>(static_cast<float>(index) * gain_);
	if (tmp > UINT16_MAX) {
		tmp = UINT16_MAX;
	}
	return gray_table_[gamma_table_[tmp]];
}

} // namespace tof_camera_example
