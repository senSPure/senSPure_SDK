/*------------------------------------------------------------------*/
/// @file		ColorTable.cpp
/// @brief		Color table class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cmath>
#include "ColorTable.h"

namespace tof_camera_example
{

ColorTable::ColorTable(void) :
	range_({0, 0})
{
	color_table_.fill(BLACK_COLOR);
}

ColorTable::~ColorTable(void)
{
}

void ColorTable::setColorRange(const krm::Range& range)
{
	std::lock_guard<std::mutex>		lock(mtx_);

	if ((range_.min != range.min) || (range_.max != range.max)) {
		RgbaColor	color;
		RgbaColor	org_color;
		uint8_t		i, k;
		uint16_t	idx, offset;
		uint16_t	max = static_cast<uint16_t>(color_table_.size()) - 1U;
		std::array<krm::Range, DIV_NUM>	color_range;
		std::array<std::array<float, COLOR_AREA_NUM>, DIV_NUM>		color_diff;
		uint16_t	div_part;
		uint16_t	diff;

		range_ = range;

		if (range_.max == UINT16_MAX) {
			range_.max--;
		}
		diff = range_.max - range_.min;

		if (diff >= DIV_NUM) {
			div_part = (range_.max - range_.min) / static_cast<uint16_t>(DIV_NUM);
			for (i = 0; i < DIV_NUM; i++) {
				color_range[i].min = range_.min + (div_part * static_cast<uint16_t>(i));
				color_range[i].max = color_range[i].min + div_part;
			}
			color_range[DIV_NUM - 1].max = range.max;

			for (i = 0; i < DIV_NUM; i++) {
				k = i + 1U;
				color_diff[i][0] = static_cast<float>(static_cast<int16_t>(COLOR_INDEX[k][0]) - static_cast<int16_t>(COLOR_INDEX[i][0])) / static_cast<float>(div_part);
				color_diff[i][1] = static_cast<float>(static_cast<int16_t>(COLOR_INDEX[k][1]) - static_cast<int16_t>(COLOR_INDEX[i][1])) / static_cast<float>(div_part);
				color_diff[i][2] = static_cast<float>(static_cast<int16_t>(COLOR_INDEX[k][2]) - static_cast<int16_t>(COLOR_INDEX[i][2])) / static_cast<float>(div_part);
			}

			for (i = 0; i < DIV_NUM; i++) {
				color = org_color = COLOR_INDEX[i];
				for (idx = color_range[i].min; idx <= color_range[i].max; idx++) {
					color_table_[idx] = color;
					offset = (idx - color_range[i].min);
					color[0] = static_cast<uint8_t>(std::abs(
						static_cast<int16_t>(static_cast<float>(org_color[0]) + (color_diff[i][0] * static_cast<float>(offset)))));
					color[1] = static_cast<uint8_t>(std::abs(
						static_cast<int16_t>(static_cast<float>(org_color[1]) + (color_diff[i][1] * static_cast<float>(offset)))));
					color[2] = static_cast<uint8_t>(std::abs(
						static_cast<int16_t>(static_cast<float>(org_color[2]) + (color_diff[i][2] * static_cast<float>(offset)))));
				}
			}
		} else {
			idx = range_.min;
			for (i = 0; i <= diff; i++) {
				color_table_[idx++] = COLOR_INDEX[i];
			}
		}

		for (idx = range_.max; idx < max; idx++) {
			color_table_[idx] = COLOR_8;
		}
		for (idx = 0; idx < range_.min; idx++) {
			color_table_[idx] = COLOR_0;
		}
		color_table_[UINT16_MAX] = BLACK_COLOR;
		color_table_[0] = WHITE_COLOR;
	}
}

void ColorTable::createColorBar(const krm::Range& d_range, uint16_t width, uint16_t height, std::vector<RgbaColor>& color_bar)
{
	std::lock_guard<std::mutex>		lock(mtx_);
	uint16_t x, y;
	RgbaColor* data;
	uint16_t offset = d_range.min;
	uint16_t tmp;
	uint16_t diff = d_range.max - d_range.min + 1U;

	color_bar.resize(width * height);
	data = color_bar.data();
	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			tmp = offset + ((x * diff) / width);
			if (tmp < d_range.min) { tmp = d_range.min; }
			if (tmp > d_range.max) { tmp = d_range.max; }
			*(data++) = color_table_[tmp];
		}
	}
}

void ColorTable::convColor(const std::vector<uint16_t>& image, std::vector<RgbaColor>& color_image)
{
	const uint16_t* org = image.data();
	const uint16_t* end = org + image.size();
	RgbaColor* dst = color_image.data();

	for (; org < end; org++, dst++) {
		*dst = color_table_[*org];
	}
}

} // namespace tof_camera_example
