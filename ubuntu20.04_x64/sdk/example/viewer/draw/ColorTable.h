/*------------------------------------------------------------------*/
/// @file		ColorTable.h
/// @brief		Color table class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <mutex>
#include "TpTofSdkDefine.h"
#include "ColorType.h"

namespace krm
{

class ColorTable
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	ColorTable(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~ColorTable(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set color range
	///	@param	[in]	range		color range[mm]
	/*------------------------------------------------------------------*/
	void setColorRange(const Range& range);

	/*------------------------------------------------------------------*/
	/// @brief	Create color bar table
	///	@param	[in]	d_range		Distance range[mm]
	///	@param	[in]	width		image width
	///	@param	[in]	height		image height
	///	@param	[out]	color_bar	color bar image
	/*------------------------------------------------------------------*/
	void createColorBar(const Range& d_range, uint16_t width, uint16_t height, std::vector<RgbaColor>& color_bar);

	/*------------------------------------------------------------------*/
	/// @brief	Convert image to color image
	///	@param	[in]	image		image data
	///	@param	[out]	color_image	image color data
	/*------------------------------------------------------------------*/
	void convColor(const std::vector<uint16_t>& image, std::vector<RgbaColor>& color_image);

	/*------------------------------------------------------------------*/
	/// @brief	Get color
	///	@param	[in]	index		color index
	///	@return	RGBA color value
	/*------------------------------------------------------------------*/
	inline RgbaColor getColor(uint16_t index) { return color_table_[index]; }

	/*------------------------------------------------------------------*/
	/// @brief	Lock table
	/*------------------------------------------------------------------*/
	inline void lock(void) { mtx_.lock(); }
	/*------------------------------------------------------------------*/
	/// @brief	Unlock table
	/*------------------------------------------------------------------*/
	inline void unlock(void) { mtx_.unlock(); }

private:
	static const uint8_t DIV_NUM = 8U;
	static constexpr uint8_t COLOR_AREA_NUM = DIV_NUM + 1U;
	static constexpr RgbaColor COLOR_0 = {127U,   0U,   0U, 255U};
	static constexpr RgbaColor COLOR_1 = {255U,   0U,   0U, 255U};
	static constexpr RgbaColor COLOR_2 = {255U, 127U,   0U, 255U};
	static constexpr RgbaColor COLOR_3 = {255U, 255U,   0U, 255U};
	static constexpr RgbaColor COLOR_4 = {255U, 255U, 127U, 255U};
	static constexpr RgbaColor COLOR_5 = {127U, 255U, 127U, 255U};
	static constexpr RgbaColor COLOR_6 = {  0U, 255U, 255U, 255U};
	static constexpr RgbaColor COLOR_7 = {  0U, 127U, 255U, 255U};
	static constexpr RgbaColor COLOR_8 = {  0U,   0U, 255U, 255U};
	static constexpr std::array<RgbaColor, COLOR_AREA_NUM> COLOR_INDEX = {
		COLOR_0, COLOR_1, COLOR_2, COLOR_3, COLOR_4, COLOR_5, COLOR_6, COLOR_7, COLOR_8
	};

	std::mutex	mtx_;

	Range range_;
	std::array<RgbaColor, UINT16_MAX + 1U>	color_table_;
};

} // namespace krm
