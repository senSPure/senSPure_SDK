/*------------------------------------------------------------------*/
/// @file		GrayTable.h
/// @brief		GrayScale table class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <mutex>
#include "TpTofSdkDefine.h"
#include "ColorType.h"

namespace krm
{

class GrayTable
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	GrayTable(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~GrayTable(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set Gain coefficient
	///	@param	[in]	gain	Gain coefficient
	/*------------------------------------------------------------------*/
	void setGain(float gain);
	/*------------------------------------------------------------------*/
	/// @brief	Set Gamma coefficient
	///	@param	[in]	gamma	Gamma coefficient
	/*------------------------------------------------------------------*/
	void setGamma(float gamma);

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
	RgbaColor getColor(uint16_t index);

	/*------------------------------------------------------------------*/
	/// @brief	Lock table
	/*------------------------------------------------------------------*/
	inline void lock(void) { mtx_.lock(); }
	/*------------------------------------------------------------------*/
	/// @brief	Unlock table
	/*------------------------------------------------------------------*/
	inline void unlock(void) { mtx_.unlock(); }

private:
	static constexpr uint32_t TABLE_NUM = UINT16_MAX + 1U;
	std::mutex	mtx_;
	float		gain_;
	float		gamma_;
	std::array<uint16_t, TABLE_NUM>		gamma_table_;
	std::array<RgbaColor, TABLE_NUM>	gray_table_;
};

} // namespace krm
