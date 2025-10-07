/*------------------------------------------------------------------*/
/// @file		ColorType.h
/// @brief		Color definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>
#include <array>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	RGBA color
/*------------------------------------------------------------------*/
static const uint8_t COLOR_NUM = 4U;
using RgbaColor = std::array<uint8_t, COLOR_NUM>;
static const RgbaColor BLACK_COLOR = {0, 0, 0, 0xFFU};
static const RgbaColor WHITE_COLOR = {0xFFU, 0xFFU, 0xFFU, 0xFFU};

} // namespace krm
