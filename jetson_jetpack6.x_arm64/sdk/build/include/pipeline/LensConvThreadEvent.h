/*------------------------------------------------------------------*/
/// @file		LensConvThreadEvent.h
/// @brief		Event definitions for LensConv Thread
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	Event IDs
/*------------------------------------------------------------------*/
enum LensConvEvent : uint8_t {
	EV_LENS_PSBL_DIST,		/*!< Check whether Distortion Correction is possible | bool */
	EV_LENS_DIST,			/*!< Switch use Distortion Correct | bool */
	EV_LENS_PCD_KIND,		/*!< Switch use coordinate in PCD conversion | bool(false:camera, true:world) */
	EV_LENS_PCD_ORG_POS,	/*!< Set origin point and rotation in world coordinate | PosOrgRotation */
	EV_LENS_PCD_COLOR		/*!< Set Color data in Point Cloud data | PcdColorKind */
};

/*------------------------------------------------------------------*/
/// @brief	Kind of color in Point Cloud
/*------------------------------------------------------------------*/
enum PcdColorKind : uint8_t {
	PCD_COLOR_NONE,		/*!< not set color(default) */
	PCD_COLOR_IR		/*!< set IR value */
};

} // namespace krm
