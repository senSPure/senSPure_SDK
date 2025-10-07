/*------------------------------------------------------------------*/
/// @file		LensType.h
/// @brief		LensConv class definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>

namespace krm
{
/*------------------------------------------------------------------*/
/// @brief	Indicates the origin position and rotation information in point cloud conversion to the world coordinate system
/*------------------------------------------------------------------*/
struct PosOrgRotation {
	struct {
		int16_t x;		/*!< Origin position: X-axis direction offset [mm] */
		int16_t y;		/*!< Origin position: Y-axis direction offset [mm] */
		int16_t z;		/*!< Origin position: Z-axis direction offset [mm] */
	} offset;
	struct {
		float pitch;	/*!< Pitch rotation angle [degree] */
		float yaw;		/*!< Yaw   rotation angle [degree] */
		float roll;		/*!< Roll  rotation angle [degree] */
	} rotation;
};
} // namespace krm
