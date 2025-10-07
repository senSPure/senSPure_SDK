/*------------------------------------------------------------------*/
/// @file		RecordType.h
/// @brief		Record class definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <filesystem>
#include "TpTofSdkDefine.h"
#include "Camera.h"

namespace krm::Record
{

/*-----------------------------------------------------------------*/
/// @brief	Configuration Parameter
/*-----------------------------------------------------------------*/
struct ConfigParam {
	std::filesystem::path	path;				/*!< Target path */
	uint32_t				save_frames;		/*!< Number of save frames */
	uint16_t				packing_frames;		/*!< Number of packing frames in a file */
	Camera*					cam_obj;			/*!< Camera class object */
	bool					is_crct_dist;		/*!< Distortion correction is already done */
	bool					is_filt_med;		/*!< Whether the image is a median filter applied */
	bool					is_filt_bil;		/*!< Whether the image is a bilateral filter applied */
	bool					is_filt_fly_p;		/*!< Whether the image is a flying pixel filter applied */
};

/*-----------------------------------------------------------------*/
/// @brief	Recording Parameter
/*-----------------------------------------------------------------*/
struct RecInfoParam {
	std::filesystem::path	path;				/*!< Target path */
	uint32_t				save_frames;		/*!< Number of save frames */
	uint16_t				packing_frames;		/*!< Number of packing frames in a file */
	DeviceInfo				device_info;		/*!< device information */
	LensInfo				lens_info;			/*!< lens information */
	CamFov					fov;				/*!< Field ov volume */
	ModeInfo				mode_info;			/*!< Current motion mode information */
	ImageFormats			image_formats;		/*!< Available image formats */
	bool					is_crct_dist;		/*!< Distortion correction is already done */
	bool					is_filt_med;		/*!< Whether the image is a median filter applied */
	bool					is_filt_bil;
	bool					is_filt_fly_p;		/*!< Whether the image is a flying pixel filter applied */
};

} // namespace krm::Record
