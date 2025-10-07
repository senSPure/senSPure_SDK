/*------------------------------------------------------------------*/
/// @file		RecordThreadEvent.h
/// @brief		Event definitions for Record Thread
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>
#include <filesystem>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	Event IDs
/*------------------------------------------------------------------*/
enum RecEvent : uint8_t {
	EV_REC_START,		/*!< Start recording | RecordParam */
	EV_REC_STOP,		/*!< Stop recording | none */
};

/*------------------------------------------------------------------*/
/// @brief	Record parameter
/*------------------------------------------------------------------*/
struct RecordParam {
	std::filesystem::path	path;				/*!< Record target path */
	uint32_t				save_frames;		/*!< Number of save frames */
	uint16_t				packing_frames;		/*!< Number of packing frames in a file */
	bool					is_filt_med;		/*!< Whether the image is a median filter applied */
	bool                  is_filt_bil;
	bool					is_filt_fly_p;		/*!< Whether the image is a flying pixel filter applied */
	bool					is_crct_dist;		/*!< Distortion correction is already done */
};

} // namespace krm
