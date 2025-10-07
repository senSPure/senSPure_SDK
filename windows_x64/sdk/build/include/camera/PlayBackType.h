/*------------------------------------------------------------------*/
/// @file		PlayBackType.h
/// @brief		PlayBack class definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <filesystem>
#include "CameraType.h"

namespace krm::PlayBack
{
/*-----------------------------------------------------------------*/
/// @brief	Property commands for PlayBack
/*-----------------------------------------------------------------*/
enum PlayBackCmd : uint16_t {
	CMD_PLAY_TARGET = CMD_MAX,	/*!< Target directory | std::filesystem::path */
	CMD_PLAY_TIME,				/*!< Playing Time | PlayTime */
	CMD_PLAY_STATUS,			/*!< Playing Status | PlayStatus */
	CMD_PAUSE,					/*!< Control : Pause | none */
	CMD_FAST_PLAY,				/*!< Control : Fast play | none */
	CMD_SLOW_PLAY,				/*!< Control : Slow play | none */
	CMD_JUMP_FW,				/*!< Control : Jump forward | uint32_t */
	CMD_JUMP_BW,				/*!< Control : Jump backward | uint32_t */
};

/*-----------------------------------------------------------------*/
/// @brief	State of PlayBack
/*-----------------------------------------------------------------*/
enum PlayState {
	STOPPED,	/*!< Stopped */
	PLAYING,	/*!< Playing (normal speed) */
	PAUSE,		/*!< Pause */
	FAST,		/*!< Fast playing */
	SLOW		/*!< Slow playing */
};

/*-----------------------------------------------------------------*/
/// @brief	State of PlayBack
/*-----------------------------------------------------------------*/
struct ConfigParam {
	std::filesystem::path	path;		/*!< Target directory */
};

/*-----------------------------------------------------------------*/
/// @brief	Playing time
/*-----------------------------------------------------------------*/
struct PlayTime {
	uint32_t	total;		/*!< total frames */
	uint32_t	current;	/*!< current frame */
};

/*-----------------------------------------------------------------*/
/// @brief	Playing status
/*-----------------------------------------------------------------*/
struct PlayStatus {
	PlayState	state;			/*!< State of PlayBack */
	uint16_t	playing_fps;	/*!< Playing framerate */
};


} // namespace krm::PlayBack
