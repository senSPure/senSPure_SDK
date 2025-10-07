/*------------------------------------------------------------------*/
/// @file		DrawThreadEvent.h
/// @brief		Event definitions for Draw Thread
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	Event IDs
/*------------------------------------------------------------------*/
enum DrawEvent : uint8_t {
	EV_SNAP_TAKE,	/*!< Take snapshot | uint8_t */
	EV_SNAP_EXIT,	/*!< Exit snapshot | none */
};

} // namespace krm
