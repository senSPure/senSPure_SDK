/*------------------------------------------------------------------*/
/// @file		PostFilterThreadEvent.h
/// @brief		Event definitions for PostFilter Thread
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	Event IDs
/*------------------------------------------------------------------*/
enum PostFilterEvent : uint8_t {
	EV_POSTFILT_PSBL_MEDF,	/*!< Check whether Median Filter is possible | bool */
	EV_POSTFILT_PSBL_BILF,
	EV_POSTFILT_PSBL_FLYPF,	/*!< Check whether Flying Pixel Filter is possible | bool */
	EV_POSTFILT_MEDF,		/*!< Switch use Median Filter | bool */
	EV_POSTFILT_BILF,
	EV_POSTFILT_FLYPF,		/*!< Switch use Flying Pixel Filter | bool */
	EV_POSTFILT_PRM,		/*!< Set Post Filter Parameters | PostFilterPrm */
};

} // namespace krm
