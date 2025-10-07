/*------------------------------------------------------------------*/
/// @file		CommonLog.h
/// @brief		Log functions
/// @copyright	Copyright  (C) 2023 TOPPAN  INC.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdio>

namespace krm
{

#define LOG_INF(...)

#define LOG_WRN(...)

#define LOG_ERR(...) \
	(void)fprintf(stderr, "[ERR][%s:%d] ", __func__, __LINE__); \
	(void)fprintf(stderr, __VA_ARGS__);

} // namespace krm
