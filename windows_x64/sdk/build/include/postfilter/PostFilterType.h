/*------------------------------------------------------------------*/
/// @file		PostFilterType.h
/// @brief		PostFilter type definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>

namespace krm
{

/*------------------------------------------------------------------*/
/// @brief	Parameters for Post filter
/*------------------------------------------------------------------*/
struct PostFilterPrm {
	uint8_t		median_ksize;	/*!< kernel size of median filter */
	uint8_t  bil_ksize;
	double   bil_sigma_depth;
	double   bil_sigma_ir;
	double   bil_sigma_space;
	uint8_t	flyp_ksize;		/*!< kernel size of flying pixel filter */
	bool		 flyp_log;		/*!< logarithmic evaluation flag of flying pixel filter */
	uint16_t flyp_thr;		/*!< threshold of flying pixel filter */
	bool		flyp_fast_proc;    /*fast processing flag of  flying pixel filter  */
};

} // namespace krm
