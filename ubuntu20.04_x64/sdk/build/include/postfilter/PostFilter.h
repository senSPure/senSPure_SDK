/*------------------------------------------------------------------*/
/// @file		PostFilter.h
/// @brief		PostFilter API class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>
#include "TpTofSdkDefine.h"
#include "PostFilterType.h"

namespace krm
{

class PostFilter
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	PostFilter(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~PostFilter(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set post filter parameters
	/// @param	[in]	filter_prm	Parameters for post filter processing
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Other, invalid argument
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result setPostFilterPrm(const PostFilterPrm& filter_prm);

	/*------------------------------------------------------------------*/
	/// @brief	Set image format
	/// @param	[in]	format	Image format
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Empty image format information
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result setFormat(const ImageFormat& format);

	/*------------------------------------------------------------------*/
	/// @brief	Run median filtering
	/// @param	[in]	org_img		Input image
	/// @param	[out]	aft_img		Image after median filter
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result filterMedian(const ImageData& org_img, ImageData& aft_img);

	/*------------------------------------------------------------------*/
/// @brief	Run bilateral  filtering 
/// @param	[in]	org_img		Input image
/// @param	[in]	is_depth	Is Depth image
/// @param	[out]	aft_img		Image after distortion correction
/// @retval	SUCCESS			Success
/// @retval	ERR_BAD_ARG		A different image format was set
/// @retval	ERR_BAD_STATE	Bad state transmission
/*------------------------------------------------------------------*/
	Result filterBilateral(const ImageData& org_img, ImageData& aft_img, bool is_depth);

	/*------------------------------------------------------------------*/
	/// @brief	Run flying pixel filtering
	/// @param	[in]	org_img		Input Depth image
	/// @param	[out]	aft_img		Depth Image after flying pixel filter
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result filterFlyingPixel(const ImageData& org_img, ImageData& aft_img);

	/* delete copy */
	PostFilter(const PostFilter&) = delete;
	PostFilter& operator=(const PostFilter&) = delete;

private :
	class PostFilterPriv;
	std::unique_ptr<PostFilterPriv> priv_; /* private class */
};

} // namespace krm
