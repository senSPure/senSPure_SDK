/*------------------------------------------------------------------*/
/// @file		LensConv.h
/// @brief		LensConv API class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>
#include "CameraType.h"
#include "LensConvType.h"


namespace krm
{

class LensConv
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	LensConv(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~LensConv(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set lens parameters
	/// @param	[in]	lens_info	Parameters for lens conversion processing
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Other, invalid argument
	/*------------------------------------------------------------------*/
	Result setLensPrm(const LensInfo& lens_info , const CamFov& fov);
	/*------------------------------------------------------------------*/
	/// @brief	Set image format
	/// @param	[in]	format	Image format
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Empty image format information
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result setFormat(const ImageFormat& format);
	/*------------------------------------------------------------------*/
	/// @brief	Set origin position and rotation
	/// @param	[in]	pos_rot	Origin position, point cloud rotation information
	/*------------------------------------------------------------------*/
	void setPosOrgRotation(const PosOrgRotation& pos_rot);

	/*------------------------------------------------------------------*/
	/// @brief	Run distortion correct
	/// @param	[in]	org_img		Input image
	/// @param	[in]	is_depth	Is Depth image
	/// @param	[out]	aft_img		Image after distortion correction
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result correctDist(const ImageData& org_img, bool is_depth, ImageData& aft_img);
	/*------------------------------------------------------------------*/
	/// @brief	Run distortion correct
	/// @param	[in]	org_img		Input image
	/// @param	[in]	is_depth	Is Depth image
	/// @param	[out]	aft_img		Image after distortion correction
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result correctDist(const uint16_t* org_img, bool is_depth, uint16_t* aft_img);
	/*------------------------------------------------------------------*/
	/// @brief	Run Point Cloud conversion on camera coordinate
	/// @param	[in]	depth	Depth image
	/// @param	[out]	pcd		Point cloud data
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result convPcdCamera(const ImageData& depth, PcdData& pcd);
	/*------------------------------------------------------------------*/
	/// @brief	Run Point Cloud conversion on camera coordinate
	/// @param	[in]	depth	Depth image
	/// @param	[out]	pcd		Point cloud data
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result convPcdCamera(const uint16_t* depth, Point3d* pcd);
	/*------------------------------------------------------------------*/
	/// @brief	Run Point Cloud conversion on world coordinate
	/// @param	[in]	depth	Depth image
	/// @param	[out]	pcd		Point cloud data
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result convPcdWorld(const ImageData& depth, PcdData& pcd);
	/*------------------------------------------------------------------*/
	/// @brief	Run Point Cloud conversion on world coordinate
	/// @param	[in]	depth	Depth image
	/// @param	[out]	pcd		Point cloud data
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		A different image format was set
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/*------------------------------------------------------------------*/
	Result convPcdWorld(const uint16_t* depth, Point3d* pcd);

	/* delete copy */
	LensConv(const LensConv&) = delete;
	LensConv& operator=(const LensConv&) = delete;

private :
	class LensPriv;
	std::unique_ptr<LensPriv> priv_; /* private class */
};

} // namespace krm
