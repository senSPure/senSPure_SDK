/*------------------------------------------------------------------*/
/// @file		ViewerConfig.h
/// @brief		Viewer Graphic User Interface
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <string_view>
#include <string>
#include <filesystem>
#include <climits>
#include <cfloat>
#include <boost/property_tree/ptree.hpp>
#include <rclcpp/rclcpp.hpp>

#include "TpTofSdkDefine.h"

namespace tof_camera_example
{
/*------------------------------------------------------------------*/
/// @brief	Viewer configurations
/*------------------------------------------------------------------*/
struct ViewerConfigData {
	krm::Version			file_ver;	/*!< File Version */
	struct {
		uint8_t		mode;		/*!< default motion mode */
		struct {
			std::filesystem::path	target_path;	/*!< Target directory for Record */
			uint8_t					packing;		/*!< Packing seconds in a file [sec/file] */
		} record;
	} camera;
	struct {
		std::filesystem::path		target_path;	/*!< Target directory for PlayBack */
		uint8_t						jump_time;		/*!< Jump Forward/Backward time [sec] */
	} playback;
	struct {
		struct {
			bool						enable;		/*!< Enable Median Filter */
			uint8_t						ksize;		/*!< Kernel size of Median Filter */
		} median;
		struct {
			bool						enable;		/*!< Enable Bilateral Filter */
			uint8_t						ksize;		/*!< Kernel size of Bilateral Filter */
			double						sigma_depth;/*!< Depth parameter of Bilateral Filter */
			double						sigma_ir;	/*!< IR parameter of Bilateral Filter */
			double						sigma_space;/*!< Space parameter of Bilateral Filter */
		} bil;
		struct {
			bool						enable;		/*!< Enable Flying Pixel Filter */
			uint8_t						ksize;		/*!< Kernel size of Flying Pixel Filter */
			bool						log_eval;	/*!< Logarithmic evaluation flag of Flying Pixel Filter */
			uint16_t					threshold;	/*!< Threshold of Flying Pixel Filter */
			bool						fast_proc;	/*!< Fast processing flag of Flying Pixel Filter */
		} flyp;
	} postfilter;
	struct {
		bool						distortion;		/*!< Enable Distortion Correct */
		struct {
			uint8_t					pcd_coord;		/*!< Point Cloud Coordinate type */
			std::vector<int16_t>	origin_offset;	/*!< Point Cloud Offset [X-axis, Y-axis, Z-axis] */
			std::vector<float>		rotation;		/*!< Point Cloud Angle [X-axis, Y-axis, Z-axis] */
		} point_cloud;
	} post;
	struct {
		float						gain;			/*!< Gain correction for Grayscale */
		float						gamma;			/*!< Gamma correction for Grayscale */
	} view;
};

class ViewerConfig
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	ViewerConfig(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~ViewerConfig(void);

	/*------------------------------------------------------------------*/
	/// @brief	Load configuration file
	///	@param	[in]	path	configuration file's path
	///	@param	[out]	config	Viewer configurations
	///	@retval	SUCCESS			Success
	///	@retval	ERR_NOT_EXIST	Configuration is not exist
	/*------------------------------------------------------------------*/
	krm::Result loadConfig(const std::filesystem::path& path, ViewerConfigData& config);

	/*------------------------------------------------------------------*/
	/// @brief	Save configuration file
	///	@param	[in]	path	configuration file's path
	///	@param	[in]	config	Viewer configurations
	///	@retval	SUCCESS			Success
	///	@retval	ERR_SYSTEM		Failed to save
	/*------------------------------------------------------------------*/
	krm::Result saveConfig(const std::filesystem::path& path, const ViewerConfigData& config);

private:
	static constexpr std::string_view CONFIG_FILE = "ViewerConfig.json";
	static constexpr krm::Version SUPPORT_FILE_VER = {1, 1, 0};
	static constexpr std::string_view CONFIG_CAMERA = "Camera.mode";

	ViewerConfigData default_config_;

	void readU8Val(const boost::property_tree::ptree& pt, const std::string& str,
		uint8_t& ret_val, uint8_t dflt = 0,
		uint8_t min = 0, uint8_t max = UINT8_MAX);
	void readU16Val(const boost::property_tree::ptree& pt, const std::string& str,
		uint16_t& ret_val, uint16_t dflt = 0,
		uint16_t min = 0, uint16_t max = UINT16_MAX);
	void readFloatVal(const boost::property_tree::ptree& pt, const std::string& str,
		float& ret_val, float dflt = 0.1,
		float min = -FLT_MAX, float max = FLT_MAX);
	void readDoubleVal(const boost::property_tree::ptree& pt, const std::string& str,
		double& ret_val, double dflt = 0.1,
		double min = -DBL_MAX, double max = DBL_MAX);
	void readPathVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::filesystem::path& ret_val, const std::filesystem::path& dflt = "");
	void readBoolVal(const boost::property_tree::ptree& pt, const std::string& str,
		bool& ret_val, bool dflt = true);
	void readInt16ArrVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::vector<int16_t>& ret_val, uint32_t num, int16_t dflt = 0, int16_t min = INT16_MIN, int16_t max = INT16_MAX);
	void readFloatArrVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::vector<float>& ret_val, uint32_t num, float dflt = 0, float min = -FLT_MAX, float max = FLT_MAX);
};

} // namespace tof_camera_example
