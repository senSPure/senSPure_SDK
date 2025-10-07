/*------------------------------------------------------------------*/
/// @file		ViewerConfig.cpp
/// @brief		Viewer configuration
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <algorithm>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/optional.hpp>

#include "ViewerConfig.h"

namespace tof_camera_example
{

ViewerConfig::ViewerConfig(void)
{
	default_config_.camera.mode = 0;
	default_config_.camera.record.target_path = "";
	default_config_.camera.record.packing = 60U;
	default_config_.playback.target_path = "";
	default_config_.playback.jump_time = 10U;
	default_config_.post.distortion = true;
	default_config_.post.point_cloud.pcd_coord = 0;
	default_config_.post.point_cloud.origin_offset = {0,0,0};
	default_config_.post.point_cloud.rotation = {0.0F, 0.0F, 0.0F};
	default_config_.view.gain = 20.0F;
	default_config_.view.gamma = 2.0F;
	default_config_.postfilter.median.enable = false;
	default_config_.postfilter.median.ksize = 3U;
	default_config_.postfilter.bil.enable = true;
	default_config_.postfilter.bil.ksize = 3U;
	default_config_.postfilter.bil.sigma_depth = 500.0;
	default_config_.postfilter.bil.sigma_ir = 100.0;
	default_config_.postfilter.bil.sigma_space = 1.0;
	default_config_.postfilter.flyp.enable = true;
	default_config_.postfilter.flyp.ksize = 3U;
	default_config_.postfilter.flyp.log_eval = true;
	default_config_.postfilter.flyp.threshold = 130U;
	default_config_.postfilter.flyp.fast_proc = true;
}

ViewerConfig::~ViewerConfig(void)
{

}

krm::Result ViewerConfig::loadConfig(const std::filesystem::path& path, ViewerConfigData& config)
{
	boost::property_tree::ptree pt;
	std::filesystem::path str = path;
	str.append(CONFIG_FILE);

	try {
		boost::property_tree::read_json(str.string().data(), pt);
	} catch (...) {
		// LOG_WRN("%s is not exist\n", CONFIG_FILE.data());
		config = default_config_;
		return krm::ERR_NOT_EXIST;
	}

	readU8Val(pt, "Version.major", config.file_ver.major);
	readU8Val(pt, "Version.minor", config.file_ver.minor);
	readU16Val(pt, "Version.revision", config.file_ver.rev);
	if ((config.file_ver.major != SUPPORT_FILE_VER.major) &&
		(config.file_ver.minor != SUPPORT_FILE_VER.minor) &&
		(config.file_ver.rev != SUPPORT_FILE_VER.rev)) {
		// RCLCPP_WARN(this->get_logger(), "This file version is not supported (%u.%u.%u)\n",
		//  	config.file_ver.major, config.file_ver.minor, config.file_ver.rev);
		config = default_config_;
		return krm::ERR_NOT_EXIST;
	}

	readU8Val(pt, "Camera.mode", config.camera.mode, default_config_.camera.mode);
	readPathVal(pt, "Camera.Record.target_path", config.camera.record.target_path, default_config_.camera.record.target_path);
	readU8Val(pt, "Camera.Record.packing", config.camera.record.packing, default_config_.camera.record.packing, 1U);

	readPathVal(pt, "PlayBack.target_path", config.playback.target_path, default_config_.playback.target_path);
	readU8Val(pt, "PlayBack.jump_time", config.playback.jump_time, default_config_.playback.jump_time, 1U);

	readBoolVal(pt, "PostProcess.distortion", config.post.distortion, default_config_.post.distortion);
	readU8Val(pt, "PostProcess.PointCloud.pcd_coord", config.post.point_cloud.pcd_coord, default_config_.post.point_cloud.pcd_coord, 0, 1U);

	readInt16ArrVal(pt, "PostProcess.PointCloud.offset", config.post.point_cloud.origin_offset, 3U, default_config_.post.point_cloud.origin_offset[0]);
	readFloatArrVal(pt, "PostProcess.PointCloud.rotation", config.post.point_cloud.rotation, 3U, default_config_.post.point_cloud.rotation[0], -180.0F, 180.0F);

	readBoolVal(pt, "PostFilter.Median.enable", config.postfilter.median.enable, default_config_.postfilter.median.enable);
	readU8Val(pt, "PostFilter.Median.ksize", config.postfilter.median.ksize, default_config_.postfilter.median.ksize);
	readBoolVal(pt, "PostFilter.Bilateral.enable", config.postfilter.bil.enable, default_config_.postfilter.bil.enable);
	readU8Val(pt, "PostFilter.Bilateral.ksize", config.postfilter.bil.ksize, default_config_.postfilter.bil.ksize);
	readDoubleVal(pt, "PostFilter.Bilateral.sigma_depth", config.postfilter.bil.sigma_depth, default_config_.postfilter.bil.sigma_depth);
	readDoubleVal(pt, "PostFilter.Bilateral.sigma_ir", config.postfilter.bil.sigma_ir, default_config_.postfilter.bil.sigma_ir);
	readDoubleVal(pt, "PostFilter.Bilateral.sigma_space", config.postfilter.bil.sigma_space, default_config_.postfilter.bil.sigma_space);
	readBoolVal(pt, "PostFilter.FlyingPixel.enable", config.postfilter.flyp.enable, default_config_.postfilter.flyp.enable);
	readU8Val(pt, "PostFilter.FlyingPixel.ksize", config.postfilter.flyp.ksize, default_config_.postfilter.flyp.ksize);
	readBoolVal(pt, "PostFilter.FlyingPixel.log_eval", config.postfilter.flyp.log_eval, default_config_.postfilter.flyp.log_eval);
	readU16Val(pt, "PostFilter.FlyingPixel.threshold", config.postfilter.flyp.threshold, default_config_.postfilter.flyp.threshold);
	readBoolVal(pt, "PostFilter.FlyingPixel.fast_proc", config.postfilter.flyp.fast_proc, default_config_.postfilter.flyp.fast_proc);

	readFloatVal(pt, "ViewSettings.Gain", config.view.gain, default_config_.view.gain, 0.1F, 50.0F);
	readFloatVal(pt, "ViewSettings.Gamma", config.view.gamma, default_config_.view.gamma, 0.1F, 5.0F);

	return krm::SUCCESS;
}

krm::Result ViewerConfig::saveConfig(const std::filesystem::path& path, const ViewerConfigData& config)
{
	boost::property_tree::ptree pt;
	boost::property_tree::ptree rec, median, bil, flyp;
	boost::property_tree::ptree pt_child_d;
	boost::property_tree::ptree offset_children;
	boost::property_tree::ptree offset_child1, offset_child2, offset_child3;
	boost::property_tree::ptree rotation_children;
	boost::property_tree::ptree rotation_child1, rotation_child2, rotation_child3;
	std::filesystem::path str = path;
	str.append(CONFIG_FILE);

	pt.put("Version.major", SUPPORT_FILE_VER.major);
	pt.put("Version.minor", SUPPORT_FILE_VER.minor);
	pt.put("Version.revision", SUPPORT_FILE_VER.rev);

	pt.put("Camera.mode", config.camera.mode);
	rec.put("target_path", config.camera.record.target_path.generic_u8string().c_str());
	rec.put("packing", config.camera.record.packing);
	pt.add_child("Camera.Record", rec);

	pt.put("PlayBack.target_path", config.playback.target_path.generic_u8string().c_str());
	pt.put("PlayBack.jump_time", config.playback.jump_time);

	pt.put("PostProcess.distortion", config.post.distortion);

	pt_child_d.put("pcd_coord", config.post.point_cloud.pcd_coord);
	offset_child1.put("", config.post.point_cloud.origin_offset[0]);
	offset_child2.put("", config.post.point_cloud.origin_offset[1]);
	offset_child3.put("", config.post.point_cloud.origin_offset[2]);
	offset_children.push_back(std::make_pair("", offset_child1));
	offset_children.push_back(std::make_pair("", offset_child2));
	offset_children.push_back(std::make_pair("", offset_child3));
	pt_child_d.add_child("offset", offset_children);
	rotation_child1.put("", config.post.point_cloud.rotation[0]);
	rotation_child2.put("", config.post.point_cloud.rotation[1]);
	rotation_child3.put("", config.post.point_cloud.rotation[2]);
	rotation_children.push_back(std::make_pair("", rotation_child1));
	rotation_children.push_back(std::make_pair("", rotation_child2));
	rotation_children.push_back(std::make_pair("", rotation_child3));
	pt_child_d.add_child("rotation", rotation_children);
	pt.add_child("PostProcess.PointCloud", pt_child_d);

	median.put("enable", config.postfilter.median.enable);
	median.put("ksize", config.postfilter.median.ksize);
	pt.add_child("PostFilter.Median", median);
	bil.put("enable", config.postfilter.bil.enable);
	bil.put("ksize", config.postfilter.bil.ksize);
	bil.put("sigma_depth", config.postfilter.bil.sigma_depth);
	bil.put("sigma_ir", config.postfilter.bil.sigma_ir);
	bil.put("sigma_space", config.postfilter.bil.sigma_space);
	pt.add_child("PostFilter.Bilateral", bil);
	flyp.put("enable", config.postfilter.flyp.enable);
	flyp.put("ksize", config.postfilter.flyp.ksize);
	flyp.put("log_eval", config.postfilter.flyp.log_eval);
	flyp.put("threshold", config.postfilter.flyp.threshold);
	flyp.put("fast_proc", config.postfilter.flyp.fast_proc);
	pt.add_child("PostFilter.FlyingPixel", flyp);

	pt.put("ViewSettings.Gain", config.view.gain);
	pt.put("ViewSettings.Gamma", config.view.gamma);

	try {
		boost::property_tree::write_json(str.string().data(), pt);
	} catch (...) {
		// RCLCPP_WARN(this->get_logger(), "failed to save %s\n", CONFIG_FILE.data());
		return krm::ERR_SYSTEM;
	}

	return krm::SUCCESS;
}

void ViewerConfig::readU8Val(const boost::property_tree::ptree& pt, const std::string& str,
	uint8_t& ret_val, uint8_t dflt, uint8_t min, uint8_t max)
{
	boost::optional<int> pt_value;
	int value;

	pt_value = pt.get_optional<int>(str.c_str());
	if (pt_value) {
		value = pt_value.get();
		if ((value >= static_cast<int>(min)) && (value <= static_cast<int>(max))) {
			ret_val = static_cast<uint8_t>(value);
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%u-%u) : %d\n", str.c_str(), min, max, value);
			ret_val = dflt;
		}
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %u\n", str.c_str(), ret_val);
}

void ViewerConfig::readU16Val(const boost::property_tree::ptree& pt, const std::string& str,
	uint16_t& ret_val, uint16_t dflt, uint16_t min, uint16_t max)
{
	boost::optional<int> pt_value;
	int value;

	pt_value = pt.get_optional<int>(str.c_str());
	if (pt_value) {
		value = pt_value.get();
		if ((value >= static_cast<int>(min)) && (value <= static_cast<int>(max))) {
			ret_val = static_cast<uint16_t>(value);
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%u-%u) : %d\n", str.c_str(), min, max, value);
			ret_val = dflt;
		}
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %u\n", str.c_str(), ret_val);
}

void ViewerConfig::readFloatVal(const boost::property_tree::ptree& pt, const std::string& str,
		float& ret_val, float dflt, float min, float max)
{
	boost::optional<float> pt_value;
	float value;

	pt_value = pt.get_optional<float>(str.c_str());
	if (pt_value) {
		value = pt_value.get();
		if ((value >= min) && (value <= max)) {
			ret_val = static_cast<float>(value);
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%f-%f) : %f\n", str.c_str(), min, max, value);
			ret_val = dflt;
		}
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %f\n", str.c_str(), ret_val);
}

void ViewerConfig::readDoubleVal(const boost::property_tree::ptree& pt, const std::string& str,
	double& ret_val, double dflt, double min, double max)
{
	boost::optional<double> pt_value;
	double value;

	pt_value = pt.get_optional<double>(str.c_str());
	if (pt_value) {
		value = pt_value.get();
		if ((value >= min) && (value <= max)) {
			ret_val = value;
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%f-%f) : %f\n", str.c_str(), min, max, value);
			ret_val = dflt;
		}
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %f\n", str.c_str(), ret_val);
}

void ViewerConfig::readPathVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::filesystem::path& ret_val, const std::filesystem::path& dflt)
{
	boost::optional<std::string> pt_value;
	std::string value;

	pt_value = pt.get_optional<std::string>(str.c_str());

	if (pt_value) {
#ifdef LNX_FUNC
		value = pt_value.get();
		ret_val = value.c_str();
#else	/* LNX_FUNC */
		try {
			value = pt.get<std::string>(str.c_str());
		} catch (...) {
		}
		ret_val = std::filesystem::u8path(value.c_str());
#endif	/* LNX_FUNC */
		ret_val.make_preferred();
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %s\n", str.c_str(), ret_val.string<char>().c_str());
}

void ViewerConfig::readBoolVal(const boost::property_tree::ptree& pt, const std::string& str,
		bool& ret_val, bool dflt)
{
	boost::optional<bool> pt_value;
	bool value;

	pt_value = pt.get_optional<bool>(str.c_str());
	if (pt_value) {
		value = pt_value.get();
		ret_val = static_cast<bool>(value);
	} else {
		// RCLCPP_WARN(this->get_logger(), "%s is not exist\n", str.c_str());
		ret_val = dflt;
	}
	// RCLCPP_INFO(this->get_logger(), "%s = %s\n", str.c_str(), ret_val ? "true" : "false");
}


void ViewerConfig::readInt16ArrVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::vector<int16_t>& ret_val, uint32_t num, int16_t dflt, int16_t min, int16_t max)
{
	uint32_t i = 0;
	int value;
	boost::property_tree::ptree	ary = pt.get_child(str.c_str());

	ret_val.resize(num);
	for (auto it = ary.begin(); it != ary.end(); ++it) {
		try {
			value = std::stoi(it->second.data());
		} catch (...) {
			// RCLCPP_WARN(this->get_logger(), "%s : value is invalid string\n", str.c_str());
			value = static_cast<int>(dflt);
		}
		if ((value >= static_cast<int>(min)) && (value <= static_cast<int>(max))) {
			ret_val[i] = static_cast<int16_t>(value);
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%u-%u) : %d\n", str.c_str(), min, max, value);
			ret_val[i] = dflt;
		}
		// RCLCPP_INFO(this->get_logger(), "%s[%u] = %d\n", str.c_str(), i, ret_val[i]);
		i++;
	}
	if (i < num) {
		// RCLCPP_WARN(this->get_logger(), "%s is not enough (%u/%u)\n", str.c_str(), i, num);
		std::fill(ret_val.begin(), ret_val.end(), dflt);
	}
}

void ViewerConfig::readFloatArrVal(const boost::property_tree::ptree& pt, const std::string& str,
		std::vector<float>& ret_val, uint32_t num, float dflt, float min, float max)
{
	uint32_t i = 0;
	float value;
	boost::property_tree::ptree	ary = pt.get_child(str.c_str());

	ret_val.resize(num);
	for (auto it = ary.begin(); it != ary.end(); ++it) {
		try {
			value = std::stof(it->second.data());
		} catch (...) {
			// RCLCPP_WARN(this->get_logger(), "%s : value is invalid string\n", str.c_str());
			value = dflt;
		}
		if ((value >= min) && (value <= max)) {
			ret_val[i] = value;
		} else {
			// RCLCPP_WARN(this->get_logger(), "%s is over the range(%f-%f) : %f\n", str.c_str(), min, max, value);
			ret_val[i] = dflt;
		}
		// RCLCPP_INFO(this->get_logger(), "%s[%u] = %f\n", str.c_str(), i, ret_val[i]);
		i++;
	}
	if (i < num) {
		// RCLCPP_WARN(this->get_logger(), "%s is not enough (%u/%u)\n", str.c_str(), i, num);
		std::fill(ret_val.begin(), ret_val.end(), dflt);
	}
}

} // namespace tof_camera_example
