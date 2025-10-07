/*------------------------------------------------------------------*/
/// @file		std_msg_component.hpp
/// @brief		StdMsg component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "tof_camera_interface/msg/event_chg_prop.hpp"
#include "tof_camera_interface/msg/event_chg_fmt.hpp"
#include "tof_camera_interface/msg/lens_info.hpp"
#include "tof_camera_interface/msg/image_format.hpp"
#include "tof_camera_interface/msg/image_formats.hpp"
#include "tof_camera_interface/msg/frame_info.hpp"
#include "tof_camera_interface/msg/frame_data.hpp"

#include "Camera.h"

namespace tof_camera_node
{

class StdMsgNode : public rclcpp::Node
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	options		node options
	/*------------------------------------------------------------------*/
	StdMsgNode(const rclcpp::NodeOptions& options);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~StdMsgNode();

private:
	sensor_msgs::msg::CameraInfo cam_info_;

	rclcpp::Subscription<tof_camera_interface::msg::EventChgProp>::SharedPtr	sub_event_chg_prop_;
	rclcpp::Subscription<tof_camera_interface::msg::EventChgFmt>::SharedPtr		sub_event_chg_fmt_;
	rclcpp::Subscription<tof_camera_interface::msg::FrameData>::SharedPtr		sub_frame_data_;

	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr				pub_cam_info_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_depth_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_ir_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_raw1_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_raw2_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_raw3_;
	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr					pub_raw4_;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr				pub_pcd_;

	sensor_msgs::msg::PointCloud2				output_pcd_;
	uint16_t	counter_;

	void createTopics(void);
	double convFixed64Float64(uint64_t fixed_point);

	void recvEventChgProp(const tof_camera_interface::msg::EventChgProp::SharedPtr property);
	void recvEventChgFmt(const tof_camera_interface::msg::EventChgFmt::SharedPtr image_formats);
	void recvFrameData(tof_camera_interface::msg::FrameData::UniquePtr frame);
};

}	// namespace tof_camera_node
