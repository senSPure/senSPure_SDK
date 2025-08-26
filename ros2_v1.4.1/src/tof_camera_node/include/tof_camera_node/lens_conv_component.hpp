/*------------------------------------------------------------------*/
/// @file		lens_conv_component.hpp
/// @brief		LensConv component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tof_camera_interface/srv/psbl_lens_conv.hpp"
#include "tof_camera_interface/srv/set_lens_conv.hpp"
#include "tof_camera_interface/srv/set_pcd_pos.hpp"
#include "tof_camera_interface/srv/set_pcd_color.hpp"
#include "tof_camera_interface/msg/event_chg_prop.hpp"
#include "tof_camera_interface/msg/event_chg_fmt.hpp"
#include "tof_camera_interface/msg/frame_data.hpp"

#include "LensConv.h"

namespace tof_camera_node
{

class LensConvNode : public rclcpp::Node
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	options		node options
	/*------------------------------------------------------------------*/
	LensConvNode(const rclcpp::NodeOptions& options);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~LensConvNode();

private:
	enum PcdColorKind : uint8_t {
		PCD_COLOR_NONE,
		PCD_COLOR_IR,
	};

	krm::LensConv		lens_conv_;
	bool				is_dist_corrected_;
	bool				enable_distortion_;
	bool				enable_pcd_;
	bool				pcd_world_;
	krm::ImageData		tmp_dp1_;
	krm::ImageData		tmp_dp2_;
	krm::ImageData		tmp_ir_;
	krm::PcdData		tmp_pcd_;
	PcdColorKind		pcd_color_;
	krm::ImageFormat	dpt_fmt_;
	size_t				image_size_;

	std::vector<sensor_msgs::msg::PointField>	fields_;

	rclcpp::Service<tof_camera_interface::srv::PsblLensConv>::SharedPtr			srv_psbl_lens_conv_;
	rclcpp::Service<tof_camera_interface::srv::SetLensConv>::SharedPtr			srv_set_lens_conv_;
	rclcpp::Service<tof_camera_interface::srv::SetPcdPos>::SharedPtr			srv_set_pcd_pos_;
	rclcpp::Service<tof_camera_interface::srv::SetPcdColor>::SharedPtr			srv_set_pcd_color_;

	rclcpp::Subscription<tof_camera_interface::msg::EventChgProp>::SharedPtr	sub_chg_prop_;
	rclcpp::Subscription<tof_camera_interface::msg::EventChgFmt>::SharedPtr		sub_chg_fmt_;
	rclcpp::Subscription<tof_camera_interface::msg::FrameData>::SharedPtr		sub_frame_data_;

	rclcpp::Publisher<tof_camera_interface::msg::FrameData>::SharedPtr			pub_frame_data_;

	void loadParam(void);
	void createTopics(void);
	bool setLensConv(uint8_t lens_conv_type, bool enable);
	void setPcdPos(krm::PosOrgRotation& pos);

	void cbSrvPsblLensConv(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::PsblLensConv::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::PsblLensConv::Response> response);
	void cbSrvSetLensConv(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetLensConv::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetLensConv::Response> response);
	void cbSrvSetPcdPos(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPcdPos::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPcdPos::Response> response);
	void cbSrvSetPcdColor(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPcdColor::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPcdColor::Response> response);

	void recvChgProp(const tof_camera_interface::msg::EventChgProp::SharedPtr request);
	void recvChgFmt(const tof_camera_interface::msg::EventChgFmt::SharedPtr request);
	void recvFrameData(tof_camera_interface::msg::FrameData::UniquePtr frame);
};

}	// namespace tof_camera_node
