/*------------------------------------------------------------------*/
/// @file		post_filter_component.hpp
/// @brief		PostFilter component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tof_camera_interface/srv/psbl_post_filt.hpp"
#include "tof_camera_interface/srv/set_post_filt.hpp"
#include "tof_camera_interface/srv/set_post_filt_prm.hpp"
#include "tof_camera_interface/msg/event_chg_prop.hpp"
#include "tof_camera_interface/msg/event_chg_fmt.hpp"
#include "tof_camera_interface/msg/frame_data.hpp"

#include "CameraType.h"
#include "PostFilter.h"

namespace tof_camera_node
{

class PostFilterNode : public rclcpp::Node
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	options		node options
	/*------------------------------------------------------------------*/
	PostFilterNode(const rclcpp::NodeOptions& options);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~PostFilterNode();

private:
	krm::PostFilter		post_filter_;
	bool				is_filt_med_;
	bool				is_filt_bil_;
	bool				is_filt_fly_p_;
	bool				enable_medf_;
	bool				enable_bilf_;
	bool				enable_flypf_;
	krm::ImageData		dp_tmp1_;
	krm::ImageData		ir_tmp1_;
	uint32_t			image_size_;

	rclcpp::Service<tof_camera_interface::srv::PsblPostFilt>::SharedPtr			srv_psbl_post_filt_;
	rclcpp::Service<tof_camera_interface::srv::SetPostFilt>::SharedPtr			srv_set_post_filt_;
	rclcpp::Service<tof_camera_interface::srv::SetPostFiltPrm>::SharedPtr		srv_set_post_filt_prm_;

	rclcpp::Subscription<tof_camera_interface::msg::EventChgProp>::SharedPtr	sub_chg_prop_;
	rclcpp::Subscription<tof_camera_interface::msg::EventChgFmt>::SharedPtr		sub_chg_fmt_;
	rclcpp::Subscription<tof_camera_interface::msg::FrameData>::SharedPtr		sub_frame_data_;

	rclcpp::Publisher<tof_camera_interface::msg::FrameData>::SharedPtr			pub_frame_data_;

	void loadParam(void);
	void createTopics(void);
	bool setPostFilt(uint8_t filt_type, bool enable);
	bool setPostFiltPrm(krm::PostFilterPrm& prm);

	void cbSrvPsblPostFilt(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::PsblPostFilt::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::PsblPostFilt::Response> response);
	void cbSrvSetPostFilt(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPostFilt::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPostFilt::Response> response);
	void cbSrvSetPostFiltPrm(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPostFiltPrm::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPostFiltPrm::Response> response);

	void recvChgProp(const tof_camera_interface::msg::EventChgProp::SharedPtr property);
	void recvChgFmt(const tof_camera_interface::msg::EventChgFmt::SharedPtr formats);
	void recvFrameData(tof_camera_interface::msg::FrameData::UniquePtr frame);
};

}	// namespace tof_camera_node
