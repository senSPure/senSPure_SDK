/*------------------------------------------------------------------*/
/// @file		record_component.hpp
/// @brief		Record component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <rclcpp/rclcpp.hpp>

#include "tof_camera_interface/msg/event_chg_prop.hpp"
#include "tof_camera_interface/msg/event_chg_fmt.hpp"
#include "tof_camera_interface/msg/device_info.hpp"
#include "tof_camera_interface/msg/lens_info.hpp"
#include "tof_camera_interface/msg/cam_fov.hpp"
#include "tof_camera_interface/msg/post_filt_info.hpp"
#include "tof_camera_interface/msg/image_format.hpp"
#include "tof_camera_interface/msg/image_formats.hpp"
#include "tof_camera_interface/msg/frame_info.hpp"
#include "tof_camera_interface/msg/frame_data.hpp"
#include "tof_camera_interface/msg/frame_image.hpp"
#include "tof_camera_interface/msg/notify.hpp"
#include "tof_camera_interface/srv/record_ctrl.hpp"

#include "Record.h"

namespace tof_camera_node
{

class RecordNode : public rclcpp::Node
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	options		node options
	/*------------------------------------------------------------------*/
	RecordNode(const rclcpp::NodeOptions& options);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~RecordNode();

private:
	krm::Record::Record record_;

	rclcpp::Service<tof_camera_interface::srv::RecordCtrl>::SharedPtr			srv_record_ctrl_;

	rclcpp::Subscription<tof_camera_interface::msg::EventChgProp>::SharedPtr	sub_chg_prop_;
	rclcpp::Subscription<tof_camera_interface::msg::EventChgFmt>::SharedPtr		sub_chg_fmt_;
	rclcpp::Subscription<tof_camera_interface::msg::FrameData>::SharedPtr		sub_frame_data_;

	rclcpp::Publisher   <tof_camera_interface::msg::Notify>::SharedPtr			pub_notify_;

	krm::DeviceInfo		device_info_;
	krm::ModeInfo		mode_info_;
	krm::LensInfo		lens_info_;
	krm::CamFov			cam_fov_;
	krm::PostFiltInfo	post_filt_;
	uint8_t				mode_;
	krm::ImageFormats	image_formats_;
	krm::Frame			frame_;
	bool				recording_;

	void createTopics(void);

	bool startRecord(const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Request> param);
	bool stopRecord(void);

	void cbSrvRecordCtrl(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Response> response);

	void recvChgProp     (const tof_camera_interface::msg::EventChgProp::SharedPtr property);
	void recvChgFmt      (const tof_camera_interface::msg::EventChgFmt::SharedPtr formats);
	void recvFrameData   (const tof_camera_interface::msg::FrameData::UniquePtr frame);

	/* type conversions for publishing */
	void convertDeviceInfo  (tof_camera_interface::msg::DeviceInfo& dev_info);
	void convertModeInfo    (tof_camera_interface::msg::ModeInfo& mode_info);
	void convertLensInfo    (tof_camera_interface::msg::LensInfo& lens_info);
	void convertCamFov      (tof_camera_interface::msg::CamFov& fov);
	void convertPostFiltInfo(tof_camera_interface::msg::PostFiltInfo& info);
};

}	// namespace tof_camera_node
