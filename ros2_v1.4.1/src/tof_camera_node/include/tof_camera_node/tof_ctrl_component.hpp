/*------------------------------------------------------------------*/
/// @file		tof_ctrl_component.hpp
/// @brief		TOF Control component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <mutex>
#include <filesystem>
#include <thread>
#include <string_view>
#include <rclcpp/rclcpp.hpp>

#include "tof_camera_interface/srv/get_dev_list.hpp"
#include "tof_camera_interface/srv/open_dev.hpp"
#include "tof_camera_interface/srv/close_dev.hpp"
#include "tof_camera_interface/srv/tof_ctrl.hpp"

#include "tof_camera_interface/srv/set_ext_trigger_type.hpp"
#include "tof_camera_interface/srv/set_ext_trigger_offset.hpp"
#include "tof_camera_interface/srv/set_mode.hpp"
#include "tof_camera_interface/srv/set_img_kinds.hpp"
#include "tof_camera_interface/srv/set_light_times.hpp"
#include "tof_camera_interface/srv/set_ae_state.hpp"
#include "tof_camera_interface/srv/set_ae_interval.hpp"
#include "tof_camera_interface/srv/set_raw_sat_threshold.hpp"
#include "tof_camera_interface/srv/set_ir_dark_threshold.hpp"
#include "tof_camera_interface/srv/set_int_supp.hpp"
#include "tof_camera_interface/srv/set_play_ctrl.hpp"
#include "tof_camera_interface/srv/set_play_target.hpp"

#include "tof_camera_interface/srv/get_dev_info.hpp"
#include "tof_camera_interface/srv/get_fov.hpp"
#include "tof_camera_interface/srv/get_ext_trigger_type.hpp"
#include "tof_camera_interface/srv/get_ext_trigger_offset.hpp"
#include "tof_camera_interface/srv/get_mode_list.hpp"
#include "tof_camera_interface/srv/get_mode.hpp"
#include "tof_camera_interface/srv/get_img_kinds.hpp"
#include "tof_camera_interface/srv/get_img_format.hpp"
#include "tof_camera_interface/srv/get_post_filt_info.hpp"
#include "tof_camera_interface/srv/get_lens_info.hpp"
#include "tof_camera_interface/srv/get_light_times.hpp"
#include "tof_camera_interface/srv/get_ae_state.hpp"
#include "tof_camera_interface/srv/get_ae_interval.hpp"
#include "tof_camera_interface/srv/get_raw_sat_threshold.hpp"
#include "tof_camera_interface/srv/get_ir_dark_threshold.hpp"
#include "tof_camera_interface/srv/get_int_supp_info.hpp"
#include "tof_camera_interface/srv/get_play_target.hpp"
#include "tof_camera_interface/srv/get_play_time.hpp"
#include "tof_camera_interface/srv/get_play_status.hpp"

#include "tof_camera_interface/msg/event_chg_prop.hpp"
#include "tof_camera_interface/msg/event_chg_fmt.hpp"

#include "tof_camera_interface/msg/conn_device.hpp"
#include "tof_camera_interface/msg/mode_info.hpp"
#include "tof_camera_interface/msg/lens_info.hpp"
#include "tof_camera_interface/msg/image_format.hpp"
#include "tof_camera_interface/msg/cam_fov.hpp"
#include "tof_camera_interface/msg/post_filt_info.hpp"
#include "tof_camera_interface/msg/device_info.hpp"
#include "tof_camera_interface/msg/frame_info.hpp"
#include "tof_camera_interface/msg/frame_data.hpp"
#include "tof_camera_interface/msg/notify.hpp"

#include "Camera.h"
#include "PlayBackType.h"

namespace tof_camera_node
{

#define IMAGE_TOPIC_DEPTH (5)

class TofCtrlNode : public rclcpp::Node
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	options		node options
	/*------------------------------------------------------------------*/
	TofCtrlNode(const rclcpp::NodeOptions& options);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~TofCtrlNode();

private:
	const std::string_view FRAME_ID = "krm";
	const uint8_t CAP_RETRY_MAX  = 5U;	/* Maximum of retry capture */
	const uint8_t CAP_RETRY_WAIT = 10U;	/* wait time[ms] when retry */
	const krm::Version SUPPORT_SDK_VERSION = {3, 0, 5};

	rclcpp::Service<tof_camera_interface::srv::GetDevList>::SharedPtr			srv_get_dev_list_;
	rclcpp::Service<tof_camera_interface::srv::OpenDev>::SharedPtr				srv_open_dev_;
	rclcpp::Service<tof_camera_interface::srv::CloseDev>::SharedPtr				srv_close_dev_;
	rclcpp::Service<tof_camera_interface::srv::TofCtrl>::SharedPtr				srv_tof_ctrl_;

	rclcpp::Service<tof_camera_interface::srv::SetExtTriggerType>::SharedPtr	srv_set_ext_trigger_type_;
	rclcpp::Service<tof_camera_interface::srv::SetExtTriggerOffset>::SharedPtr	srv_set_ext_trigger_offset_;
	rclcpp::Service<tof_camera_interface::srv::SetMode>::SharedPtr				srv_set_mode_;
	rclcpp::Service<tof_camera_interface::srv::SetImgKinds>::SharedPtr			srv_set_img_kinds_;
	rclcpp::Service<tof_camera_interface::srv::SetLightTimes>::SharedPtr		srv_set_light_times_;
	rclcpp::Service<tof_camera_interface::srv::SetAEState>::SharedPtr			srv_set_ae_state_;
	rclcpp::Service<tof_camera_interface::srv::SetAEInterval>::SharedPtr		srv_set_ae_interval_;
	rclcpp::Service<tof_camera_interface::srv::SetRawSatThreshold>::SharedPtr	srv_set_raw_sat_threshold_;
	rclcpp::Service<tof_camera_interface::srv::SetIrDarkThreshold>::SharedPtr	srv_set_ir_dark_threshold_;
	rclcpp::Service<tof_camera_interface::srv::SetIntSupp>::SharedPtr			srv_set_int_supp_;
	rclcpp::Service<tof_camera_interface::srv::SetPlayTarget>::SharedPtr		srv_set_play_target_;
	rclcpp::Service<tof_camera_interface::srv::SetPlayCtrl>::SharedPtr			srv_set_play_ctrl_;

	rclcpp::Service<tof_camera_interface::srv::GetDevInfo>::SharedPtr			srv_get_dev_info_;
	rclcpp::Service<tof_camera_interface::srv::GetFov>::SharedPtr				srv_get_fov_;
	rclcpp::Service<tof_camera_interface::srv::GetExtTriggerType>::SharedPtr	srv_get_ext_trigger_type_;
	rclcpp::Service<tof_camera_interface::srv::GetExtTriggerOffset>::SharedPtr	srv_get_ext_trigger_offset_;
	rclcpp::Service<tof_camera_interface::srv::GetModeList>::SharedPtr			srv_get_mode_list_;
	rclcpp::Service<tof_camera_interface::srv::GetMode>::SharedPtr				srv_get_mode_;
	rclcpp::Service<tof_camera_interface::srv::GetImgKinds>::SharedPtr			srv_get_img_kinds_;
	rclcpp::Service<tof_camera_interface::srv::GetImgFormat>::SharedPtr			srv_get_img_format_;
	rclcpp::Service<tof_camera_interface::srv::GetPostFiltInfo>::SharedPtr		srv_get_post_filt_info_;
	rclcpp::Service<tof_camera_interface::srv::GetLensInfo>::SharedPtr			srv_get_lens_info_;
	rclcpp::Service<tof_camera_interface::srv::GetLightTimes>::SharedPtr		srv_get_light_times_;
	rclcpp::Service<tof_camera_interface::srv::GetAEState>::SharedPtr			srv_get_ae_state_;
	rclcpp::Service<tof_camera_interface::srv::GetAEInterval>::SharedPtr		srv_get_ae_interval_;
	rclcpp::Service<tof_camera_interface::srv::GetRawSatThreshold>::SharedPtr	srv_get_raw_sat_threshold_;
	rclcpp::Service<tof_camera_interface::srv::GetIrDarkThreshold>::SharedPtr	srv_get_ir_dark_threshold_;
	rclcpp::Service<tof_camera_interface::srv::GetIntSuppInfo>::SharedPtr		srv_get_int_supp_info_;
	rclcpp::Service<tof_camera_interface::srv::GetPlayTarget>::SharedPtr		srv_get_play_target_;
	rclcpp::Service<tof_camera_interface::srv::GetPlayTime>::SharedPtr			srv_get_play_time_;
	rclcpp::Service<tof_camera_interface::srv::GetPlayStatus>::SharedPtr		srv_get_play_status_;

	rclcpp::Publisher<tof_camera_interface::msg::EventChgProp>::SharedPtr	pub_event_chg_prop_;
	rclcpp::Publisher<tof_camera_interface::msg::EventChgFmt>::SharedPtr	pub_event_chg_fmt_;

	rclcpp::Publisher<tof_camera_interface::msg::FrameData>::SharedPtr		pub_frame_data_;
	rclcpp::Publisher<tof_camera_interface::msg::Notify>::SharedPtr			pub_notify_;

	std::unique_ptr<krm::Camera>	camera_;
	krm::CameraType					cam_type_;
	uint16_t						cam_index_;
	uint8_t							cam_mode_;
	uint8_t							img_out_kind_;
	std::filesystem::path			play_dir_;
	bool							auto_start_;
	bool							auto_open_;
	std::recursive_mutex			th_mtx_;
	std::unique_ptr<std::thread>	cap_thread_;

	std::string						frame_id_;
	krm::ModeList					mode_list_;
	krm::ImageFormats				img_fmts_;
	uint16_t						cur_fps_;
	uint8_t							img_index_;

	bool							first_frame_;

	void createTopics(void);
	bool loadParam(void);
	bool initCamera(void);
	bool openCamera(uint16_t id, krm::CameraType type);
	bool closeCamera(void);
	bool checkVersion(void);

	bool notifyProperty(bool notify_nodes = true);
	bool notifyImgFmt(void);
	void resetFpsCount(void);
	uint32_t getFpsCalcCycle(void);
	void calcFps(const struct timespec& pre_time,
		const struct timespec& cur_time,
		uint32_t cycle, uint64_t& total_time,
		uint32_t& frame_cnt, uint16_t& rcv_fps
	);

	bool changeMode(uint8_t mode);

	bool startCapture(void);
	bool stopCapture(void);
	bool setPlayTime(uint32_t time);
	bool setPlayPause(void);
	bool setPlayFast(void);
	bool setPlaySlow(void);
	bool setPlayJumpFw(uint32_t time);
	bool setPlayJumpBw(uint32_t time);
	bool getPlayTime(krm::PlayBack::PlayTime& time);
	bool getPlayStatus(krm::PlayBack::PlayStatus& status);

	void convPlayTime(tof_camera_interface::msg::PlayTime& stamp);
	void convImage(const krm::ImageData* src, const krm::ImageFormat* fmt, sensor_msgs::msg::Image* dst);
	void pubFrameData(const krm::Frame& frame, uint16_t rcv_fps);

	/* callbacks */
	void cbSrvGetDevList(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetDevList::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetDevList::Response> response);
	void cbSrvOpenDev(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::OpenDev::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::OpenDev::Response> response);
	void cbSrvCloseDev(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::CloseDev::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::CloseDev::Response> response);
	void cbSrvTofCtrl(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::TofCtrl::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::TofCtrl::Response> response);


	void cbSrvSetExtTriggerType(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerType::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerType::Response> response);
	void cbSrvSetExtTriggerOffset(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerOffset::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerOffset::Response> response);
	void cbSrvSetMode(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetMode::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetMode::Response> response);
	void cbSrvSetImgKinds(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetImgKinds::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetImgKinds::Response> response);
	void cbSrvSetLightTimes(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetLightTimes::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetLightTimes::Response> response);
	void cbSrvSetAEState(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetAEState::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetAEState::Response> response);
	void cbSrvSetAEInterval(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetAEInterval::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetAEInterval::Response> response);
	void cbSrvSetRawSatThreshold(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetRawSatThreshold::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetRawSatThreshold::Response> response);
	void cbSrvSetIrDarkThreshold(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetIrDarkThreshold::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetIrDarkThreshold::Response> response);
	void cbSrvSetIntSupp(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetIntSupp::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetIntSupp::Response> response);
	void cbSrvSetPlayTarget(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPlayTarget::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPlayTarget::Response> response);
	void cbSrvSetPlayCtrl(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::SetPlayCtrl::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::SetPlayCtrl::Response> response);

	void cbSrvGetDevInfo(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetDevInfo::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetDevInfo::Response> response);
	void cbSrvGetFov(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetFov::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetFov::Response> response);
	void cbSrvGetExtTriggerType(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerType::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerType::Response> response);
	void cbSrvGetExtTriggerOffset(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerOffset::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerOffset::Response> response);
	void cbSrvGetModeList(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetModeList::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetModeList::Response> response);
	void cbSrvGetMode(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetMode::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetMode::Response> response);
	void cbSrvGetImgKinds(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetImgKinds::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetImgKinds::Response> response);
	void cbSrvGetImgFormat(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetImgFormat::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetImgFormat::Response> response);
	void cbSrvGetPostFiltInfo(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetPostFiltInfo::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetPostFiltInfo::Response> response);
	void cbSrvGetLensInfo(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetLensInfo::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetLensInfo::Response> response);
	void cbSrvGetLightTimes(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetLightTimes::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetLightTimes::Response> response);
	void cbSrvGetAEState(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetAEState::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetAEState::Response> response);
	void cbSrvGetAEInterval(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetAEInterval::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetAEInterval::Response> response);
	void cbSrvGetRawSatThreshold(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetRawSatThreshold::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetRawSatThreshold::Response> response);
	void cbSrvGetIrDarkThreshold(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetIrDarkThreshold::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetIrDarkThreshold::Response> response);
	void cbSrvGetIntSuppInfo(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetIntSuppInfo::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetIntSuppInfo::Response> response);
	void cbSrvGetPlayTarget(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayTarget::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayTarget::Response> response);
	void cbSrvGetPlayTime(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayTime::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayTime::Response> response);
	void cbSrvGetPlayStatus(
		const std::shared_ptr<rmw_request_id_t> request_header,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayStatus::Request> request,
		const std::shared_ptr<tof_camera_interface::srv::GetPlayStatus::Response> response);

	void captureThread(void);
	bool retryCapture(uint8_t& retry);

	/* type conversions for publishing */
	void convertDeviceInfo(krm::DeviceInfo& from, tof_camera_interface::msg::DeviceInfo& to);
	void convertLensInfo(krm::LensInfo& from, tof_camera_interface::msg::LensInfo& to);
	void convertCamFov(krm::CamFov& from, tof_camera_interface::msg::CamFov& to);
	void convertPostFiltInfo(krm::PostFiltInfo& from, tof_camera_interface::msg::PostFiltInfo& to);
	void convertModeInfo(krm::ModeInfo& from, tof_camera_interface::msg::ModeInfo& to);
	void convertImageFormat(krm::ImageFormat& from, tof_camera_interface::msg::ImageFormat& to);
};

}	// namespace tof_camera_node
