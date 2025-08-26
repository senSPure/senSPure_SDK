/*------------------------------------------------------------------*/
/// @file		tof_ctrl_component.cpp
/// @brief		TOF Control component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cstring>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "tof_ctrl_component.hpp"

#include "Record.h"

namespace tof_camera_node
{

TofCtrlNode::TofCtrlNode(
	const rclcpp::NodeOptions& options
): Node("TofCtrlNode", options),
	camera_(nullptr),
	cam_type_(krm::C11_USB), cam_index_(0),
	cam_mode_(0), img_out_kind_(0),
	play_dir_(""), auto_start_(true), auto_open_(true), cap_thread_(nullptr),
	frame_id_(""),
	cur_fps_(1U),
	first_frame_(true)
{
	bool result = false;

	RCLCPP_INFO(this->get_logger(), "run TofCtrlNode");
	mode_list_.clear();
	for (auto& fmt : img_fmts_) {
		fmt.set();
	}

	createTopics();
	if (loadParam()) {
		if (checkVersion()) {
			if (!auto_open_ || initCamera()) {
				result = true;
			}
		}
	}

	if (result) {
		if (auto_open_ && auto_start_) {
			result = startCapture();
		}
	}
	if (result) {
		RCLCPP_INFO(this->get_logger(), "wakeup TofCtrlNode");
	} else {
		tof_camera_interface::msg::Notify notify;
		notify.notify = tof_camera_interface::msg::Notify::ERR_PARAM;
		pub_notify_->publish(notify);
		(void)closeCamera();
		if (camera_ != nullptr) { camera_.reset(nullptr); }
		RCLCPP_ERROR(this->get_logger(), "Failed to wakeup. Please shutdown this Node");
	}
}

TofCtrlNode::~TofCtrlNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit TofCtrlNode");
	if (camera_ && cap_thread_) { (void)stopCapture(); }
	(void)closeCamera();
	if (camera_ != nullptr) { camera_.reset(nullptr); }
}

void TofCtrlNode::createTopics(void)
{
	rclcpp::QoS qos_status(rclcpp::KeepLast(1));
	rclcpp::QoS qos_image(rclcpp::KeepLast(IMAGE_TOPIC_DEPTH));
	rclcpp::QoS qos_event(rclcpp::KeepLast(5));

	auto cb_list = std::bind(&TofCtrlNode::cbSrvGetDevList         , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_open = std::bind(&TofCtrlNode::cbSrvOpenDev            , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_clos = std::bind(&TofCtrlNode::cbSrvCloseDev           , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_ctrl = std::bind(&TofCtrlNode::cbSrvTofCtrl            , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	auto cb_s_ttyp = std::bind(&TofCtrlNode::cbSrvSetExtTriggerType  , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_toff = std::bind(&TofCtrlNode::cbSrvSetExtTriggerOffset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_mode = std::bind(&TofCtrlNode::cbSrvSetMode            , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_kind = std::bind(&TofCtrlNode::cbSrvSetImgKinds        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_ltim = std::bind(&TofCtrlNode::cbSrvSetLightTimes      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_aest = std::bind(&TofCtrlNode::cbSrvSetAEState         , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_aeit = std::bind(&TofCtrlNode::cbSrvSetAEInterval      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_rsth = std::bind(&TofCtrlNode::cbSrvSetRawSatThreshold , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_irth = std::bind(&TofCtrlNode::cbSrvSetIrDarkThreshold , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_insp = std::bind(&TofCtrlNode::cbSrvSetIntSupp         , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_ptgt = std::bind(&TofCtrlNode::cbSrvSetPlayTarget      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_s_play = std::bind(&TofCtrlNode::cbSrvSetPlayCtrl        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	auto cb_g_devi = std::bind(&TofCtrlNode::cbSrvGetDevInfo         , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_dfov = std::bind(&TofCtrlNode::cbSrvGetFov             , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_ttyp = std::bind(&TofCtrlNode::cbSrvGetExtTriggerType  , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_toff = std::bind(&TofCtrlNode::cbSrvGetExtTriggerOffset, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_mlst = std::bind(&TofCtrlNode::cbSrvGetModeList        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_mode = std::bind(&TofCtrlNode::cbSrvGetMode            , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_kind = std::bind(&TofCtrlNode::cbSrvGetImgKinds        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_ifmt = std::bind(&TofCtrlNode::cbSrvGetImgFormat       , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_pinf = std::bind(&TofCtrlNode::cbSrvGetPostFiltInfo    , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_linf = std::bind(&TofCtrlNode::cbSrvGetLensInfo        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_ltim = std::bind(&TofCtrlNode::cbSrvGetLightTimes      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_aest = std::bind(&TofCtrlNode::cbSrvGetAEState         , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_aeit = std::bind(&TofCtrlNode::cbSrvGetAEInterval      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_rsth = std::bind(&TofCtrlNode::cbSrvGetRawSatThreshold , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_irth = std::bind(&TofCtrlNode::cbSrvGetIrDarkThreshold , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_insp = std::bind(&TofCtrlNode::cbSrvGetIntSuppInfo     , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_ptgt = std::bind(&TofCtrlNode::cbSrvGetPlayTarget      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_time = std::bind(&TofCtrlNode::cbSrvGetPlayTime        , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_g_pstt = std::bind(&TofCtrlNode::cbSrvGetPlayStatus      , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

	qos_status = qos_status.reliable().durability_volatile();
	qos_image = qos_image.reliable().durability_volatile();
	qos_event = qos_event.reliable().durability_volatile();

	srv_get_dev_list_           = create_service<tof_camera_interface::srv::GetDevList         >("krm/get_dev_list"          , cb_list);
	srv_open_dev_               = create_service<tof_camera_interface::srv::OpenDev            >("krm/open_dev"              , cb_open);
	srv_close_dev_              = create_service<tof_camera_interface::srv::CloseDev           >("krm/close_dev"             , cb_clos);
	srv_tof_ctrl_               = create_service<tof_camera_interface::srv::TofCtrl            >("krm/tof_ctrl"              , cb_ctrl);

	srv_set_ext_trigger_type_   = create_service<tof_camera_interface::srv::SetExtTriggerType  >("krm/set_ext_trigger_type"  , cb_s_ttyp);
	srv_set_ext_trigger_offset_ = create_service<tof_camera_interface::srv::SetExtTriggerOffset>("krm/set_ext_trigger_offset", cb_s_toff);
	srv_set_mode_               = create_service<tof_camera_interface::srv::SetMode            >("krm/set_mode"              , cb_s_mode);
	srv_set_img_kinds_          = create_service<tof_camera_interface::srv::SetImgKinds        >("krm/set_img_kinds"         , cb_s_kind);
	srv_set_light_times_        = create_service<tof_camera_interface::srv::SetLightTimes      >("krm/set_light_times"       , cb_s_ltim);
	srv_set_ae_state_           = create_service<tof_camera_interface::srv::SetAEState         >("krm/set_ae_state"          , cb_s_aest);
	srv_set_ae_interval_        = create_service<tof_camera_interface::srv::SetAEInterval      >("krm/set_ae_interval"       , cb_s_aeit);
	srv_set_raw_sat_threshold_  = create_service<tof_camera_interface::srv::SetRawSatThreshold >("krm/set_raw_sat_threshold" , cb_s_rsth);
	srv_set_ir_dark_threshold_  = create_service<tof_camera_interface::srv::SetIrDarkThreshold >("krm/set_ir_dark_threshold" , cb_s_irth);
	srv_set_int_supp_           = create_service<tof_camera_interface::srv::SetIntSupp         >("krm/set_int_supp"          , cb_s_insp);
	srv_set_play_target_        = create_service<tof_camera_interface::srv::SetPlayTarget      >("krm/set_play_target"       , cb_s_ptgt);
	srv_set_play_ctrl_          = create_service<tof_camera_interface::srv::SetPlayCtrl        >("krm/set_play_ctrl"         , cb_s_play);

	srv_get_dev_info_           = create_service<tof_camera_interface::srv::GetDevInfo         >("krm/get_dev_info"          , cb_g_devi);
	srv_get_fov_                = create_service<tof_camera_interface::srv::GetFov             >("krm/get_fov"               , cb_g_dfov);
	srv_get_ext_trigger_type_   = create_service<tof_camera_interface::srv::GetExtTriggerType  >("krm/get_ext_trigger_type"  , cb_g_ttyp);
	srv_get_ext_trigger_offset_ = create_service<tof_camera_interface::srv::GetExtTriggerOffset>("krm/get_ext_trigger_offset", cb_g_toff);
	srv_get_mode_list_          = create_service<tof_camera_interface::srv::GetModeList        >("krm/get_mode_list"         , cb_g_mlst);
	srv_get_mode_               = create_service<tof_camera_interface::srv::GetMode            >("krm/get_mode"              , cb_g_mode);
	srv_get_img_kinds_          = create_service<tof_camera_interface::srv::GetImgKinds        >("krm/get_img_kinds"         , cb_g_kind);
	srv_get_img_format_         = create_service<tof_camera_interface::srv::GetImgFormat       >("krm/get_img_format"        , cb_g_ifmt);
	srv_get_post_filt_info_     = create_service<tof_camera_interface::srv::GetPostFiltInfo    >("krm/get_post_filt_info"    , cb_g_pinf);
	srv_get_lens_info_          = create_service<tof_camera_interface::srv::GetLensInfo        >("krm/get_lens_info"         , cb_g_linf);
	srv_get_light_times_        = create_service<tof_camera_interface::srv::GetLightTimes      >("krm/get_light_times"       , cb_g_ltim);
	srv_get_ae_state_           = create_service<tof_camera_interface::srv::GetAEState         >("krm/get_ae_state"          , cb_g_aest);
	srv_get_ae_interval_        = create_service<tof_camera_interface::srv::GetAEInterval      >("krm/get_ae_interval"       , cb_g_aeit);
	srv_get_raw_sat_threshold_  = create_service<tof_camera_interface::srv::GetRawSatThreshold >("krm/get_raw_sat_threshold" , cb_g_rsth);
	srv_get_ir_dark_threshold_  = create_service<tof_camera_interface::srv::GetIrDarkThreshold >("krm/get_ir_dark_threshold" , cb_g_irth);
	srv_get_int_supp_info_      = create_service<tof_camera_interface::srv::GetIntSuppInfo     >("krm/get_int_supp_info"     , cb_g_insp);
	srv_get_play_target_        = create_service<tof_camera_interface::srv::GetPlayTarget      >("krm/get_play_target"       , cb_g_ptgt);
	srv_get_play_time_          = create_service<tof_camera_interface::srv::GetPlayTime        >("krm/get_play_time"         , cb_g_time);
	srv_get_play_status_        = create_service<tof_camera_interface::srv::GetPlayStatus      >("krm/get_play_status"       , cb_g_pstt);

	pub_event_chg_prop_ = create_publisher<tof_camera_interface::msg::EventChgProp>("krm/event_chg_prop", qos_event);
	pub_event_chg_fmt_  = create_publisher<tof_camera_interface::msg::EventChgFmt >("krm/event_chg_fmt" , qos_event);

	pub_notify_ 	   = create_publisher<tof_camera_interface::msg::Notify      >("krm/notify"       , qos_status);
	pub_frame_data_	   = create_publisher<tof_camera_interface::msg::FrameData   >("krm/tof_out"      , qos_image);
}

bool TofCtrlNode::loadParam(void)
{
	int cam_type;
	int cam_index;
	int cam_mode;
	std::string play_dir;

	declare_parameter("cam_type", 0);
	declare_parameter("cam_index", 0);
	declare_parameter("cam_mode", 0);
	declare_parameter("play_dir", ".");
	declare_parameter("start", true);
	declare_parameter("open_dev", true);

	get_parameter("cam_type", cam_type);
	get_parameter("cam_index", cam_index);
	get_parameter("cam_mode", cam_mode);
	get_parameter("play_dir", play_dir);
	get_parameter("start", auto_start_);
	get_parameter("open_dev", auto_open_);

	RCLCPP_INFO(this->get_logger(), "cam_type  : %d", cam_type);
	RCLCPP_INFO(this->get_logger(), "cam_index : %d", cam_index);
	RCLCPP_INFO(this->get_logger(), "cam_mode  : %d", cam_mode);
	RCLCPP_INFO(this->get_logger(), "play_dir  : %s", play_dir.c_str());
	RCLCPP_INFO(this->get_logger(), "start     : %d", auto_start_);
	RCLCPP_INFO(this->get_logger(), "open_dev  : %d", auto_open_);

	if (auto_open_) {
		if (cam_type == 0) {
			cam_type_ = krm::C11_USB;
		} else if (cam_type == 1) {
			cam_type_ = krm::PLAYBACK;
		} else {
			RCLCPP_ERROR(this->get_logger(), "invalid cam_type : %d", cam_type);
			return false;
		}

		if ((cam_index < 0) || (cam_index > UINT16_MAX)) {
			RCLCPP_ERROR(this->get_logger(), "invalid cam_index : %d", cam_index);
			return false;
		} else {
			cam_index_ = static_cast<uint16_t>(cam_index);
		}

		if ((cam_mode < 0) || (cam_mode > UINT8_MAX)) {
			RCLCPP_ERROR(this->get_logger(), "invalid cam_mode : %d", cam_mode);
			return false;
		} else {
			cam_mode_ = static_cast<uint8_t>(cam_mode);
		}
		play_dir_ = play_dir;
	}

	return true;
}

bool TofCtrlNode::initCamera(void)
{
	krm::Result ret;
	std::vector<krm::ConnDevice> con_list;
	uint8_t mode;
	krm::PlayBack::ConfigParam play_dir2;

	if (cam_type_ == krm::C11_USB) {
		camera_ = std::make_unique<krm::Camera>(cam_type_, nullptr, ret);
	} else if (cam_type_ == krm::PLAYBACK) {
		krm::PlayBack::ConfigParam play_dir;
		camera_ = std::make_unique<krm::Camera>(cam_type_, &play_dir, ret);
	} else {
		ret = krm::ERR_BAD_ARG;
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to create camera device : %d", ret);
		return false;
	}

	ret = camera_->getDeviceList(con_list);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "camera device is not exist : %d", ret);
		return false;
	}

	if (con_list.size() <= static_cast<size_t>(cam_index_)) {
		RCLCPP_ERROR(this->get_logger(), "invalid cam_index : %d", cam_index_);
		return false;
	}

	if(!openCamera(con_list[cam_index_].id, cam_type_)) {
		return false;
	}

	if (cam_type_ == krm::PLAYBACK) {
		ret = camera_->setProperty(krm::PlayBack::CMD_PLAY_TARGET, &play_dir_);
		if (ret != krm::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "failed to set PlayTarget : %s", play_dir_.string<char>().c_str());
			(void)closeCamera();
			camera_.reset(nullptr);
			return false;
		}
	}
	frame_id_ = FRAME_ID.data() + std::to_string(cam_index_);
	RCLCPP_INFO(this->get_logger(), "FRAME_ID = %s", frame_id_.c_str());

	ret = camera_->getProperty(krm::CMD_MODE, &mode);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get MODE : %d", ret);
		return false;
	}
	if (cam_type_ == krm::PLAYBACK) {
		cam_mode_ = mode;
		(void)notifyProperty(false);
		(void)notifyImgFmt();
	} else {
		if ((mode != cam_mode_) && !changeMode(cam_mode_)) {
			(void)closeCamera();
			camera_.reset(nullptr);
			return false;
		}
	}
	return true;
}

bool TofCtrlNode::openCamera(uint16_t id, krm::CameraType type)
{
	krm::Result ret = krm::SUCCESS;

	if (type == krm::C11_USB) {
		if ((camera_ != nullptr) && (cam_type_ != krm::C11_USB)) {
			return false;
		}
		if (camera_ == nullptr) {
			std::vector<krm::ConnDevice> conn_dev;
			camera_ = std::make_unique<krm::Camera>(type, nullptr, ret);
			camera_->getDeviceList(conn_dev);
		}
	} else if (type == krm::PLAYBACK) {
		if ((camera_ != nullptr) && (cam_type_ != krm::PLAYBACK)) {
			return false;
		}
		if (camera_ == nullptr) {
			std::vector<krm::ConnDevice> conn_dev;
			camera_ = std::make_unique<krm::Camera>(type, &play_dir_, ret);
			camera_->getDeviceList(conn_dev);
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "not support camera type : %d", type);
		return false;
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to create camera device : %d", ret);
		return false;
	}

	ret = camera_->openDevice(id);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to open camera device : %d", ret);
		return false;
	}
	cam_type_ = type;

	(void)notifyProperty(false);
	(void)notifyImgFmt();

	return true;
}

bool TofCtrlNode::closeCamera(void)
{
	krm::Result ret = krm::SUCCESS;
	if (camera_ != nullptr) {
		ret = camera_->closeDevice();
		camera_.reset(nullptr);
	}

	return (ret == krm::SUCCESS);
}

bool TofCtrlNode::checkVersion(void)
{
	if ((krm::SDK_VERSION.major != SUPPORT_SDK_VERSION.major) ||
		(krm::SDK_VERSION.minor != SUPPORT_SDK_VERSION.minor)) {
		RCLCPP_ERROR(this->get_logger(), "Invalid SDK Version : %u.%u.%u (support:%u.%u.X)",
			krm::SDK_VERSION.major, krm::SDK_VERSION.minor, krm::SDK_VERSION.rev,
			SUPPORT_SDK_VERSION.major, SUPPORT_SDK_VERSION.minor);
		return false;
	}
	return true;
}

bool TofCtrlNode::notifyProperty(bool notify_nodes)
{
	krm::Result			ret;
	uint8_t				mode;
	krm::ModeList		mode_list;

	krm::DeviceInfo		dev_info;
	krm::ModeInfo		mode_info;
	krm::LensInfo		lens_info;
	krm::CamFov			fov;
	krm::PostFiltInfo	post_filt_info;

	bool				found = false;
	tof_camera_interface::msg::EventChgProp msg_chg_prop;

	ret = camera_->getProperty(krm::CMD_MODE, &mode);
	if (ret == krm::SUCCESS) {
		ret = camera_->getProperty(krm::CMD_MODE_LIST, &mode_list);
		if (ret == krm::SUCCESS) {
			if ((cam_type_ == krm::PLAYBACK) && (mode_list.size() == 0)) {
				ret = krm::ERR_EMPTY;
			} else {
				for (uint8_t i = 0; i < static_cast<uint8_t>(mode_list.size()); i++) {
					if (mode_list[i].id == mode) {
						mode_info = mode_list[i];
						found = true;
						break;
					}
				}
				if (!found) {
					RCLCPP_ERROR(this->get_logger(), "Mode ID[%u] is invalid", mode);
					ret = krm::ERR_NOT_EXIST;
				} else {
					ret = camera_->getProperty(krm::CMD_DEV_INFO, &dev_info);
					if (ret == krm::SUCCESS) {
						ret = camera_->getProperty(krm::CMD_LENS_INFO, &lens_info);
						if (ret == krm::SUCCESS) {
							ret = camera_->getProperty(krm::CMD_FOV, &fov);
							if (ret == krm::SUCCESS) {
								ret = camera_->getProperty(krm::CMD_POSTFILT_INFO, &post_filt_info);
								if (ret == krm::SUCCESS) {
									convertDeviceInfo(dev_info, msg_chg_prop.dev_info);
									convertModeInfo(mode_info, msg_chg_prop.mode_info);
									convertLensInfo(lens_info, msg_chg_prop.lens_info);
									convertCamFov(fov, msg_chg_prop.fov);
									convertPostFiltInfo(post_filt_info, msg_chg_prop.post_filt_info);
								}
							}
						}
					}
				}
			}
		}
	}
	if (notify_nodes) {
		if (ret == krm::SUCCESS) {
			pub_event_chg_prop_->publish(msg_chg_prop);
		}
	} else {
		if (ret != krm::SUCCESS) {
			msg_chg_prop.dev_info.hw_kind = 0;
			msg_chg_prop.dev_info.serial_no = 0;
			msg_chg_prop.dev_info.map_ver.major = 0;
			msg_chg_prop.dev_info.map_ver.minor = 0;
			msg_chg_prop.dev_info.map_ver.rev = 0;
			msg_chg_prop.dev_info.firm_ver.major = 0;
			msg_chg_prop.dev_info.firm_ver.minor = 0;
			msg_chg_prop.dev_info.firm_ver.rev = 0;
			msg_chg_prop.dev_info.adjust_no = 0;
			msg_chg_prop.dev_info.ld_wave = 0;
			msg_chg_prop.dev_info.ld_enable = 0;
			msg_chg_prop.dev_info.correct_calib = 0;
			msg_chg_prop.mode_info.id = 0;
			msg_chg_prop.mode_info.description.clear();
			msg_chg_prop.mode_info.img_out.clear();
			msg_chg_prop.mode_info.dist_range.min = 0;
			msg_chg_prop.mode_info.dist_range.max = 0xFFFFU;
			msg_chg_prop.mode_info.fps = 0;
			msg_chg_prop.mode_info.thin_w = 1U;
			msg_chg_prop.mode_info.thin_h = 1U;
			msg_chg_prop.mode_info.crop.x = 0;
			msg_chg_prop.mode_info.crop.y = 0;
			msg_chg_prop.mode_info.light_times = false;
			msg_chg_prop.mode_info.range_calib = 0;
			msg_chg_prop.lens_info.sens_w = 1U;
			msg_chg_prop.lens_info.sens_h = 1U;
			msg_chg_prop.lens_info.focal_len = 1U;
			msg_chg_prop.lens_info.thin_w = 1U;
			msg_chg_prop.lens_info.thin_h = 1U;
			msg_chg_prop.lens_info.crop.x = 0;
			msg_chg_prop.lens_info.crop.y = 0;
			msg_chg_prop.lens_info.cam_dist = false;
			std::fill(msg_chg_prop.lens_info.dist.begin(), msg_chg_prop.lens_info.dist.end(), 0);
			msg_chg_prop.lens_info.lens_calib = 0;
			msg_chg_prop.fov.horz = 0;
			msg_chg_prop.fov.vert = 0;
			msg_chg_prop.post_filt_info.cam_med_filt = false;
			msg_chg_prop.post_filt_info.cam_bil_filt = false;
			msg_chg_prop.post_filt_info.cam_fly_p_filt = false;
			ret = krm::SUCCESS;
		}
		pub_event_chg_prop_->publish(msg_chg_prop);
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to notify property : %d", ret);
		return false;
	}
	cur_fps_ = mode_info.fps;
	return true;
}

bool TofCtrlNode::notifyImgFmt(void)
{
	krm::Result ret;
	bool res;
	uint8_t size;
	tof_camera_interface::msg::EventChgFmt msg_chg_fmt;

	ret = camera_->getProperty(krm::CMD_IMG_FORMAT, &img_fmts_);
	if (ret == krm::SUCCESS) {
		size = static_cast<uint8_t>(img_fmts_.size());

		msg_chg_fmt.formats.data.resize(size);
		for (uint8_t i = 0; i < size; i++) {
			convertImageFormat(img_fmts_[i], msg_chg_fmt.formats.data[i]);
		}
		pub_event_chg_fmt_->publish(msg_chg_fmt);
		res = true;
	} else {
		RCLCPP_ERROR(this->get_logger(), "Failed to notify Image Formats : %d", ret);
		res = false;
	}
	return res;
}

void TofCtrlNode::resetFpsCount(void)
{
	std::lock_guard<std::recursive_mutex> lock(th_mtx_);
	first_frame_ = true;
}

uint32_t TofCtrlNode::getFpsCalcCycle(void)
{
	krm::Result		ret;
	krm::ModeList	list;
	uint8_t		mode;
	uint32_t	cycle = 15U;

	ret = camera_->getProperty(krm::CMD_MODE_LIST, &list);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to get Mode list : %d", ret);
	} else {
		ret = camera_->getProperty(krm::CMD_MODE, &mode);
		if (ret != krm::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "Failed to get Mode : %d", ret);
		} else {
			for (auto info : list) {
				if (info.id == mode) {
					cycle = info.fps / 200U;	// (fps / 2) / 100
					if (cycle == 0) {
						RCLCPP_INFO(this->get_logger(), "Under 1fps");
						cycle = 1U;
					}
					break;
				}
			}
		}
	}
	return cycle;
}

void TofCtrlNode::calcFps(const struct timespec& pre_time,
	const struct timespec& cur_time,
	uint32_t cycle, uint64_t& total_time,
	uint32_t& frame_cnt, uint16_t& rcv_fps)
{
	int64_t diff_sec  = static_cast<int64_t>(cur_time.tv_sec) - static_cast<int64_t>(pre_time.tv_sec);
	int64_t diff_nsec = static_cast<int64_t>(cur_time.tv_nsec) - static_cast<int64_t>(pre_time.tv_nsec);

	frame_cnt++;
	if (diff_sec < 0) {
		diff_sec = 0;
		RCLCPP_WARN(this->get_logger(), "cur:%lld.%lld, pre:%lld.%lld",
			static_cast<unsigned long long>(cur_time.tv_sec), static_cast<unsigned long long>(cur_time.tv_nsec),
			static_cast<unsigned long long>(pre_time.tv_sec), static_cast<unsigned long long>(pre_time.tv_nsec));
	}
	if (diff_nsec < 0) {
		if (diff_sec > 0) {
			diff_sec--;
			diff_nsec = 1000000000 - std::abs(diff_nsec);
		} else {
			diff_nsec = 0;
			RCLCPP_WARN(this->get_logger(), "cur:%lld.%lld, pre:%lld.%lld",
				static_cast<unsigned long long>(cur_time.tv_sec), static_cast<unsigned long long>(cur_time.tv_nsec),
				static_cast<unsigned long long>(pre_time.tv_sec), static_cast<unsigned long long>(pre_time.tv_nsec));
		}
	}
	total_time += (static_cast<uint64_t>(diff_sec) * 1000000U) + (static_cast<int64_t>(diff_nsec) / 1000U);
	if (frame_cnt >= cycle) {
		uint64_t ave = total_time / cycle;
		if (ave > 0) {
			rcv_fps = static_cast<uint16_t>(100000000U / ave);		// us to fps * 100
		}
		total_time = 0;
		frame_cnt = 0;
	}
}

bool TofCtrlNode::changeMode(uint8_t mode)
{
	krm::Result	ret;

	ret = camera_->setProperty(krm::CMD_MODE, &mode);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set MODE : %d", ret);
		return false;
	}
	if (!notifyProperty()) {
		return false;
	}

	return notifyImgFmt();
}

bool TofCtrlNode::startCapture(void)
{
	krm::Result ret;
	std::lock_guard<std::recursive_mutex> lock(th_mtx_);

	if (cap_thread_) { stopCapture(); }
	cap_thread_ = std::make_unique<std::thread>(&TofCtrlNode::captureThread, this);

	resetFpsCount();
	ret = camera_->startCapture();
	if (ret != krm::SUCCESS) {
		if (cap_thread_->joinable()) {
			camera_->cancel();
			cap_thread_->join();
		}
		cap_thread_.reset(nullptr);
		RCLCPP_ERROR(this->get_logger(), "failed to auto start capture : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::stopCapture(void)
{
	std::lock_guard<std::recursive_mutex> lock(th_mtx_);
	krm::Result ret = camera_->stopCapture();
	std::unique_ptr<tof_camera_interface::msg::FrameData> msg = std::make_unique<tof_camera_interface::msg::FrameData>();
	if (cap_thread_) {
		if (cap_thread_->joinable()) {
			camera_->cancel();
			cap_thread_->join();
		}
		cap_thread_.reset(nullptr);
	}

	msg->stopped = true;
	pub_frame_data_->publish(std::move(msg));

	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to stop Capture");
		return false;
	}
	return true;
}

bool TofCtrlNode::setPlayTime(uint32_t time)
{
	krm::Result ret;
	krm::PlayBack::PlayTime play_time;	// unit is [frame]
	play_time.current = time;
	ret = camera_->setProperty(krm::PlayBack::CMD_PLAY_TIME, &play_time);
	if(ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set PLAY_TIME : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::setPlayPause(void)
{
	krm::Result ret = camera_->setProperty(krm::PlayBack::CMD_PAUSE);
	resetFpsCount();
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set PAUSE : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::setPlayFast(void)
{
	krm::Result ret = camera_->setProperty(krm::PlayBack::CMD_FAST_PLAY);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set FAST_PLAY : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::setPlaySlow(void)
{
	krm::Result ret = camera_->setProperty(krm::PlayBack::CMD_SLOW_PLAY);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set SLOW_PLAY : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::setPlayJumpFw(uint32_t time)
{
	krm::Result ret = camera_->setProperty(krm::PlayBack::CMD_JUMP_FW, &time);	// unit is [frame]
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set JUMP_FW : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::setPlayJumpBw(uint32_t time)
{
	krm::Result ret = camera_->setProperty(krm::PlayBack::CMD_JUMP_BW, &time);	// unit is [frame]
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set JUMP_BW : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::getPlayTime(krm::PlayBack::PlayTime& time)
{
	krm::Result ret = camera_->getProperty(krm::PlayBack::CMD_PLAY_TIME, &time);	// unit is [frame]
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get PLAY_TIME : %d", ret);
		return false;
	}

	return true;
}

bool TofCtrlNode::getPlayStatus(krm::PlayBack::PlayStatus& status)
{
	krm::Result ret = camera_->getProperty(krm::PlayBack::CMD_PLAY_STATUS, &status);	// unit is [frame]
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get PLAY_STATUS : %d", ret);
		return false;
	}

	return true;
}

void TofCtrlNode::convPlayTime(tof_camera_interface::msg::PlayTime& play_time)
{
	if (cam_type_ == krm::PLAYBACK) {
		krm::PlayBack::PlayTime time;
		krm::Result ret = camera_->getProperty(krm::PlayBack::CMD_PLAY_TIME, &time);
		if (ret == krm::SUCCESS) {
			play_time.current = time.current;
			play_time.total = time.total;
		}
	} else {
		play_time.current = 0;
		play_time.total = 0;
	}
}

void TofCtrlNode::convImage(const krm::ImageData* src, const krm::ImageFormat* fmt, sensor_msgs::msg::Image* dst)
{
	size_t size;
	dst->header.stamp.sec = static_cast<builtin_interfaces::msg::Time_<std::allocator<void>>::_sec_type>(src->info.time.tv_sec);
	dst->header.stamp.nanosec = src->info.time.tv_nsec;
	dst->header.frame_id = frame_id_;
	dst->height = fmt->height;
	dst->width = fmt->width;
	dst->encoding = sensor_msgs::image_encodings::MONO16;
	dst->is_bigendian = 0;	// little
	dst->step = fmt->width * fmt->bpp;
	size = dst->step * dst->height;
	dst->data.resize(size);
	std::memcpy(dst->data.data(), src->data.data(), size);
}

void TofCtrlNode::pubFrameData(const krm::Frame& frame, uint16_t rcv_fps)
{
	std::unique_ptr<tof_camera_interface::msg::FrameData> msg = std::make_unique<tof_camera_interface::msg::FrameData>();
	tof_camera_interface::msg::FrameInfo* frame_info[krm::IMG_KINDS] = {&msg->depth.info, &msg->ir.info, &msg->raw1.info, &msg->raw2.info, &msg->raw3.info, &msg->raw4.info};
	sensor_msgs::msg::Image* sens_image[krm::IMG_KINDS] = {&msg->depth.image, &msg->ir.image, &msg->raw1.image, &msg->raw2.image, &msg->raw3.image, &msg->raw4.image};

	const krm::ImageData* img;
	krm::ImageFormat* fmt;
	tof_camera_interface::msg::FrameInfo* info;
	sensor_msgs::msg::Image* sens;

	convPlayTime(msg->play_time);
	msg->discontinuity = false;
	msg->stopped = false;
	msg->rcv_fps = rcv_fps;
	for (uint8_t i = 0; i < static_cast<uint8_t>(krm::IMG_KINDS); i++) {
		fmt = &img_fmts_[i];
		sens = sens_image[i];
		if (fmt->isExist()) {
			img = &frame.images[i];
			info = frame_info[i];
			msg->discontinuity |= (img->info.frm_err.drop != 0);
			info->number = img->info.number;
			info->frm_err = *reinterpret_cast<const uint16_t*>(&img->info.frm_err);
			info->temperature = img->info.temperature;
			info->light_cnt = img->info.light_cnt;
			info->conv_stat = *reinterpret_cast<const uint8_t*>(&img->info.conv_stat);
			convImage(img, fmt, sens);
		} else {
			sens->height = 0;
			sens->width = 0;
			sens->encoding = sensor_msgs::image_encodings::MONO16;
			sens->is_bigendian = 0;	// little
			sens->step = fmt->width * fmt->bpp;
			sens->data.clear();
		}
	}
	pub_frame_data_->publish(std::move(msg));
}

void TofCtrlNode::cbSrvGetDevList(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetDevList::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::GetDevList::Response> response)
{
	krm::Result ret = krm::SUCCESS;
	std::vector<krm::ConnDevice> con_list;
	tof_camera_interface::msg::ConnDevice conn_dev;
	std::unique_ptr<krm::Camera> cam_obj;
	krm::PlayBack::ConfigParam play_dir;

	RCLCPP_INFO(this->get_logger(), "Get Device List (type:%d)", request->type.type);
	response->result = false;

	switch (request->type.type) {
	case tof_camera_interface::msg::CameraType::C11_USB:
		if ((camera_ == nullptr) || (cam_type_ != krm::C11_USB)) {
			cam_obj = std::make_unique<krm::Camera>(krm::C11_USB, nullptr, ret);
		}
		break;
	case tof_camera_interface::msg::CameraType::PLAYBACK:
		if ((camera_ == nullptr) || (cam_type_ != krm::PLAYBACK)) {
			cam_obj = std::make_unique<krm::Camera>(krm::PLAYBACK, &play_dir, ret);
		}
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "not support camera type : %d", request->type.type);
		return;
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to create camera device : %d", ret);
		return;
	}

	ret = cam_obj->getDeviceList(con_list);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "camera device is not exist : %d", ret);
		return;
	}

	response->dev_list.clear();
	for (auto dev : con_list) {
		conn_dev.id = dev.id;
		conn_dev.name = dev.name;
		response->dev_list.push_back(conn_dev);
	}
	response->result = true;
}

void TofCtrlNode::cbSrvOpenDev(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::OpenDev::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::OpenDev::Response> response)
{
	krm::CameraType type = static_cast<krm::CameraType>(request->type.type);
	uint16_t id = request->dev_id;
	RCLCPP_INFO(this->get_logger(), "Open Device(dev_type:%d, dev_id:%d)", request->type.type, request->dev_id);
	response->result = openCamera(id, type);
}

void TofCtrlNode::cbSrvCloseDev(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::CloseDev::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::CloseDev::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Close Device");
	response->result = closeCamera();
}

void TofCtrlNode::cbSrvTofCtrl(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::TofCtrl::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::TofCtrl::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Control Command(%u)", request->cmd);
	response->result = false;
	if (camera_ == nullptr) { return; }

	switch (request->cmd) {
	case tof_camera_interface::srv::TofCtrl::Request::CMD_START:
		response->result = startCapture();
		break;
	case tof_camera_interface::srv::TofCtrl::Request::CMD_STOP:
		response->result = stopCapture();
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "not support command type : %d", request->cmd);
		break;
	}
}

void TofCtrlNode::cbSrvSetExtTriggerType(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerType::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerType::Response> response)
{
	krm::Result			ret;
	krm::ExtTriggerType	ext_trg_type;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Set ExtTriggerType");
	if (camera_ == nullptr) { return; }

	ext_trg_type = static_cast<krm::ExtTriggerType>(request->type.ext_trigger_type);
	ret = camera_->setProperty(krm::CMD_EXT_TRG_TYPE, &ext_trg_type);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set EXT_TRG_TYPE : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetExtTriggerOffset(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerOffset::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetExtTriggerOffset::Response> response)
{
	krm::Result ret;
	uint8_t		ext_trg_offset;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "ExtTriggerOffset");
	if (camera_ == nullptr) { return; }

	ext_trg_offset = request->offset;
	ret = camera_->setProperty(krm::CMD_EXT_TRG_OFFSET, &ext_trg_offset);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set EXT_TRG_OFFSET : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetMode(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetMode::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetMode::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Change Mode(%u)", request->mode);
	response->result = false;
	if (camera_ == nullptr) { return; }
	response->result = changeMode(request->mode);
}

void TofCtrlNode::cbSrvSetImgKinds(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetImgKinds::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetImgKinds::Response> response)
{
	krm::Result		ret;
	krm::ImgOutKind	img_kind;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "ImgKinds");
	if (camera_ == nullptr) { return; }

	img_kind = static_cast<krm::ImgOutKind>(request->img_out.img_out_kind);
	ret = camera_->setProperty(krm::CMD_IMG_KINDS, &img_kind);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set IMG_KIND : %d", ret);
		return;
	}
	response->result = notifyImgFmt();
}

void TofCtrlNode::cbSrvSetLightTimes(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetLightTimes::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetLightTimes::Response> response)
{
	krm::Result			ret;
	krm::LightTimesInfo	light_times;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "LightTimes");
	if (camera_ == nullptr) { return; }

	light_times.count = request->count;
	ret = camera_->setProperty(krm::CMD_LIGHT_TIMES, &light_times);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set LIGHT_TIMES : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetAEState(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetAEState::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetAEState::Response> response)
{
	krm::Result	ret;
	bool		ae_state;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "AEState");
	if (camera_ == nullptr) { return; }

	ae_state = request->enable;
	ret = camera_->setProperty(krm::CMD_AE_STATE, &ae_state);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set AE_STATE : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetAEInterval(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetAEInterval::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetAEInterval::Response> response)
{
	krm::Result			ret;
	krm::AEIntervalInfo	ae_interval;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "AEInterval");
	if (camera_ == nullptr) { return; }

	ae_interval.interval = request->interval;
	ret = camera_->setProperty(krm::CMD_AE_INTERVAL, &ae_interval);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set AE_INTERVAL : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetRawSatThreshold(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetRawSatThreshold::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetRawSatThreshold::Response> response)
{
	krm::Result					ret;
	krm::SignalThresholdInfo	threshold;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "RawSatThreshold");
	if (camera_ == nullptr) { return; }

	threshold.threshold = request->raw_threshold;
	ret = camera_->setProperty(krm::CMD_RAW_SAT_TH, &threshold);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set RAW_SAT_TH : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetIrDarkThreshold(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetIrDarkThreshold::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetIrDarkThreshold::Response> response)
{
	krm::Result					ret;
	krm::SignalThresholdInfo	threshold;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "IrDarkThreshold");
	if (camera_ == nullptr) { return; }

	threshold.threshold = request->ir_threshold;
	ret = camera_->setProperty(krm::CMD_IR_DARK_TH, &threshold);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set IR_DARK_TH : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetIntSupp(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetIntSupp::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetIntSupp::Response> response)
{
	krm::Result			ret;
	krm::IntSuppInfo	int_supp;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "IntSupp");
	if (camera_ == nullptr) { return; }

	int_supp.mode         = static_cast<krm::IntSuppModeType>(request->int_supp_mode.int_supp_mode_type);
	int_supp.prm_m.value  = request->int_supp_prm_m;
	int_supp.prm_a1.value = request->int_supp_prm_a1;
	int_supp.prm_a2.value = request->int_supp_prm_a2;
	int_supp.prm_a3.value = request->int_supp_prm_a3;
	ret = camera_->setProperty(krm::CMD_INT_SUPP_INFO, &int_supp);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set INT_SUPP_INFO : %d", ret);
		return;
	}
	response->result = true;
}

void TofCtrlNode::cbSrvSetPlayTarget(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPlayTarget::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPlayTarget::Response> response)
{
	krm::Result				ret;
	std::filesystem::path	path;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "set Playback Target (%s)", request->directory.c_str());
	if (camera_ == nullptr) { return; }

#ifdef LNX_FUNC
	path.assign(request->directory);
#else	/* LNX_FUNC */
	path = std::filesystem::u8path(request->directory.c_str());
#endif	/* LNX_FUNC */

	ret = camera_->setProperty(krm::PlayBack::CMD_PLAY_TARGET, &path);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to set PLAY_TARGET : %d", ret);
		return;
	}
	response->result = (notifyProperty(false) && notifyImgFmt());
}

void TofCtrlNode::cbSrvSetPlayCtrl(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPlayCtrl::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPlayCtrl::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Play Control Command(%u)", request->cmd);
	response->result = false;
	if (camera_ == nullptr) { return; }
	if (cam_type_ != krm::PLAYBACK) { return; }

	switch (request->cmd) {
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_PLAY_TIME:
		RCLCPP_INFO(this->get_logger(), "Play Control (time value = %u[frame])", request->time);
		response->result = setPlayTime(request->time);
		break;
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_PAUSE:
		response->result = setPlayPause();
		break;
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_FAST_PLAY:
		response->result = setPlayFast();
		break;
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_SLOW_PLAY:
		response->result = setPlaySlow();
		break;
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_JUMP_FW:
		RCLCPP_INFO(this->get_logger(), "Play Time (time value = %u[frame])", request->time);
		response->result = setPlayJumpFw(request->time);
		break;
	case tof_camera_interface::srv::SetPlayCtrl::Request::CMD_JUMP_BW:
		RCLCPP_INFO(this->get_logger(), "Play Time (time value = %u[frame])", request->time);
		response->result = setPlayJumpBw(request->time);
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "not support command type : %d", request->cmd);
		break;
	}
}

void TofCtrlNode::cbSrvGetDevInfo(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetDevInfo::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetDevInfo::Response> response)
{
	krm::Result			ret;
	krm::DeviceInfo		dev_info;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get DevInfo");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_DEV_INFO, &dev_info);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get DEV_INFO : %d", ret);
		return;
	}

	response->result = true;
	convertDeviceInfo(dev_info, response->dev_info);
}

void TofCtrlNode::cbSrvGetFov(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetFov::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetFov::Response> response)
{
	krm::Result		ret;
	krm::CamFov		cam_fov;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get Fov");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_FOV, &cam_fov);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get FOV : %d", ret);
		return;
	}

	response->result = true;
	convertCamFov(cam_fov, response->cam_fov);
}

void TofCtrlNode::cbSrvGetExtTriggerType(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerType::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerType::Response> response)
{
	krm::Result				ret;
	krm::ExtTriggerType		ext_trig_type;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get ExtTriggerType");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_EXT_TRG_TYPE, &ext_trig_type);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get EXT_TRG_TYPE : %d", ret);
		return;
	}

	response->result = true;
	response->type.ext_trigger_type = static_cast<uint8_t>(ext_trig_type);
}

void TofCtrlNode::cbSrvGetExtTriggerOffset(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerOffset::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetExtTriggerOffset::Response> response)
{
	krm::Result		ret;
	uint8_t			ext_trig_offset;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get ExtTriggerOffset");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_EXT_TRG_OFFSET, &ext_trig_offset);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get EXT_TRG_OFFSET : %d", ret);
		return;
	}

	response->result = true;
	response->offset = ext_trig_offset;
}

void TofCtrlNode::cbSrvGetModeList(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetModeList::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetModeList::Response> response)
{
	krm::Result		ret;
	krm::ModeList	mode_list;
	tof_camera_interface::msg::ModeInfo info;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get ModeList");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_MODE_LIST, &mode_list);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get EXT_TRG_OFFSET : %d", ret);
		return;
	}

	response->result = true;
	response->mode_list.clear();
	for (auto mode_info : mode_list) {
		convertModeInfo(mode_info, info);
		response->mode_list.push_back(info);
	}
}

void TofCtrlNode::cbSrvGetMode(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetMode::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetMode::Response> response)
{
	krm::Result		ret;
	uint8_t			mode;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get Mode");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_MODE, &mode);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get MODE : %d", ret);
		return;
	}

	response->result = true;
	response->mode = mode;
}

void TofCtrlNode::cbSrvGetImgKinds(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetImgKinds::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetImgKinds::Response> response)
{
	krm::Result		ret;
	uint8_t			img_out_kind;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get ImgKinds");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_IMG_KINDS, &img_out_kind);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get IMG_KINDS : %d", ret);
		return;
	}

	response->result = true;
	response->img_out.img_out_kind = img_out_kind;
}

void TofCtrlNode::cbSrvGetImgFormat(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetImgFormat::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetImgFormat::Response> response)
{
	krm::Result			ret;
	uint8_t				size;
	krm::ImageFormats	img_fmts;
	tof_camera_interface::msg::ImageFormat fmt;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get ImgFormat");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_IMG_FORMAT, &img_fmts);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get EXT_TRG_OFFSET : %d", ret);
		return;
	}

	response->result = true;
	size = static_cast<uint8_t>(img_fmts.size());
	response->img_fmts.data.clear();
	response->img_fmts.data.resize(size);
	for (uint8_t i = 0; i < size; i++) {
		convertImageFormat(img_fmts[i], response->img_fmts.data[i]);
	}
}

void TofCtrlNode::cbSrvGetPostFiltInfo(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPostFiltInfo::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPostFiltInfo::Response> response)
{
	krm::Result			ret;
	krm::PostFiltInfo	post_filt_info;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get PostFiltInfo");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_POSTFILT_INFO, &post_filt_info);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get POSTFILT_INFO : %d", ret);
		return;
	}

	response->result = true;
	convertPostFiltInfo(post_filt_info, response->post_filt_info);
}

void TofCtrlNode::cbSrvGetLensInfo(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetLensInfo::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetLensInfo::Response> response)
{
	krm::Result		ret;
	krm::LensInfo	lens_info;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get LensInfo");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_LENS_INFO, &lens_info);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get LENS_INFO : %d", ret);
		return;
	}

	response->result = true;
	convertLensInfo(lens_info, response->lens_info);
}

void TofCtrlNode::cbSrvGetLightTimes(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetLightTimes::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetLightTimes::Response> response)
{
	krm::Result			ret;
	krm::LightTimesInfo	light_times;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get LightTimes");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_LIGHT_TIMES, &light_times);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get LIGHT_TIMES : %d", ret);
		return;
	}

	response->result = true;
	response->light_times.min = light_times.min;
	response->light_times.value = light_times.count;
	response->light_times.max = light_times.max;
}

void TofCtrlNode::cbSrvGetAEState(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetAEState::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetAEState::Response> response)
{
	krm::Result		ret;
	bool			ae_state;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get AEState");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_AE_STATE, &ae_state);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get AE_STATE : %d", ret);
		return;
	}

	response->result = true;
	response->enable = ae_state;
}

void TofCtrlNode::cbSrvGetAEInterval(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetAEInterval::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetAEInterval::Response> response)
{
	krm::Result			ret;
	krm::AEIntervalInfo	ae_interval;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get AEInterval");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_AE_INTERVAL, &ae_interval);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get AE_INTERVAL : %d", ret);
		return;
	}

	response->result = true;
	response->interval.min = ae_interval.min;
	response->interval.value = ae_interval.interval;
	response->interval.max = ae_interval.max;
}

void TofCtrlNode::cbSrvGetRawSatThreshold(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetRawSatThreshold::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetRawSatThreshold::Response> response)
{
	krm::Result					ret;
	krm::SignalThresholdInfo	raw_sat_th;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get RawSatThreshold");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_RAW_SAT_TH, &raw_sat_th);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get RAW_SAT_TH : %d", ret);
		return;
	}

	response->result = true;
	response->raw_sat_th.min = raw_sat_th.min;
	response->raw_sat_th.value = raw_sat_th.threshold;
	response->raw_sat_th.max = raw_sat_th.max;
}

void TofCtrlNode::cbSrvGetIrDarkThreshold(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetIrDarkThreshold::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetIrDarkThreshold::Response> response)
{
	krm::Result					ret;
	krm::SignalThresholdInfo	ir_dark_th;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get IrDarkThreshold");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_IR_DARK_TH, &ir_dark_th);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get IR_DARK_TH : %d", ret);
		return;
	}

	response->result = true;
	response->ir_dark_th.min = ir_dark_th.min;
	response->ir_dark_th.value = ir_dark_th.threshold;
	response->ir_dark_th.max = ir_dark_th.max;
}

void TofCtrlNode::cbSrvGetIntSuppInfo(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetIntSuppInfo::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetIntSuppInfo::Response> response)
{
	krm::Result			ret;
	krm::IntSuppInfo	int_supp_info;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get IntSuppInfo");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::CMD_INT_SUPP_INFO, &int_supp_info);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get INT_SUPP_INFO : %d", ret);
		return;
	}

	response->result = true;
	response->int_supp_info.mode.int_supp_mode_type = int_supp_info.mode;
	response->int_supp_info.prm_m.min               = int_supp_info.prm_m.min;
	response->int_supp_info.prm_m.max               = int_supp_info.prm_m.max;
	response->int_supp_info.prm_m.value             = int_supp_info.prm_m.value;
	response->int_supp_info.prm_a1.min              = int_supp_info.prm_a1.min;
	response->int_supp_info.prm_a1.max              = int_supp_info.prm_a1.max;
	response->int_supp_info.prm_a1.value            = int_supp_info.prm_a1.value;
	response->int_supp_info.prm_a2.min              = int_supp_info.prm_a2.min;
	response->int_supp_info.prm_a2.max              = int_supp_info.prm_a2.max;
	response->int_supp_info.prm_a2.value            = int_supp_info.prm_a2.value;
	response->int_supp_info.prm_a3.min              = int_supp_info.prm_a3.min;
	response->int_supp_info.prm_a3.max              = int_supp_info.prm_a3.max;
	response->int_supp_info.prm_a3.value            = int_supp_info.prm_a3.value;

}

void TofCtrlNode::cbSrvGetPlayTarget(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayTarget::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayTarget::Response> response)
{
	krm::Result				ret;
	std::filesystem::path	play_dir;

	response->result = false;
	RCLCPP_INFO(this->get_logger(), "Get PlayTarget");
	if (camera_ == nullptr) { return; }

	ret = camera_->getProperty(krm::PlayBack::CMD_PLAY_TARGET, &play_dir);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to get PLAY_TARGET : %d", ret);
		return;
	}

#ifdef LNX_FUNC
	std::string str = play_dir.string<char>();
#else	/* LNX_FUNC */
#ifdef __cpp_lib_char8_t
	std::u8string u8_str = play_dir.u8string();
	std::string str = std::string(u8_str.begin(), u8_str.end());
#else	/* __cpp_lib_char8_t */
	std::string str = play_dir.u8string();
#endif	/* __cpp_lib_char8_t */
#endif	/* LNX_FUNC */

	response->result = true;
	response->directory = str;

}

void TofCtrlNode::cbSrvGetPlayTime(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayTime::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayTime::Response> response)
{
	krm::PlayBack::PlayTime play_time;

	RCLCPP_INFO(this->get_logger(), "get current Play Time");
	response->result = false;
	response->play_time.total = 0;
	response->play_time.current = 0;

	if (camera_ == nullptr) { return; }
	if (cam_type_ != krm::PLAYBACK) { return; }

	response->result = getPlayTime(play_time);
	if (response->result) {
		response->play_time.total = play_time.total;
		response->play_time.current = play_time.current;
	}
}

void TofCtrlNode::cbSrvGetPlayStatus(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayStatus::Request> /*request*/,
	const std::shared_ptr<tof_camera_interface::srv::GetPlayStatus::Response> response)
{
	krm::PlayBack::PlayStatus status;

	RCLCPP_INFO(this->get_logger(), "get current Play Status");
	response->result = false;
	response->state = 0;
	response->playing_fps = 0;

	if (camera_ == nullptr) { return; }
	if (cam_type_ != krm::PLAYBACK) { return; }

	response->result = getPlayStatus(status);
	if (response->result) {
		response->state = status.state;
		response->playing_fps = status.playing_fps;
	}
}

void TofCtrlNode::captureThread(void)
{
	krm::Result ret;
	krm::Frame	frame;
	bool		loop = true;
	uint8_t		retry = 0;
	uint64_t	total_time = 0;
	struct timespec	pre_time, cur_time;
	uint32_t	frame_cnt = 0;
	uint32_t	cycle = getFpsCalcCycle();
	uint16_t	rcv_fps = UINT16_MAX;
	tof_camera_interface::msg::Notify notify;

	RCLCPP_INFO(this->get_logger(), "waked thread");

	for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
		frame.images[i].resize(img_fmts_[i]);
	}

	first_frame_ = true;

	while (loop) {
		ret = camera_->capture(frame);
		switch (ret) {
		case krm::SUCCESS:
			timespec_get(&cur_time, TIME_UTC);
			if (first_frame_) {
				first_frame_ = false;
				total_time = 0;
				frame_cnt = 0;
				rcv_fps = UINT16_MAX;
			} else {
				calcFps(pre_time, cur_time, cycle, total_time, frame_cnt, rcv_fps);
			}
			pre_time = cur_time;
			retry = 0;

			pubFrameData(frame, rcv_fps);
			break;
		case krm::CANCELED:
			RCLCPP_INFO(this->get_logger(), "Canceled");
			loop = false;
			break;
		case krm::REACH_EOF:
			if (cam_type_ == krm::PLAYBACK) {
				RCLCPP_INFO(this->get_logger(), "PlayBack : Reached EOF");
				notify.notify = tof_camera_interface::msg::Notify::PLAY_REACHED_EOF;
				pub_notify_->publish(notify);
				loop = false;
				th_mtx_.lock();
				cap_thread_->detach();
				th_mtx_.unlock();
			}
			break;
		case krm::ERR_TIMEOUT:
			if (!retryCapture(retry)) {
				notify.notify = tof_camera_interface::msg::Notify::ERR_TIMEOUT;
				pub_notify_->publish(notify);
				loop = false;
				th_mtx_.lock();
				cap_thread_->detach();
				th_mtx_.unlock();
			}
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "Failed capture : %d", ret);
			notify.notify = tof_camera_interface::msg::Notify::ERR_TIMEOUT;
			pub_notify_->publish(notify);
			loop = false;
			th_mtx_.lock();
			cap_thread_->detach();
			th_mtx_.unlock();
			break;
		}
	}

	RCLCPP_INFO(this->get_logger(), "exit thread");
}

bool TofCtrlNode::retryCapture(uint8_t& retry)
{
	krm::Result ret;
	retry++;
	if (retry > CAP_RETRY_MAX) {
		RCLCPP_ERROR(this->get_logger(), "Timeout capture : give-up retry");
		return false;
	} else {
		ret = camera_->stopCapture();
		if (ret != krm::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "Give up retry(failed stop) : %d", ret);
			return false;
		} else {
			std::this_thread::sleep_for(std::chrono::milliseconds(CAP_RETRY_WAIT));
			ret = camera_->startCapture();
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Give up retry(failed restart) : %d", ret);
				return false;
			} else {
				RCLCPP_INFO(this->get_logger(), "Timeout capture : restarted(retry:%u)", retry);
			}
		}
	}
	return true;
}

void TofCtrlNode::convertDeviceInfo(krm::DeviceInfo& from, tof_camera_interface::msg::DeviceInfo& to)
{
	to.hw_kind = from.hw_kind;
	to.serial_no = from.serial_no;
	to.map_ver.major = from.map_ver.major;
	to.map_ver.minor = from.map_ver.minor;
	to.map_ver.rev = from.map_ver.rev;
	to.firm_ver.major = from.firm_ver.major;
	to.firm_ver.minor = from.firm_ver.minor;
	to.firm_ver.rev = from.firm_ver.rev;
	to.adjust_no = from.adjust_no;
	to.ld_wave = from.ld_wave;
	to.ld_enable = from.ld_enable;
	to.correct_calib = from.correct_calib;
}

void TofCtrlNode::convertLensInfo(krm::LensInfo& from, tof_camera_interface::msg::LensInfo& to)
{
	uint8_t dist_size = static_cast<uint8_t>(sizeof(from.dist) / sizeof(from.dist[0]));

	to.sens_w = from.sens_w;
	to.sens_h = from.sens_h;
	to.focal_len = from.focal_len;
	to.thin_w = from.thin_w;
	to.thin_h = from.thin_h;
	to.crop.x = from.crop.x;
	to.crop.y = from.crop.y;
	to.cam_dist = from.cam_dist;
	for (uint8_t i = 0; i < dist_size; i++) {
		to.dist[i] = from.dist[i];
	}
	to.lens_calib = from.lens_calib;
}

void TofCtrlNode::convertCamFov(krm::CamFov& from, tof_camera_interface::msg::CamFov& to)
{
	to.horz = from.horz;
	to.vert = from.vert;
}

void TofCtrlNode::convertPostFiltInfo(krm::PostFiltInfo& from, tof_camera_interface::msg::PostFiltInfo& to)
{
	to.cam_med_filt = from.cam_med_filt;
	to.cam_bil_filt = from.cam_bil_filt;
	to.cam_fly_p_filt = from.cam_fly_p_filt;
}

void TofCtrlNode::convertModeInfo(krm::ModeInfo& from, tof_camera_interface::msg::ModeInfo& to)
{
	to.id = from.id;
	to.description = from.description;
	to.img_out.clear();
	to.img_out.resize(from.img_out.size());
	for (uint8_t i = 0; i < from.img_out.size(); i++) {
		to.img_out[i].img_out_kind = static_cast<uint8_t>(from.img_out[i]);
	}
	to.dist_range.min = from.dist_range.min;
	to.dist_range.max = from.dist_range.max;
	to.fps = from.fps;
	to.thin_w = from.thin_w;
	to.thin_h = from.thin_h;
	to.crop.x = from.crop.x;
	to.crop.y = from.crop.y;
	to.light_times = from.light_times;
	to.range_calib = from.range_calib;
}

void TofCtrlNode::convertImageFormat(krm::ImageFormat& from, tof_camera_interface::msg::ImageFormat& to)
{
	to.width = from.width;
	to.height = from.height;
	to.active_start.x = from.active_start.x;
	to.active_start.y = from.active_start.y;
	to.active_w = from.active_w;
	to.active_h = from.active_h;
	to.pixels = from.pixels;
	to.bpp = from.bpp;
	to.size = static_cast<uint64_t>(from.size);
}

}	// namespace tof_camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_node::TofCtrlNode)
