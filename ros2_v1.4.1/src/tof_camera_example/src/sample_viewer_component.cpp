/*------------------------------------------------------------------*/
/// @file		sample_viewer_component.cpp
/// @brief		Sample Viewer component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cstring>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/image_encodings.hpp>

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "imgui_internal.h"

#include "sample_viewer_component.hpp"

namespace tof_camera_example
{

krm::Point2d SampleViewerNode::mouse_pos_raw_ = {UINT16_MAX, UINT16_MAX};
bool		 SampleViewerNode::mouse_enter_raw_ = false;

SampleViewerNode::SampleViewerNode(
	const rclcpp::NodeOptions& options
): Node("SampleViewerNode", options),
	cam_type_(krm::C11_USB), cam_index_(0),
	cam_mode_(0), img_out_kind_(0),
	play_dir_("."), auto_start_(true), gui_thread_(nullptr),
	frame_id_(""),
	frame_({}), recv_frm_(false), recv_notify_(false),
	recv_frm_stat_(tof_camera_interface::msg::Notify::ERR_SYSTEM),
	rcv_fps_(1U), rcv_play_time_({0, 0}),
	running_(false), win_(NULL),
	dir_browser_(DIR_BROW), file_browser_(FILE_BROW), browser_target_(FILE_BROWS_NONE),
	mode_(0), cur_mode_info_(nullptr),
	out_kind_(krm::OUT_IMG_DEPTH_IR), light_times_({0, 100U, 50U}),
	/* Panel Layout */
	win_w_(WIN_W), win_h_(WIN_H),
	pnl_ctrl_pos_(ImVec2(0, 0)), pnl_ctrl_size_(ImVec2(CTRL_WIN_W, WIN_H)),
	pnl_view_pos_(ImVec2(CTRL_WIN_W, 0)), pnl_view_size_(ImVec2(IMG_WIN_W, IMG_WIN_H)),
	pnl_sts0_pos_(ImVec2(0, 0)), pnl_sts1_pos_(ImVec2(0, 0)), pnl_sts2_pos_(ImVec2(0, 0)),
	pnl_sts0_size_(ImVec2(STS_WIN_W, STS_WIN_H)), pnl_sts2_size_(ImVec2(STS_WIN_W, STS_WIN_H)),
	pnl_msg_pos_(ImVec2(0, 0)), pnl_msg_size_(ImVec2(0, 0)),
	/* Popup messages */
	show_message_(false), show_msg_closing_(true), exit_viewer_(false),
	message_lbl_(""), message_str_(""),
	/*--- Control Panel ---*/
	ctrl_parts_w_(CTRL_PARTS_W), ctrl_node_w_(CTRL_NODE_W),
	ctrl_btn_w_(CTRL_BTN_W), ctrl_btn_idt_(CTRL_BTN_INDENT),
	ctrl_dir_btn_w_(CTRL_DIR_BTN_W), ctrl_dir_txt_w_(CTRL_DIR_TXT_W), ctrl_dir_idt_(CTRL_DIR_IDT),
	/* Target Device */
	tgt_reload_idt_(TGT_RELOAD_IDT),
	tgt_dev_open_(true), tgt_dev_enable_(true), tgt_dev_selected_(false),
	tgt_dev_sel_idx_(0), tgt_cam_idx_(0), tgt_btn_reload_(false),
	/* Device Control */
	/* Control Camera device */
	cam_rec_len_w_(CAM_REC_LEN_W),
	cap_dev_con_(false), cam_dev_open_(false), cam_cap_disable_(false),
	cam_mode_sel_(0), ini_mode_(0), cam_image_idx_(0), cam_disable_raw_(true),
	cam_light_slid_(50), cam_disable_light_time_(true), cam_light_cnt_img_(krm::IMG_DEPTH),
	cam_ae_enable_(false), cam_ae_interval_({0, 255U, 4U}),
	cam_raw_sat_th_({0, 4095U, 20U}), cam_ir_dark_th_({0, 4095U, 20U}),
	cam_ext_trg_list_({"Standalone", "Slave", "Master"}), cam_ext_trigger_(krm::EXT_TRG_STANDALONE),
	cam_ext_trg_disable_(false), cam_ext_trg_offset_(0),
	cam_int_supp_mode_list_({"Off", "Manual", "Auto"}),
	cam_int_supp_info_({krm::INT_SUPP_MODE_OFF, {0, 255U, 0}, {0, 255U, 0}, {0, 255U, 0}, {0, 7U, 0}}),
	recording_(false), cam_rec_packing_(60),
	/* PlayBack */
	play_time_w_(PLAY_TIME_W), play_time_space_(PLAY_TIME_SPACE),
	play_jump_w_(PLAY_JUMP_W), play_jump_idt_(PLAY_JUMP_IDT), play_time_idt_(PLAY_TIME_IDT),
	play_cap_disable_(false), play_pausing_(false),
	play_jump_time_sec_(10), play_jump_time_(0), play_jump_val_(0),
	play_time_(""), play_time_total_(), play_con_timer_(0),
	/* PostFilter */
	pstf_ofst_w_(PSTF_OFST_W),
	pstf_inp_w_(PSTF_INP_W),
	pstf_ksize_map_({3U, 5U}),
	pstf_ksize_list_({"3", "5"}),
	pstf_method_list_({"Differential", "Ratio"}), pstf_priority_list_({"Accuracy" ,"Speed"}),
	pstf_enable_medf_(false), pstf_medf_check_disable_(false),
	pstf_enable_bilf_(false), pstf_bilf_check_disable_(false),
	pstf_enable_flypf_(false), pstf_flypf_check_disable_(false),
	pstf_prm_({3U, 3U, 500.0, 100.0, 1.0, 3U, true, 130U, true}),
	/* Post Proccess */
	pst_pcd_ofst_w_(PST_PCD_OFST_W), pst_pcd_ofst_idt_(PST_PCD_OFST_IDT),
	pst_pcd_rot_w_(PST_PCD_ROT_W), pst_pcd_rot_idt_(PST_PCD_ROT_IDT),
	pst_dist_check_disable_(false), pst_dist_enable_(false), pst_pcd_origin_(0),
	pst_pcd_cam_coord_(true), pst_pcd_ofst_x_(0), pst_pcd_ofst_y_(0), pst_pcd_ofst_z_(0),
	pst_pcd_rot_x_(0), pst_pcd_rot_y_(0), pst_pcd_rot_z_(0),
	/* View Settings */
	view_dpt_rng_idt_(VIEW_DPT_RNG_IDT), view_dpt_w_(VIEW_DPT_W), view_dpt_btn_idt_(0),
	view_dpt_bar_w_(VIEW_DPT_BAR_W), view_dpt_bar_h_(VIEW_DPT_BAR_H),
	view_gray_idt_(VIEW_GRAY_IDT), view_gray_w_(VIEW_GRAY_W), view_pcd_grid_idt_(0),
	view_pcd_input_idt_(VIEW_PCD_INPUT_IDT), view_pcd_input_w_(VIEW_PCD_INPUT_W), view_pcd_ang_btn_(VIEW_PCD_ANG_BTN),
	view_dep_range_min_(0), view_dep_range_max_(VIEW_DEPTH_MAX),
	view_dep_range_min_max_(view_dep_range_max_ - VIEW_COLOR_DIFF_MIN),
	view_dep_range_max_min_(view_dep_range_min_ + VIEW_COLOR_DIFF_MIN),
	view_dep_min_(0), view_dep_max_(VIEW_DEPTH_MAX),
	view_depth_range_({0, VIEW_DEPTH_MAX}), view_gray_gain_(1.0F),
	view_gray_gamma_(1.0F), view_image_idx_(0), view_aux_check_(true), view_aux_grid_(1000U),
	view_radio_button_(PCD_C_DEPTH), view_disable_winsize_(false), view_radio_winsize_(DEFAULT_WIN_SIZE),
	view_intlv_enable_(true), view_intlv_distance_(0), view_intlv_factor_(2),
	/*--- Status Panel ---*/
	sts_item_idt_(STS_ITEM_IDT), sts_fps_idt_(STS_FPS_IDT),
	sts_cur_d_val_idt_(STS_CUR_D_VAL_IDT), sts_cur_ir_idt_(STS_CUR_IR_IDT), sts_cur_ir_val_idt_(STS_CUR_IR_VAL_IDT),
	sts_cur_raw_g_idt_(STS_CUR_RAW_G_IDT), sts_cur_raw_idt_(STS_CUR_RAW_IDT),
	sts_oth_idt_(STS_OTH_IDT), sts_oth_btn_w_(STS_OTH_BTN_W),
 	state_cam_(ST_CLOSED), state_rec_(ST_REC_STOP),
	stat_disable_cfg_save_(false), stat_disable_cfg_load_(false),
	hide_ctrl_panel_str_("Hide"), hide_ctrl_panel_(false),

	config_(),
	color_table_(std::make_shared<ColorTable>()), gray_table_(std::make_shared<GrayTable>()),
	/* PCD View */
	is_view_rot_changing_(false), is_view_pos_changing_(false), is_scale_changing_(false),
	grid_view_kind_(GRID_KIND_CAMERA), grid_line_w_(1.F), pcd_point_size_(1.F)
{
	bool res = false;
	DrawRaws raws;
	krm::PlayBack::PlayTime time;
	time.current = 0;
	time.total = 0;

	updatePlayingTime(time, true);

	RCLCPP_INFO(this->get_logger(), "run SampleViewerNode");

	btn_color_[0] = ImColor::HSV(3.0F / 7.0F, 0.6F, 0.6F).Value;
	btn_color_[1] = ImColor::HSV(3.0F / 7.0F, 0.7F, 0.7F).Value;
	btn_color_[2] = ImColor::HSV(3.0F / 7.0F, 0.8F, 0.8F).Value;

	grid_color_ = IM_COL32(255, 255, 255, 255);

	mode_list_.clear();
	for (auto& fmt : img_fmts_) { fmt.set(); }
	cam_rec_dir_path_c_.fill(0);
	cam_rec_dir_path_.clear();
	cam_rec_len_.fill(0);
	std::snprintf(cam_rec_len_.data(), cam_rec_len_.size(), "1");
	cam_rec_dir_path_c_.fill(0);
	play_target_path_.clear();
	for (auto& t : play_jump_) {
		t.fill(0);
		std::snprintf(t.data(), t.size(), "0");
	}
	play_target_path_c_.fill(0);

	img_win_pos_.fill(ImVec2(0,0));

	makeFpsString(UINT16_MAX, state_fps_);
	makeFpsString(UINT16_MAX, play_frame_fps_);

	cam_mode_list_.clear();
	cam_image_list_.clear();
	view_image_list_.clear();
	view_image_list_.push_back("None");
	view_image_pair_.clear();
	show_img_.fill(DISP_NONE);
	for (auto & fmt : show_fmt_) { fmt.set(); }

	draw_imgs_[krm::IMG_DEPTH] = std::make_shared<DrawDepth>(color_table_);
	draw_imgs_[krm::IMG_IR]    = std::make_shared<DrawGray>(gray_table_);
	for (uint8_t i = krm::IMG_RAW1; i <= krm::IMG_RAW4; i++) {
		draw_imgs_[i] = std::make_shared<DrawRaw>(i - krm::IMG_RAW1, gray_table_);
		raws[i - krm::IMG_RAW1] = std::dynamic_pointer_cast<DrawRaw>(draw_imgs_[i]);
	}
	draw_imgs_[IMG_PCD]   = std::make_shared<DrawPcd>(color_table_, gray_table_);
	color_table_->setColorRange(view_depth_range_);
	color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
	gray_table_->setGain(view_gray_gain_);
	gray_table_->setGamma(view_gray_gamma_);
	raw_win_ = std::make_shared<DisplayRaw>(raws);
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(view_radio_button_ != PCD_C_DEPTH);

	createTopics();
	if (loadParam()) {
		if (checkVersion()) {
			res = true;
		}
	}

	if (res) {
		RCLCPP_INFO(this->get_logger(), "wakeup SampleViewerNode");
		gui_thread_ = std::make_unique<std::thread>(&SampleViewerNode::guiThread, this);
	} else {
		RCLCPP_ERROR(this->get_logger(), "Failed to wakeup. Please shutdown this Node");
	}
}

SampleViewerNode::~SampleViewerNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit SampleViewerNode");
	if (gui_thread_) { (void)stopCapture(); }
	saveConfig();
	terminate();
}

void SampleViewerNode::createTopics(void)
{
	rclcpp::QoS qos_status(rclcpp::KeepLast(1));
	rclcpp::QoS qos_image(rclcpp::KeepLast(5));
	auto cb_notify = std::bind(&SampleViewerNode::recvNotify    , this, std::placeholders::_1);
	auto cb_frame  = std::bind(&SampleViewerNode::recvFrameData , this, std::placeholders::_1);

	qos_status = qos_status.reliable().durability_volatile();
	qos_image = qos_image.reliable().durability_volatile();

	clt_tof_ctrl_               = create_client<tof_camera_interface::srv::TofCtrl            >("krm/tof_ctrl");
	clt_get_dev_list_           = create_client<tof_camera_interface::srv::GetDevList         >("krm/get_dev_list");
	clt_open_dev_               = create_client<tof_camera_interface::srv::OpenDev            >("krm/open_dev");
	clt_close_dev_              = create_client<tof_camera_interface::srv::CloseDev           >("krm/close_dev");

	clt_set_ext_trigger_type_   = create_client<tof_camera_interface::srv::SetExtTriggerType  >("krm/set_ext_trigger_type");
	clt_set_ext_trigger_offset_ = create_client<tof_camera_interface::srv::SetExtTriggerOffset>("krm/set_ext_trigger_offset");
	clt_set_mode_               = create_client<tof_camera_interface::srv::SetMode            >("krm/set_mode");
	clt_set_img_kinds_          = create_client<tof_camera_interface::srv::SetImgKinds        >("krm/set_img_kinds");
	clt_set_light_times_        = create_client<tof_camera_interface::srv::SetLightTimes      >("krm/set_light_times");
	clt_set_ae_state_           = create_client<tof_camera_interface::srv::SetAEState         >("krm/set_ae_state");
	clt_set_ae_interval_        = create_client<tof_camera_interface::srv::SetAEInterval      >("krm/set_ae_interval");
	clt_set_int_supp_           = create_client<tof_camera_interface::srv::SetIntSupp         >("krm/set_int_supp");
	clt_set_ir_dark_threshold_  = create_client<tof_camera_interface::srv::SetIrDarkThreshold >("krm/set_ir_dark_threshold");
	clt_set_raw_sat_threshold_  = create_client<tof_camera_interface::srv::SetRawSatThreshold >("krm/set_raw_sat_threshold");
	clt_set_play_target_        = create_client<tof_camera_interface::srv::SetPlayTarget      >("krm/set_play_target");
	clt_set_play_ctrl_          = create_client<tof_camera_interface::srv::SetPlayCtrl        >("krm/set_play_ctrl");
	clt_get_dev_info_           = create_client<tof_camera_interface::srv::GetDevInfo         >("krm/get_dev_info");
	clt_get_fov_                = create_client<tof_camera_interface::srv::GetFov             >("krm/get_fov");
	clt_get_post_filt_info_     = create_client<tof_camera_interface::srv::GetPostFiltInfo    >("krm/get_post_filt_info");
	clt_get_ext_trigger_type_   = create_client<tof_camera_interface::srv::GetExtTriggerType  >("krm/get_ext_trigger_type");
	clt_get_ext_trigger_offset_ = create_client<tof_camera_interface::srv::GetExtTriggerOffset>("krm/get_ext_trigger_offset");
	clt_get_lens_info_          = create_client<tof_camera_interface::srv::GetLensInfo        >("krm/get_lens_info");
	clt_get_mode_list_          = create_client<tof_camera_interface::srv::GetModeList        >("krm/get_mode_list");
	clt_get_mode_               = create_client<tof_camera_interface::srv::GetMode            >("krm/get_mode");
	clt_get_img_kinds_          = create_client<tof_camera_interface::srv::GetImgKinds        >("krm/get_img_kinds");
	clt_get_light_times_        = create_client<tof_camera_interface::srv::GetLightTimes      >("krm/get_light_times");
	clt_get_ae_state_           = create_client<tof_camera_interface::srv::GetAEState         >("krm/get_ae_state");
	clt_get_ae_interval_        = create_client<tof_camera_interface::srv::GetAEInterval      >("krm/get_ae_interval");
	clt_get_raw_sat_th_         = create_client<tof_camera_interface::srv::GetRawSatThreshold >("krm/get_raw_sat_threshold");
	clt_get_ir_dark_th_         = create_client<tof_camera_interface::srv::GetIrDarkThreshold >("krm/get_ir_dark_threshold");
	clt_get_int_supp_info_      = create_client<tof_camera_interface::srv::GetIntSuppInfo     >("krm/get_int_supp_info");
	clt_get_play_target_        = create_client<tof_camera_interface::srv::GetPlayTarget      >("krm/get_play_target");
	clt_get_play_time_          = create_client<tof_camera_interface::srv::GetPlayTime        >("krm/get_play_time");
	clt_get_play_status_        = create_client<tof_camera_interface::srv::GetPlayStatus      >("krm/get_play_status");
	clt_get_img_format_         = create_client<tof_camera_interface::srv::GetImgFormat       >("krm/get_img_format");
	clt_psbl_post_filt_         = create_client<tof_camera_interface::srv::PsblPostFilt       >("krm/psbl_post_filt");
	clt_set_post_filt_          = create_client<tof_camera_interface::srv::SetPostFilt        >("krm/set_post_filt");
	clt_set_post_filt_prm_      = create_client<tof_camera_interface::srv::SetPostFiltPrm     >("krm/set_post_filt_prm");
	clt_psbl_lens_conv_         = create_client<tof_camera_interface::srv::PsblLensConv       >("krm/psbl_lens_conv");
	clt_set_lens_conv_          = create_client<tof_camera_interface::srv::SetLensConv        >("krm/set_lens_conv");
	clt_set_pcd_color_          = create_client<tof_camera_interface::srv::SetPcdColor        >("krm/set_pcd_color");
	clt_set_pcd_pos_            = create_client<tof_camera_interface::srv::SetPcdPos          >("krm/set_pcd_pos");
	clt_set_rec_ctrl_           = create_client<tof_camera_interface::srv::RecordCtrl         >("krm/record_ctrl");

	sub_notify_     = create_subscription<tof_camera_interface::msg::Notify    >("krm/notify"  , qos_status, cb_notify);
	sub_frame_data_ = create_subscription<tof_camera_interface::msg::FrameData >("krm/lens_out", qos_image , cb_frame);
}

bool SampleViewerNode::loadParam(void)
{
	std::string config_path, font_path;

	declare_parameter("config_path", ".");
	declare_parameter("font_path", ".");
	get_parameter("config_path", config_path);
	get_parameter("font_path", font_path);
	cfg_path_ = std::filesystem::path(config_path);
	font_path_ = std::filesystem::path(font_path);

	RCLCPP_INFO(this->get_logger(), "config_path : %s", cfg_path_.string().c_str());
	RCLCPP_INFO(this->get_logger(), "font_path : %s", font_path_.string().c_str());

	return true;
}

bool SampleViewerNode::checkVersion(void)
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

bool SampleViewerNode::initialize(void)
{
	glfwSetErrorCallback(SampleViewerNode::gl_error);
	if (glfwInit() == GL_FALSE) { return false; }
	running_ = createWindow();
	return running_;
}

void SampleViewerNode::terminate(void)
{
	glfwSetErrorCallback(NULL);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	destroyWindow();
	glfwTerminate();
	if (gui_thread_ != nullptr) { gui_thread_.reset(nullptr); }
}

void SampleViewerNode::gl_error(int error, const char* description)
{
	(void)error;
	(void)description;
}

bool SampleViewerNode::init_device(void)
{
	krm::PlayBack::ConfigParam param;
	param.path = play_target_path_;

	reloadDevice();
	return true;
}

void SampleViewerNode::reloadDevice(void)
{
	dev_list_.clear();
	dev_list_.push_back("Please select target");
	cam_list_.clear();

	if (getDevList(tof_camera_interface::msg::CameraType::C11_USB, cam_list_)) {
		for (auto dev : cam_list_) {
			dev_list_.push_back(std::to_string(dev.id) + " : " + dev.name);
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "Camera::getDeviceList : %d", clt_res_.result);
	}
	if (getDevList(tof_camera_interface::msg::CameraType::PLAYBACK, play_list_)) {
		dev_list_.push_back(play_list_[0].name);
	} else {
		RCLCPP_ERROR(this->get_logger(), "PlayBack::getDeviceList : %d", clt_res_.result);
	}
}

bool SampleViewerNode::openDevice(void)
{
	bool world_coord = !pst_pcd_cam_coord_;
	uint8_t pcd_color = (view_radio_button_ == PCD_C_DEPTH) ?
						tof_camera_interface::srv::SetPcdColor::Request::PCD_COLOR_NONE :
						tof_camera_interface::srv::SetPcdColor::Request::PCD_COLOR_IR;
	uint8_t cam_type;
	uint16_t dev_id;
	auto request = std::make_shared<tof_camera_interface::srv::OpenDev::Request>();

	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		dev_id = cam_list_[tgt_cam_idx_].id;
		cam_type = tof_camera_interface::msg::CameraType::C11_USB;
	} else {
		dev_id = play_list_[0].id;
		cam_type = tof_camera_interface::msg::CameraType::PLAYBACK;
	}

	if (openDev(cam_type, dev_id)) {
		updateDeviceInfo();
		if (tgt_cam_idx_ < PLAYBACK_INDEX) {
			cam_cap_disable_ = false;
			cap_dev_con_ = false;
			// set initial motion mode
			if (ini_mode_ != mode_) {
				for (uint8_t i = 0; i < mode_list_.size(); i++) {
					if (ini_mode_ == mode_list_[i].id) {
						if (setMode(ini_mode_)) {
							updateModeInfo();
						}
						break;
					}
				}
				ini_mode_ = mode_;
			}
		} else {
			play_cap_disable_ = false;
			play_pausing_ = false;
			state_cam_ = ST_STOPPED;
			if(!play_target_path_.empty() && setPlayTarget(play_target_path_.string<char>())) {
				updateDeviceInfo();
			}
		}
		view_disable_winsize_ = true;
		// notify current settings
		if (!pst_dist_check_disable_) {
			setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_CONV_DIST, pst_dist_enable_);
		}
		setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_PCD_KIND, world_coord);
		notifyPcdPrm();
		setPcdColor(pcd_color);
		if (!pstf_medf_check_disable_) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF, pstf_enable_medf_);
		}
		if (!pstf_bilf_check_disable_) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF, pstf_enable_bilf_);
		}
		if (!pstf_flypf_check_disable_) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF, pstf_enable_flypf_);
		}
		setPostFiltPrm(pstf_prm_);
	} else {
		RCLCPP_ERROR(this->get_logger(), "failed to openDevice : %d", clt_res_.result);
		return false;
	}

	return true;
}

void SampleViewerNode::closeDevice(void)
{
	if (state_cam_ != ST_STOPPED) { stopCapture(); }
	if (recording_) { stopRecord(); }
	if (!closeDev()) {
		RCLCPP_ERROR(this->get_logger(), "Failed to close device : %d", clt_res_.result);
	}
	tgt_dev_enable_ = true;
	tgt_dev_selected_ = false;
	cam_dev_open_ = false;
	state_cam_ = ST_CLOSED;
	state_rec_ = ST_REC_STOP;
	pst_dist_check_disable_ = false;
	pstf_medf_check_disable_ = false;
	pstf_bilf_check_disable_ = false;
	pstf_flypf_check_disable_ = false;
	tgt_dev_sel_idx_ = 0;
	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		ini_mode_ = mode_;
	}
	updateColorRange({0, VIEW_DEPTH_MAX});
	view_disable_winsize_ = false;
	raw_win_->destroyWin();
	show_img_.fill(DISP_NONE);
	view_image_list_.clear();
	view_image_idx_ = 0;
	view_image_pair_.clear();
	view_image_list_.push_back("None");
	cam_disable_raw_ = true;
	for (auto& fmt : img_fmts_) { fmt.set(); }
	for (auto& fmt : show_fmt_) { fmt.set(); }
}

void SampleViewerNode::startCapture(void)
{
	if (!setTofCtrl(tof_camera_interface::srv::TofCtrl::Request::CMD_START)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to start capture : %d", clt_res_.result);
		showMessage(MSG_WARN, "Failed to Start Capture.");
	} else {
		if (tgt_cam_idx_ < PLAYBACK_INDEX) {
			cam_cap_disable_ = true;
			cap_dev_con_ = true;
		} else {
			play_cap_disable_ = true;
			updatePlayInfo();
		}
		stat_disable_cfg_load_ = true;
		state_cam_ = ST_STREAMING;
	}
}

void SampleViewerNode::stopCapture(void)
{
	bool recv_stop = false;
	uint32_t sleep_time = 500000U / cur_mode_info_->fps;

	if (!setTofCtrl(tof_camera_interface::srv::TofCtrl::Request::CMD_STOP)) {
		RCLCPP_ERROR(this->get_logger(), "Failed to stop capture : %d", clt_res_.result);
		showMessage(MSG_WARN, "Failed to Stop Capture.\n");
	}

	for (uint8_t i = 0; i < 5U; i++) {
		std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
		recv_stop = recv_stop_;
		if (recv_stop) {
			break;
		}
	}

	if (!recv_stop) {
		ImVec2 pos = pnl_view_pos_;
		krm::DeviceInfo dev_info;

		for (uint8_t i = 0; i < 6U; i++) {
			getDevInfo(dev_info);
		}
		th_mtx_.lock();
		rcv_fps_ = UINT16_MAX;
		rcv_play_time_.current = 0;
		recv_stop_ = true;
		th_mtx_.unlock();

		raw_win_->clear();
		raw_win_->draw();
		glfwMakeContextCurrent(win_);
		for (auto& draw : draw_imgs_) {
			draw->clear();
		}
		for (uint8_t i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
			if (show_img_[i] == DISP_PCD) {
				updatePointCloud(pos, true);
			}
			pos.x += pnl_view_size_.x;
		}
		draw_points_.clear();
		if (tgt_cam_idx_ == PLAYBACK_INDEX) {
			updatePlayingTime(rcv_play_time_);
		}
	}

	if (recording_) { stopRecord(); }

	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		cam_cap_disable_ = false;
		cap_dev_con_ = false;
	} else {
		play_cap_disable_ = false;
		updatePlayInfo();
	}

	makeFpsString(UINT16_MAX, state_fps_);
	stat_disable_cfg_load_ = hide_ctrl_panel_;

	state_cam_ = ST_STOPPED;
}

void SampleViewerNode::updateDeviceInfo(void)
{
	krm::DeviceInfo dev_info;
	krm::CamFov fov;
	bool cam_med_filt, cam_bil_filt, cam_flyp_filt;
	krm::LensInfo lens_info;
	std::stringstream str_st;
	uint8_t ld_num = 0;
	uint16_t ld_enables;
	uint8_t trg_offset;
	bool is_possible_dist;
	bool is_possible_medf;
	bool is_possible_bilf;
	bool is_possible_flypf;

	cur_mode_info_ = nullptr;
	mode_list_.clear();
	cam_mode_list_.clear();

	if (getDevInfo(dev_info)) {
		str_st << std::hex << dev_info.hw_kind;
		cam_inf_list_[0] = str_st.str();
		cam_inf_list_[1] = std::to_string(dev_info.serial_no);
		cam_inf_list_[2] = makeVerString(dev_info.map_ver);
		cam_inf_list_[3] = std::to_string(dev_info.adjust_no);
		cam_inf_list_[4] = makeVerString(dev_info.firm_ver);
		cam_inf_list_[5] = std::to_string(dev_info.ld_wave) + " (nm)";
		ld_enables = dev_info.ld_enable;
		str_st.str("");
		str_st << std::hex << dev_info.ld_enable;
		for (uint8_t i = 0; i < 16U; i++) {
			if ((ld_enables & 0x0001U) != 0) {
				ld_num++;
			}
			ld_enables >>= 1U;
		}
		cam_inf_list_[6] = std::to_string(ld_num) + "(" + str_st.str() + ")";
		cam_inf_list_[7] = std::to_string(dev_info.correct_calib);
	}
	if (getCamFov(fov)) {
		cam_inf_list_[8] = makeDegString(fov.horz);
		cam_inf_list_[9] = makeDegString(fov.vert);
	}
	if (getPostFiltInfo(cam_med_filt, cam_bil_filt, cam_flyp_filt)) {
		cam_inf_list_[10] = std::to_string(cam_med_filt);
		cam_inf_list_[11] = std::to_string(cam_bil_filt);
		cam_inf_list_[12] = std::to_string(cam_flyp_filt);
	}
	if (getLensInfo(lens_info)) {
		cam_inf_list_[13] = std::to_string(lens_info.cam_dist);
		cam_inf_list_[14] = std::to_string(lens_info.lens_calib);
	}
	cam_ext_trg_disable_ = false;
	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		uint8_t ext_trg_type;
		if (getExtTriggerType(ext_trg_type)) {
			cam_ext_trigger_ = static_cast<krm::ExtTriggerType>(ext_trg_type);
			cam_ext_trg_disable_ = (cam_ext_trigger_ == krm::EXT_TRG_SLAVE);
		}
		if (getExtTriggerOffset(trg_offset)) {
			cam_ext_trg_offset_ = static_cast<float>(trg_offset) / 10.0F;
		}
	}

	pstf_medf_check_disable_ = false;
	pstf_bilf_check_disable_ = false;
	pstf_flypf_check_disable_ = false;
	pst_dist_check_disable_ = false;

	(void)psblLensConv(tof_camera_interface::srv::PsblLensConv::Request::LENS_CONV_DIST, is_possible_dist);
	pst_dist_check_disable_ = !is_possible_dist;
	if (pst_dist_check_disable_) {
		pst_dist_enable_ = true;
		pstf_flypf_check_disable_ = true;
		pstf_bilf_check_disable_ = true;
		pstf_medf_check_disable_ = true;
	} else {
		setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_CONV_DIST, pst_dist_enable_);
	}

	(void)psblPostFilt(tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_FLYF, is_possible_flypf);
	if (pstf_flypf_check_disable_) {
		pstf_enable_flypf_ = !is_possible_flypf;
		if (is_possible_flypf) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF, pstf_enable_flypf_);
		}
	} else {
		pstf_flypf_check_disable_ = !is_possible_flypf;
		if (pstf_flypf_check_disable_) {
			pstf_enable_flypf_ = true;
			pstf_bilf_check_disable_ = true;
			pstf_medf_check_disable_ = true;
		} else {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF, pstf_enable_flypf_);
		}
	}
	(void)psblPostFilt(tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_BILF, is_possible_bilf);
	if (pstf_bilf_check_disable_) {
		pstf_enable_bilf_ = !is_possible_bilf;
		if (is_possible_bilf) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF, pstf_enable_bilf_);
		}
	} else {
		pstf_bilf_check_disable_ = !is_possible_bilf;
		if (pstf_bilf_check_disable_) {
			pstf_enable_bilf_ = true;
			pstf_medf_check_disable_ = true;
		} else {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF, pstf_enable_bilf_);
		}
	}
	(void)psblPostFilt(tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_MEDF, is_possible_medf);
	if (pstf_medf_check_disable_) {
		pstf_enable_medf_ = !is_possible_medf;
		if (is_possible_medf) {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF, pstf_enable_medf_);
		}
	} else {
		pstf_medf_check_disable_ = !is_possible_medf;
		if (pstf_medf_check_disable_) {
			pstf_enable_medf_ = true;
		} else {
			setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF, pstf_enable_medf_);
		}
	}

	if (getModeList(mode_list_)) {
		if (mode_list_.size() > 0) {
			cam_mode_list_.resize(mode_list_.size());
			for (uint8_t i = 0; i < mode_list_.size(); i++) {
				cam_mode_list_[i] = makeModeString(mode_list_[i]);
			}
			updateModeInfo();
		}
	}

	if (fov.horz > 18000U) { grid_view_kind_ = GRID_KIND_FORCE_ORIGIN; }
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setFov(fov.horz);
}

void SampleViewerNode::updateModeInfo(void)
{
	cur_mode_info_ = nullptr;
	cam_image_list_.clear();
	cam_disable_light_time_ = true;
	light_times_.min = light_times_.max = light_times_.count = 0;
	cam_light_slid_ = 0;
	cam_ae_enable_ = false;
	cam_ae_interval_ = {0, 255U, 4U};
	if (tgt_cam_idx_ == PLAYBACK_INDEX) {
		for (auto& t : play_jump_) {
			t.fill(0);
			std::snprintf(t.data(), t.size(), "0");
		}
	}

	if (getMode(mode_)) {
		for (uint8_t i = 0; i < static_cast<uint8_t>(mode_list_.size()); i++) {
			if (mode_ == mode_list_[i].id) {
				std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
				std::array<char, 16> fps_str;
				RCLCPP_INFO(this->get_logger(), "Mode Info found : %u", mode_);
				cam_mode_sel_ = i;
				cur_mode_info_ = &mode_list_[i];
				cam_image_idx_ = 0;
				if (cur_mode_info_->img_out.size() > 1) {
					krm::ImgOutKind kind;
					if (getImgKinds(kind)) {
						auto itr = std::find(cur_mode_info_->img_out.begin(), cur_mode_info_->img_out.end(), kind);
						if (itr != cur_mode_info_->img_out.end()) {
							cam_image_idx_ = static_cast<uint16_t>(std::distance(cur_mode_info_->img_out.begin(), itr));
						}
					}
				}
				cam_inf_list_[15] = std::to_string(cur_mode_info_->dist_range.min) + " (mm)";
				cam_inf_list_[16] = std::to_string(cur_mode_info_->dist_range.max) + " (mm)";
				makeFpsString(cur_mode_info_->fps, fps_str, true);
				cam_inf_list_[17] = fps_str.data();
				cam_inf_list_[17] += " (fps)";
				cam_inf_list_[18] = "1 / " + std::to_string(cur_mode_info_->thin_w);
				cam_inf_list_[19] = "1 / " + std::to_string(cur_mode_info_->thin_h);
				cam_inf_list_[20] = "(" + std::to_string(cur_mode_info_->crop.x) + ", " + std::to_string(mode_list_[i].crop.y) + ")";
				cam_inf_list_[21] = std::to_string(cur_mode_info_->range_calib);
				cam_image_list_.resize(cur_mode_info_->img_out.size());
				for (uint8_t n = 0; n < static_cast<uint8_t>(cur_mode_info_->img_out.size()); n++) {
					cam_image_list_[n] = IMG_OUT_KIND_LIST[cur_mode_info_->img_out[n]].data();
				}

				if (cur_mode_info_->light_times) {
					if (getLightTimes(light_times_)) {
						cam_light_slid_ = static_cast<int>(light_times_.count);
						cam_disable_light_time_ = false;
					}
					if (getAEState(cam_ae_enable_)) {
						if (cam_ae_enable_) {
							RCLCPP_WARN(this->get_logger(), "AE is still enable. force disable.");
							cam_ae_enable_ = false;
							(void)setAEState(cam_ae_enable_);
						}
					}
					(void)getAEInterval(cam_ae_interval_);
				}
				updateColorRange(cur_mode_info_->dist_range);

				if (tgt_cam_idx_ < PLAYBACK_INDEX) {
					if (!getRawSatThreshold(cam_raw_sat_th_)) {
						cam_raw_sat_th_ = {0, 0, 0};
					}
					if (!getIrDarkThreshold(cam_ir_dark_th_)) {
						cam_ir_dark_th_ = {0, 0, 0};
					}
					if (!getIntSuppInfo(cam_int_supp_info_)) {
						cam_int_supp_info_ = {krm::INT_SUPP_MODE_OFF, {0, 255U, 0}, {0, 255U, 0}, {0, 255U, 0}, {0, 7U, 0}};
					}
				}

				play_jump_time_ = (static_cast<uint32_t>(play_jump_time_sec_) * cur_mode_info_->fps) / 100U;
				if (tgt_cam_idx_ == PLAYBACK_INDEX) { updatePlayInfo(); }
				for (uint8_t j = 0; j < krm::IMG_KINDS; j++) {
					draw_imgs_[j]->setAveCycle(cur_mode_info_->fps);
				}
				std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setRange(cur_mode_info_->dist_range.max);
				draw_pcd->setCenterPos(static_cast<float>(cur_mode_info_->dist_range.max) / 2.0F);
				draw_pcd->setViewPos(VIEW_TOP);
				draw_pcd->setInterleave(view_intlv_enable_, static_cast<float>(view_intlv_distance_), static_cast<uint8_t>(view_intlv_factor_));

				break;
			}
		}
	}
	updateImgKind();
}

void SampleViewerNode::updateImgKind(void)
{
	view_image_list_.clear();
	view_image_pair_.clear();
	view_image_idx_ = 0;

	if (cur_mode_info_ == nullptr) {
		RCLCPP_ERROR(this->get_logger(), "Mode information is empty");
		return;
	}

	out_kind_ = cur_mode_info_->img_out[cam_image_idx_];
	switch (out_kind_) {
	case krm::OUT_IMG_DEPTH:
		show_img_ = {DISP_DEPTH, DISP_PCD};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		if (view_radio_button_ == PCD_C_IR) {
			std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(false);
			view_radio_button_ = PCD_C_DEPTH;
		}
		cam_light_cnt_img_ = krm::IMG_DEPTH;
		break;
	case krm::OUT_IMG_IR:
		show_img_ = {DISP_IR, DISP_NONE};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_IR);
		cam_light_cnt_img_ = krm::IMG_IR;
		break;
	case krm::OUT_IMG_DEPTH_IR:
		show_img_ = {DISP_DEPTH, DISP_IR};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_IR);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_IR_PCD);
		cam_light_cnt_img_ = krm::IMG_DEPTH;
		break;
	case krm::OUT_IMG_DEPTH_IR_RAW:
		show_img_ = {DISP_DEPTH, DISP_IR};
		cam_disable_raw_ = false;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_IR);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_IR_PCD);
		cam_light_cnt_img_ = krm::IMG_RAW1;
		break;
	case krm::OUT_IMG_RAW:
		show_img_.fill(DISP_NONE);
		cam_disable_raw_ = false;
		view_image_list_.push_back("None");
		if (view_radio_button_ == PCD_C_IR) {
			std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(false);
			view_radio_button_ = PCD_C_DEPTH;
		}
		cam_light_cnt_img_ = krm::IMG_RAW1;
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "Unknown image output kind : %u", out_kind_);
		show_img_.fill(DISP_NONE);
		view_image_list_.push_back("None");
		cam_disable_raw_ = true;
		break;
	}
	updateImgFmts();
}

void SampleViewerNode::updateImgFmts(void)
{
	if (getImgFormat(img_fmts_)) {
		for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
			draw_imgs_[i]->setFormat(img_fmts_[i]);
			frame_.images[i].resize(img_fmts_[i]);
		}
		std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setFormat(img_fmts_[krm::IMG_DEPTH]);
		frame_.pcd.resize(img_fmts_[krm::IMG_DEPTH]);
		updateShowImgKind();
		if (!raw_win_->setFormat(img_fmts_)) {
			RCLCPP_ERROR(this->get_logger(), "failed to CreateWindow");
		}
		raw_win_->setMouseCb(static_cast<GLFWcursorenterfun>(&this->enterRawCursor), static_cast<GLFWcursorposfun>(&this->updateRawCursor));
		glfwMakeContextCurrent(win_);
		if (recording_) { stopRecord(); }
	}
}

void SampleViewerNode::updateShowImgKind(void)
{
	if (view_image_pair_.empty()) {
		show_img_.fill(DISP_NONE);
		show_fmt_[0].set();
		show_fmt_[1].set();
	} else {
		switch (view_image_pair_[view_image_idx_]) {
		case SHOW_PAIR_DEPTH_IR:
			show_img_ = {DISP_DEPTH, DISP_IR};
			show_fmt_[0] = img_fmts_[krm::IMG_DEPTH];
			show_fmt_[1] = img_fmts_[krm::IMG_IR];
			break;
		case SHOW_PAIR_DEPTH_PCD:
			show_img_ = {DISP_DEPTH, DISP_PCD};
			show_fmt_[0] = img_fmts_[krm::IMG_DEPTH];
			show_fmt_[1].set();
			break;
		case SHOW_PAIR_IR_PCD:
			show_img_ = {DISP_IR, DISP_PCD};
			show_fmt_[0] = img_fmts_[krm::IMG_IR];
			show_fmt_[1].set();
			break;
		case SHOW_PAIR_IR:
			show_img_ = {DISP_IR, DISP_NONE};
			show_fmt_[0] = img_fmts_[krm::IMG_IR];
			show_fmt_[1].set();
			break;
		default:
			show_img_.fill(DISP_NONE);
			show_fmt_[0].set();
			show_fmt_[1].set();
			break;
		}
	}
	if (view_radio_winsize_ == DOUBLE_WIN_SIZE) {
		for (auto& fmt : show_fmt_) {
			fmt.set(fmt.width * 2U, fmt.height * 2U, fmt.bpp);
		}
	}
}

void SampleViewerNode::updateColorRange(const krm::Range& d_range)
{
	view_depth_range_ = d_range;
	if ((view_depth_range_.max - view_depth_range_.min) < VIEW_COLOR_DIFF_MIN) {
		int max = static_cast<int>(view_depth_range_.max) + VIEW_COLOR_DIFF_MIN;
		if (max > VIEW_DEPTH_MAX) {
			view_depth_range_.max = VIEW_DEPTH_MAX;
			view_depth_range_.min = view_depth_range_.max - VIEW_COLOR_DIFF_MIN;
		} else {
			view_depth_range_.max = static_cast<uint16_t>(max);
		}
	}
	view_dep_min_ = view_dep_range_min_ = static_cast<int>(view_depth_range_.min);
	view_dep_max_ = view_dep_range_max_ = static_cast<int>(view_depth_range_.max);
	view_dep_range_min_max_ = view_dep_range_max_ - VIEW_COLOR_DIFF_MIN;
	view_dep_range_max_min_ = view_dep_range_min_ + VIEW_COLOR_DIFF_MIN;
	color_table_->setColorRange({static_cast<uint16_t>(view_dep_min_), static_cast<uint16_t>(view_dep_max_)});
	color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
}

void SampleViewerNode::updatePlayInfo(bool update_time)
{
	krm::PlayBack::PlayStatus	status;
	krm::PlayBack::PlayTime		time;

	if (getPlayStatus(status)) {
		makeFpsString(status.playing_fps, play_frame_fps_);
		switch (status.state) {
		case krm::PlayBack::STOPPED:
			play_pausing_ = false;
			state_cam_ = ST_STOPPED;
			break;
		case krm::PlayBack::PLAYING:
			play_pausing_ = false;
			state_cam_ = ST_STREAMING;
			break;
		case krm::PlayBack::PAUSE:
			play_pausing_ = true;
			state_cam_ = ST_PAUSE_PLAY;
			makeFpsString(UINT16_MAX, state_fps_);
			break;
		case krm::PlayBack::FAST:
			state_cam_ = ST_FAST_PLAY;
			break;
		case krm::PlayBack::SLOW:
			state_cam_ = ST_SLOW_PLAY;
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "Unknown state : %d\n", status.state);
			break;
		}
	}

	if (update_time) {
		if (getPlayTime(time)) {
			updatePlayingTime(time, true);
		}
	}
}

void SampleViewerNode::updatePlayingTime(const krm::PlayBack::PlayTime& time, bool update_total)
{
	std::array<std::string, 3>	cur;
	std::stringstream ss;

	makePlayTimeString(time.current, cur);
	if (update_total) { makePlayTimeString(time.total, play_time_total_); }
	if (time.total > 0) {
		play_con_timer_ = static_cast<int>(static_cast<uint64_t>(time.current) * 100U) / static_cast<uint64_t>(time.total);
	} else {
		play_con_timer_ = 0;
	}
	ss << std::setw(4) << cur[0] << " : " <<
		std::setw(2) << cur[1] << " : " <<
		std::setw(2) << cur[2] << " / " <<
		std::setw(4) << play_time_total_[0] << " : " <<
		std::setw(2) << play_time_total_[1] << " : " <<
		std::setw(2) << play_time_total_[2];

	play_time_ = ss.str();
}

bool SampleViewerNode::updateCapture(void)
{
	bool updated = false;
	bool recv_frm, recv_notify, recv_stop;
	uint8_t notify;
	uint16_t rcv_fps;
	uint32_t light_cnt;
	krm::PlayBack::PlayTime play_time;

	if (state_cam_ == ST_CLOSED) { return false; }

	th_mtx_.lock();
	recv_frm = recv_frm_;
	if (recv_frm_) {
		updated = true;
		recv_frm_ = false;
		if (tgt_cam_idx_ != PLAYBACK_INDEX) {
			light_cnt = light_cnt_[cam_light_cnt_img_];
		}
	}
	recv_notify = recv_notify_;
	if (recv_notify_) {
		updated = true;
		notify = recv_frm_stat_;
		recv_notify_ = false;
	}
	recv_stop = recv_stop_;
	recv_stop_ = false;
	rcv_fps = rcv_fps_;
	play_time = rcv_play_time_;
	th_mtx_.unlock();

	if (recv_stop) {
		ImVec2 pos = pnl_view_pos_;

		raw_win_->clear();
		raw_win_->draw();
		glfwMakeContextCurrent(win_);
		for (auto& draw : draw_imgs_) {
			draw->clear();
		}
		for (uint8_t i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
			if (show_img_[i] == DISP_PCD) {
				updatePointCloud(pos, true);
			}
			pos.x += pnl_view_size_.x;
		}
		draw_points_.clear();
		makeFpsString(rcv_fps, state_fps_);
		if (tgt_cam_idx_ == PLAYBACK_INDEX) {
			updatePlayingTime(play_time);
		}
	}
	if (!updated) { return false; }
	if (recv_notify) {
		switch (notify) {
		case tof_camera_interface::msg::Notify::REC_REACHED_EOF:
			setRecState(ST_REC_FINISH);
			showMessage(MSG_ATTENTION, "Record is finished");
			break;
		case tof_camera_interface::msg::Notify::PLAY_REACHED_EOF:
			stopCapture();
			updatePlayInfo();
			showMessage(MSG_ATTENTION, "Reached End of record file");
			break;
		case tof_camera_interface::msg::Notify::ERR_TIMEOUT:
			stopCapture();
			state_cam_ = ST_TIMEOUT;
			showMessage(MSG_CAUTION, "Time out receiving image");
			return false;
		case tof_camera_interface::msg::Notify::REC_ERR_SYSTEM:
			setRecState(ST_REC_STOP);
			RCLCPP_ERROR(this->get_logger(), "Failed to record : %d", notify);
			showMessage(MSG_CAUTION, "Failed to Record");
			break;
		default:
			RCLCPP_ERROR(this->get_logger(), "unknown : notify = %d", notify);
			showMessage(MSG_CAUTION, "Failed to receive image");
			break;
		}
	}
	if (recv_frm) {
		makeFpsString(rcv_fps, state_fps_);
		if (tgt_cam_idx_ == PLAYBACK_INDEX) {
			updatePlayingTime(play_time);
		} else {
			if (!cam_disable_light_time_) {
				cam_light_slid_ = static_cast<int>(light_cnt);
			}
		}
		return true;
	}
	return false;
}

void SampleViewerNode::notifyPcdPrm(void)
{
	PosOrgRotation pos = {
		{static_cast<int16_t>(pst_pcd_ofst_x_), static_cast<int16_t>(pst_pcd_ofst_y_), static_cast<int16_t>(pst_pcd_ofst_z_)},
		{convRadDeg(pst_pcd_rot_x_), convRadDeg(pst_pcd_rot_y_), convRadDeg(pst_pcd_rot_z_)}
	};
	setPcdPos(pos);
}

void SampleViewerNode::startRecord(void)
{
	bool ret;
	uint64_t	len;
	uint32_t	pack = static_cast<uint32_t>(cam_rec_packing_);

	if (cam_rec_dir_path_.empty()) {
		showMessage(MSG_ATTENTION, "Record directory is not exist");
		return;
	}
	len = static_cast<uint64_t>(convStrVal(cam_rec_len_.data()));
	if (len == 0) {
		showMessage(MSG_ATTENTION, "Record Length must over 0 sec");
		std::snprintf(cam_rec_len_.data(), cam_rec_len_.size(), "1");
		return;
	}

	len = (len * static_cast<uint64_t>(cur_mode_info_->fps)) / 100ULL;
	if (len > UINT32_MAX) {
		RCLCPP_WARN(this->get_logger(), "Record Length is over : len=%llu fps=%.2f",
			static_cast<unsigned long long>(len), static_cast<float>(cur_mode_info_->fps) / 100.0F);
		showMessage(MSG_ATTENTION, "Record Length is over");
		return;
	}
	pack = (pack * static_cast<uint32_t>(cur_mode_info_->fps)) / 100UL;
	if (pack > UINT16_MAX) {
		RCLCPP_WARN(this->get_logger(), "Record Packing size is over : len=%u fps=%.2f", pack, static_cast<float>(cur_mode_info_->fps) / 100.0F);
		showMessage(MSG_ATTENTION, "Record Packing size is over");
		return;
	}

	ret = setRecCtrl(tof_camera_interface::srv::RecordCtrl::Request::CMD_REC_START,
					cam_rec_dir_path_.string<char>(), static_cast<uint32_t>(len), static_cast<uint16_t>(pack),
					pst_dist_enable_, pstf_enable_medf_, pstf_enable_bilf_, pstf_enable_flypf_);

	if (!ret) {
		showMessage(MSG_WARN, "Failed to start recording");
	} else {
		setRecState(ST_REC_RECORD);
	}
	return;
}

void SampleViewerNode::stopRecord(void)
{
	if (!setRecCtrl(tof_camera_interface::srv::RecordCtrl::Request::CMD_REC_STOP)) {
		showMessage(MSG_WARN, "Failed to stop recording");
	}
	setRecState(ST_REC_STOP);
}

void SampleViewerNode::setRecState(RecState state)
{
	state_rec_ = state;
	switch (state) {
	case ST_REC_STOP:
	case ST_REC_FINISH:
	case ST_REC_FULL:
		recording_ = false;
		if (!isCapturing()) {
			stat_disable_cfg_load_ = false;
		}
		stat_disable_cfg_save_ = false;
		break;
	case ST_REC_RECORD:
		recording_ = true;
		stat_disable_cfg_save_ = true;
		stat_disable_cfg_load_ = true;
		break;
	default:
		break;
	}
}

void SampleViewerNode::loadConfig(void)
{
	ViewerConfigData cfg;

	if (config_.loadConfig(cfg_path_, cfg) != krm::SUCCESS) {
		showMessage(MSG_ATTENTION, "Viewer configuration is not exist");
	}
	ini_mode_ = cfg.camera.mode;

	cam_rec_dir_path_ = cfg.camera.record.target_path;
	(void)convToCharPath(cam_rec_dir_path_, cam_rec_dir_path_c_);
	cam_rec_packing_ = cfg.camera.record.packing;

	play_target_path_ = cfg.playback.target_path;
	(void)convToCharPath(play_target_path_, play_target_path_c_);
	play_jump_time_sec_ = cfg.playback.jump_time;

	pst_dist_enable_ = cfg.post.distortion;
	pst_pcd_origin_ = static_cast<int>(cfg.post.point_cloud.pcd_coord);
	pst_pcd_cam_coord_ = (pst_pcd_origin_ == 0);
	if (cfg.post.point_cloud.origin_offset.size() == 3) {
		pst_pcd_ofst_x_ = cfg.post.point_cloud.origin_offset[0];
		pst_pcd_ofst_y_ = cfg.post.point_cloud.origin_offset[1];
		pst_pcd_ofst_z_ = cfg.post.point_cloud.origin_offset[2];
	} else {
		pst_pcd_ofst_x_ = 0;
		pst_pcd_ofst_y_ = 0;
		pst_pcd_ofst_z_ = 0;
	}
	if (cfg.post.point_cloud.rotation.size() == 3) {
		pst_pcd_rot_x_ = convDegRad(cfg.post.point_cloud.rotation[0]);
		pst_pcd_rot_y_ = convDegRad(cfg.post.point_cloud.rotation[1]);
		pst_pcd_rot_z_ = convDegRad(cfg.post.point_cloud.rotation[2]);
	} else {
		pst_pcd_rot_x_ = 0;
		pst_pcd_rot_y_ = 0;
		pst_pcd_rot_z_ = 0;
	}

	view_gray_gain_ = cfg.view.gain;
	view_gray_gamma_ = cfg.view.gamma;
	gray_table_->setGain(view_gray_gain_);
	gray_table_->setGamma(view_gray_gamma_);

	if (grid_view_kind_ < GRID_KIND_FORCE_ORIGIN) {
		grid_view_kind_ = (pst_pcd_origin_ == 0) ? GRID_KIND_CAMERA : GRID_KIND_ORIGIN;
	}

	pstf_enable_medf_ = cfg.postfilter.median.enable;
	pstf_prm_.median_ksize = cfg.postfilter.median.ksize;
	pstf_enable_bilf_ = cfg.postfilter.bil.enable;
	pstf_prm_.bil_ksize = cfg.postfilter.bil.ksize;
	pstf_prm_.bil_sigma_depth = cfg.postfilter.bil.sigma_depth;
	pstf_prm_.bil_sigma_ir = cfg.postfilter.bil.sigma_ir;
	pstf_prm_.bil_sigma_space = cfg.postfilter.bil.sigma_space;
	pstf_enable_flypf_ = cfg.postfilter.flyp.enable;
	pstf_prm_.flyp_ksize = cfg.postfilter.flyp.ksize;
	pstf_prm_.flyp_log = cfg.postfilter.flyp.log_eval;
	pstf_prm_.flyp_thr = cfg.postfilter.flyp.threshold;
	pstf_prm_.flyp_fast_proc = cfg.postfilter.flyp.fast_proc;
}

void SampleViewerNode::saveConfig(void)
{
	ViewerConfigData cfg;

	cfg.camera.mode = ini_mode_;
	cfg.camera.record.target_path = cam_rec_dir_path_;
	cfg.camera.record.packing = cam_rec_packing_;
	cfg.playback.target_path = play_target_path_;
	cfg.playback.jump_time = play_jump_time_sec_;
	cfg.post.distortion = pst_dist_enable_;
	cfg.post.point_cloud.pcd_coord = static_cast<uint8_t>(pst_pcd_origin_);

	cfg.post.point_cloud.origin_offset.resize(3);
	cfg.post.point_cloud.origin_offset[0] = pst_pcd_ofst_x_;
	cfg.post.point_cloud.origin_offset[1] = pst_pcd_ofst_y_;
	cfg.post.point_cloud.origin_offset[2] = pst_pcd_ofst_z_;
	cfg.post.point_cloud.rotation.resize(3);
	cfg.post.point_cloud.rotation[0] = convRadDeg(pst_pcd_rot_x_);
	cfg.post.point_cloud.rotation[1] = convRadDeg(pst_pcd_rot_y_);
	cfg.post.point_cloud.rotation[2] = convRadDeg(pst_pcd_rot_z_);

	cfg.postfilter.median.enable = pstf_enable_medf_;
	cfg.postfilter.median.ksize = pstf_prm_.median_ksize;
	cfg.postfilter.bil.enable = pstf_enable_bilf_;
	cfg.postfilter.bil.ksize = pstf_prm_.bil_ksize;
	cfg.postfilter.bil.sigma_depth = pstf_prm_.bil_sigma_depth;
	cfg.postfilter.bil.sigma_ir = pstf_prm_.bil_sigma_ir;
	cfg.postfilter.bil.sigma_space = pstf_prm_.bil_sigma_space;
	cfg.postfilter.flyp.enable = pstf_enable_flypf_;
	cfg.postfilter.flyp.ksize = pstf_prm_.flyp_ksize;
	cfg.postfilter.flyp.log_eval = pstf_prm_.flyp_log;
	cfg.postfilter.flyp.threshold = pstf_prm_.flyp_thr;
	cfg.postfilter.flyp.fast_proc = pstf_prm_.flyp_fast_proc;

	cfg.view.gain = view_gray_gain_;
	cfg.view.gamma = view_gray_gamma_;

	if (config_.saveConfig(cfg_path_, cfg) != krm::SUCCESS) {
		showMessage(MSG_CAUTION, "Failed to save Viewer configuration");
	}
}

void SampleViewerNode::disableCfgBtn(void)
{
	stat_disable_cfg_save_ = true;
	stat_disable_cfg_load_ = true;
}

void SampleViewerNode::enableCfgBtn(void)
{
	stat_disable_cfg_save_ = recording_;
	stat_disable_cfg_load_ = !isCaptureStopped();
}

bool SampleViewerNode::createWindow(void)
{
#if defined(IMGUI_IMPL_OPENGL_ES2)
	// GL ES 2.0 + GLSL 100
	const char* glsl_version = "#version 100";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
	glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#else
	// GL 3.0 + GLSL 130
	const char* glsl_version = "#version 130";
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_MAXIMIZED, GL_FALSE);

	win_= glfwCreateWindow(win_w_, win_h_, WIN_NAME.data(), NULL, NULL);
	if (win_ == NULL) {
		RCLCPP_ERROR(this->get_logger(), "failed to CreateWindow\n");
		return false;
	}
	glfwSetInputMode(win_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	glfwMakeContextCurrent(win_);
	glfwSwapInterval(1); // Enable vsync

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	// Setup Dear ImGui style
	ImGui::StyleColorsDark();
	// Setup Platform
	ImGui_ImplGlfw_InitForOpenGL(win_, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	glGenTextures(TEXTURE_NUM, texture_);

	setFont();
	updateLayout();

	return true;
}

void SampleViewerNode::destroyWindow(void)
{
	glDeleteTextures(TEXTURE_NUM, texture_);

	if (win_ != NULL) {
		glfwDestroyWindow(win_);
		win_ = NULL;
	}
	dir_browser_.Close();
	file_browser_.Close();
}

void SampleViewerNode::setFont(void)
{
	std::error_code ec;
	std::filesystem::path path;
	bool exist;

	path = font_path_;
#ifdef LNX_FUNC
	path = JP_FONT;
#else
	path.append(JP_FONT);
#endif

	exist = std::filesystem::exists(path, ec);
	if (!ec && exist) {
		ImGuiIO& io = ImGui::GetIO();
		ImFontConfig config;
		config.MergeMode = true;
		io.Fonts->AddFontDefault();
		io.Fonts->AddFontFromFileTTF(path.string().data(), 9.0f, &config, io.Fonts->GetGlyphRangesJapanese());
	} else {
		RCLCPP_INFO(this->get_logger(), "Japanese font is not installed : %s", path.string().data());
	}
}

bool SampleViewerNode::drawButton(const std::string& str, float width, bool disable)
{
	bool ret = false;

	if (disable) { ImGui::BeginDisabled(); }

	ImGui::PushStyleColor(ImGuiCol_Button, btn_color_[0]);
	ImGui::PushStyleColor(ImGuiCol_ButtonHovered, btn_color_[1]);
	ImGui::PushStyleColor(ImGuiCol_ButtonActive, btn_color_[2]);

	if (ImGui::Button(str.c_str(), ImVec2(width, 0))) {
		ret = true;
	}

	ImGui::PopStyleColor(3);

	if (disable) { ImGui::EndDisabled(); }

	return ret;
}

bool SampleViewerNode::drawCombo(const std::string& label, const std::vector<std::string>& list, uint16_t& index, bool disable)
{
	bool update = false;
	bool selected = false;

	if (disable) { ImGui::BeginDisabled(); }

	if (ImGui::BeginCombo(label.c_str(), list[index].c_str(), 0)) {
		for (uint16_t i = 0; i < static_cast<uint16_t>(list.size()); i++) {
			selected = (index == i);
			if (ImGui::Selectable(list[i].c_str(), selected)) {
				index = i;
				update = true;
			}
			if (selected) {
				ImGui::SetItemDefaultFocus();
			}
		}
		ImGui::EndCombo();
	}

	if (disable) { ImGui::EndDisabled(); }

	return update;
}

bool SampleViewerNode::convToCharPath(const std::filesystem::path& path, std::array<char, MAX_PATH_LEN>& c_path)
{
#ifdef LNX_FUNC
	std::string str = path.string<char>();
#else	/* LNX_FUNC */
#ifdef __cpp_lib_char8_t
	std::u8string u8_str = path.u8string();
	std::string str = std::string(u8_str.begin(), u8_str.end());
#else	/* __cpp_lib_char8_t */
	std::string str = path.u8string();
#endif	/* __cpp_lib_char8_t */
#endif	/* LNX_FUNC */
	if (str.size() >= (MAX_PATH_LEN - 1U)) {
		return false;
	}
	c_path.fill(0);
	std::memcpy(c_path.data(), str.data(), str.size());
	return true;
}

void SampleViewerNode::convToFsPath(const std::array<char, MAX_PATH_LEN>& c_path, std::filesystem::path& path)
{
	if (std::strncmp(c_path.data(), "", c_path.size()) == 0) {
		path = std::filesystem::current_path();
	} else {
#ifdef LNX_FUNC
		std::string str(c_path.data());
		path.assign(str);
#else	/* LNX_FUNC */
		path = std::filesystem::u8path(c_path.data());
#endif	/* LNX_FUNC */
	}
}

void SampleViewerNode::drawPopUpMessage(void)
{
	if (show_message_) {
		ImGui::SetNextWindowSize(pnl_msg_size_);
		ImGui::SetNextWindowPos(pnl_msg_pos_);
		ImGui::OpenPopup(message_lbl_.c_str());
		if (ImGui::BeginPopupModal(message_lbl_.c_str(), NULL, ImGuiWindowFlags_AlwaysAutoResize))
		{
			ImGui::Text("%s", message_str_.c_str());

			if (show_msg_closing_) {
				if (drawButton("Close", ctrl_btn_w_)) {
					ImGui::CloseCurrentPopup();
					show_message_ = false;
					if (exit_viewer_) {
						running_ = false;
					}
				}
			}
			ImGui::EndPopup();
		}
	}
}

void SampleViewerNode::updateLayout(void)
{
	float scale = 1.F;
	uint16_t scale_u = 1U;
	int ctrl_w;
	int img_w = IMG_WIN_W, img_h = IMG_WIN_H;
	int sts_w, sts_h = STS_WIN_H;
	int file_win_x = FILE_WIN_X;
	int file_win_y = FILE_WIN_Y;
	int file_win_w = FILE_WIN_W;
	int file_win_h = FILE_WIN_H;
	std::string brow_str = (view_radio_winsize_ == DEFAULT_WIN_SIZE) ? "Default" : "Double";
	ImGuiIO& io = ImGui::GetIO();

	/* initial value */
	io.FontGlobalScale = 1.F;
	if (hide_ctrl_panel_) {
		win_w_ = WIN_W - CTRL_WIN_W;
		ctrl_w = 0;
	} else {
		win_w_ = WIN_W;
		ctrl_w = CTRL_WIN_W;
	}
	win_h_ = WIN_H;
	ctrl_parts_w_ = CTRL_PARTS_W;
	ctrl_btn_w_ = CTRL_BTN_W;
	ctrl_dir_btn_w_ = CTRL_DIR_BTN_W;
	cam_rec_len_w_ = CAM_REC_LEN_W;
	play_time_w_ = PLAY_TIME_W;
	play_time_space_ = PLAY_TIME_SPACE;
	play_jump_w_ = PLAY_JUMP_W;
	play_time_idt_ = PLAY_TIME_IDT;
	pst_pcd_ofst_w_ = PST_PCD_OFST_W;
	pst_pcd_ofst_idt_ = PST_PCD_OFST_IDT;
	pst_pcd_rot_w_ = PST_PCD_ROT_W;
	pst_pcd_rot_idt_ = PST_PCD_ROT_IDT;
	pstf_ofst_w_ = PSTF_OFST_W;
	pstf_inp_w_ = PSTF_INP_W;
	view_dpt_w_ = VIEW_DPT_W;
	view_dpt_bar_h_ = VIEW_DPT_BAR_H;
	view_gray_idt_ = VIEW_GRAY_IDT;
	view_gray_w_ = VIEW_GRAY_W;
	view_pcd_input_idt_ = VIEW_PCD_INPUT_IDT;
	view_pcd_input_w_ = VIEW_PCD_INPUT_W;
	view_pcd_ang_btn_ = VIEW_PCD_ANG_BTN;
	sts_item_idt_ = STS_ITEM_IDT;
	sts_fps_idt_ = STS_FPS_IDT;
	sts_cur_d_val_idt_ = STS_CUR_D_VAL_IDT;
	sts_cur_ir_idt_ = STS_CUR_IR_IDT;
	sts_cur_ir_val_idt_ = STS_CUR_IR_VAL_IDT;
	sts_cur_raw_g_idt_ = STS_CUR_RAW_G_IDT;
	sts_cur_raw_idt_ = STS_CUR_RAW_IDT;
	sts_oth_idt_ = STS_OTH_IDT;
	sts_oth_btn_w_ = STS_OTH_BTN_W;

	if (view_radio_winsize_ == DOUBLE_WIN_SIZE) {
		scale = 2.F;
	}
	scale_u = static_cast<uint16_t>(scale);
	if (view_radio_winsize_ != DEFAULT_WIN_SIZE) {
		io.FontGlobalScale = scale;
		win_w_ *= scale_u;
		win_h_ *= scale_u;
		ctrl_w *= scale_u;
		img_w *= scale_u;
		img_h *= scale_u;
		sts_h *= scale_u;
		file_win_w *= scale_u;
		file_win_h *= scale_u;
		ctrl_parts_w_ = ctrl_w - UI_HEADER_W;
		ctrl_btn_w_ *= scale;
		ctrl_dir_btn_w_ *= scale;
		cam_rec_len_w_ *= scale;
		play_time_w_ *= scale;
		play_time_space_ *= scale;
		play_jump_w_ *= scale;
		play_time_idt_ *= scale;
		pst_pcd_ofst_w_ *= scale;
		pst_pcd_ofst_idt_ *= scale;
		pst_pcd_rot_w_ *= scale;
		pst_pcd_rot_idt_ *= scale;
		pstf_ofst_w_ *= scale;
		pstf_inp_w_ *= scale;
		view_dpt_w_ *= scale;
		view_dpt_bar_h_ *= scale_u;
		view_gray_idt_ *= scale;
		view_gray_w_ *= scale;
		view_pcd_input_idt_ *= scale;
		view_pcd_input_w_ *= scale;
		view_pcd_ang_btn_ *= scale;
		sts_item_idt_ *= scale;
		sts_fps_idt_ *= scale;
		sts_cur_d_val_idt_ *= scale;
		sts_cur_ir_idt_ *= scale;
		sts_cur_ir_val_idt_ *= scale;
		sts_cur_raw_g_idt_ *= scale;
		sts_cur_raw_idt_ *= scale;
		sts_oth_idt_ *= scale;
		sts_oth_btn_w_ *= scale;
	}

	sts_w = (img_w * 2) / 3;
	file_win_x = (win_w_ / 2) - (file_win_w / 2);
	file_win_y = (win_h_ / 2) - (file_win_h / 2);
	ctrl_node_w_ = ctrl_parts_w_ - UI_NODE_W;
	ctrl_btn_idt_ = ctrl_node_w_ - ctrl_btn_w_ + 29.F;
	ctrl_dir_txt_w_ = ctrl_node_w_ - ctrl_dir_btn_w_ - (INDENT_SPACE * scale * 2.F);
	ctrl_dir_idt_ = ctrl_dir_btn_w_ + 38.F - scale;
	tgt_reload_idt_ = ctrl_parts_w_ - ctrl_btn_w_ - 2.F;
	play_jump_idt_ = ctrl_node_w_ - play_jump_w_ + 29.F;
	view_dpt_rng_idt_ = ctrl_node_w_ - (VIEW_DPT_RNG_W * scale) + 29.F;
	view_dpt_bar_w_ = static_cast<uint16_t>(ctrl_node_w_);
	view_dpt_btn_idt_ = ctrl_node_w_ - view_dpt_w_ + 29.F;

	pnl_ctrl_size_ = ImVec2(static_cast<float>(ctrl_w), static_cast<float>(win_h_));
	pnl_view_pos_ = ImVec2(static_cast<float>(ctrl_w), 0);
	pnl_view_size_ = ImVec2(static_cast<float>(img_w), static_cast<float>(img_h));
	pnl_sts0_pos_ = ImVec2(static_cast<float>(ctrl_w), pnl_view_size_.y);
	pnl_sts1_pos_ = ImVec2(pnl_sts0_pos_.x + static_cast<float>(sts_w), pnl_view_size_.y);
	pnl_sts2_pos_ = ImVec2(pnl_sts1_pos_.x + static_cast<float>(sts_w), pnl_view_size_.y);
	pnl_sts0_size_ = ImVec2(static_cast<float>(sts_w), static_cast<float>(sts_h));
	pnl_sts2_size_ = ImVec2(static_cast<float>(img_w * 2) - static_cast<float>(sts_w * 2), static_cast<float>(sts_h));

	grid_line_w_ = scale;
	pcd_point_size_ = scale;

	glfwSetWindowSize(win_, win_w_, win_h_);
	dir_browser_.SetTitle("Directory browser##" + brow_str);
	file_browser_.SetTitle("File browser##" + brow_str);
	dir_browser_.SetWindowPos(file_win_x, file_win_y);
	file_browser_.SetWindowPos(file_win_x, file_win_y);
	dir_browser_.SetWindowSize(file_win_w, file_win_h);
	file_browser_.SetWindowSize(file_win_w, file_win_h);
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setWindow(
		pnl_view_pos_.x + pnl_view_size_.x, pnl_view_pos_.y,
		pnl_view_size_.x, pnl_view_size_.y
	);
	color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
	raw_win_->setScale(scale_u);
	for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
		draw_imgs_[i]->setScale(scale_u);
	}
}

void SampleViewerNode::drawCtrlPanel(void)
{
	ImGui::SetNextWindowPos(pnl_ctrl_pos_);
	ImGui::SetNextWindowSize(pnl_ctrl_size_);
	ImGui::Begin("Control", NULL, CTRL_WIN_FLAG);

	drawTgtDevLbl();
	drawDevCtrlLbl();
	drawPostFiltLbl();
	drawPostProcLbl();
	drawViewStgLbl();

	ImGui::End();
}

void SampleViewerNode::drawTgtDevLbl(void)
{
	if (tgt_dev_open_) { ImGui::SetNextItemOpen(tgt_dev_open_); }
	if (ImGui::CollapsingHeader("Target Device")) {
		ImGui::SetNextItemWidth(ctrl_parts_w_);
		if (drawCombo("##Target Device", dev_list_, tgt_dev_sel_idx_, tgt_dev_selected_)) {
			if (tgt_dev_sel_idx_ > 0) {	// 0 : default
				uint16_t idx = tgt_dev_sel_idx_ - 1U;
				if (idx >= static_cast<uint16_t>(cam_list_.size())) {
					tgt_cam_idx_ = PLAYBACK_INDEX;	// PlayBack
				} else {
					tgt_cam_idx_ = idx;				// Camera
				}
				if (openDevice()) {
					tgt_dev_enable_ = false;
					tgt_dev_selected_ = true;
					cam_dev_open_ = true;
					state_cam_ = ST_STOPPED;
				} else {
					showMessage(MSG_WARN, "Failed to access target device");
					tgt_dev_sel_idx_ = 0;
					tgt_cam_idx_ = 0;
				}
			}
		}

		ImGui::Indent(tgt_reload_idt_);
		if (tgt_dev_selected_) {
			if (drawButton("Close Device", ctrl_btn_w_)) {
				closeDevice();
			}
		} else {
			if (drawButton("Reload", ctrl_btn_w_)) {
				reloadDevice();
				if (cam_list_.empty()) {
					showMessage(MSG_WARN, "Camera device is not connected");
				}
			}
		}
		ImGui::Unindent(tgt_reload_idt_);
	} else {
		tgt_dev_open_ = false;
	}
}

void SampleViewerNode::drawDevCtrlLbl(void)
{
	if (cam_dev_open_) { ImGui::SetNextItemOpen(cam_dev_open_); }
	if (ImGui::CollapsingHeader("Device Control")) {
		if (tgt_dev_enable_) {
			// none
		} else if (tgt_cam_idx_ < PLAYBACK_INDEX) {
			drawDevCtrlCam();	// Control Camera device
		} else {
			drawDevCtrlPlay();	// Control PlayBack
		}
	} else {
		cam_dev_open_ = false;
	}
}

void SampleViewerNode::drawDevCtrlCam(void)
{
	if (cam_dev_open_) { ImGui::SetNextItemOpen(cam_dev_open_); }
	if (ImGui::TreeNode("Control")) {
		if (cap_dev_con_) {
			if (drawButton("Stop Capture", ctrl_btn_w_)) {
				stopCapture();
			}
		} else {
			if (drawButton("Start Capture", ctrl_btn_w_)) {
				startCapture();
			}
		}
		ImGui::TreePop();
	} else {
		cam_dev_open_ = false;
	}
	if (ImGui::TreeNode("Motion Mode")) {
		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		for (uint8_t i = 0; i < static_cast<uint8_t>(cam_mode_list_.size()); i++) {
			if (ImGui::Selectable(cam_mode_list_[i].c_str(), (cam_mode_sel_ == i))) {
				if (cam_mode_sel_ != i) {
					if (setMode(mode_list_[i].id)) {
						cam_mode_sel_ = i;
						updateModeInfo();
					} else {
						showMessage(MSG_WARN, "Failed to change Motion Mode");
					}
				}
			}
		}
		if (cam_cap_disable_) { ImGui::EndDisabled(); }
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Image Kinds")) {
		uint16_t cur_img_idx = cam_image_idx_;
		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (drawCombo("##Image Kinds", cam_image_list_, cam_image_idx_, (cam_image_list_.size() == 0))) {
			if (setImgKinds(cur_mode_info_->img_out[cam_image_idx_])) {
				updateImgKind();
			} else {
				cam_image_idx_ = cur_img_idx;
				showMessage(MSG_WARN, "Failed to change Image Kind");
			}
		}
		if (cam_cap_disable_) { ImGui::EndDisabled(); }
		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Information##device")) {
		if (ImGui::BeginTable("##information", DEV_INF_ROW, INF_TBL_FLAGS)) {
			for (uint8_t i = 0; i < CAM_INF_NUM; i++) {
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%s", CAM_INF_LIST[i].data());
				ImGui::TableNextColumn();
				ImGui::Text("%s", cam_inf_list_[i].c_str());
			}
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}

	if (cam_disable_light_time_) { ImGui::BeginDisabled(); ImGui::SetNextItemOpen(false); }
	if (ImGui::TreeNode("Light Times")) {
		int ae_interval = static_cast<int>(cam_ae_interval_.interval);
		if (ImGui::BeginTable("", 2, TBL_FLAGS)) {
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("Minimum");
			ImGui::TableNextColumn();
			ImGui::Text("%u", light_times_.min);
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("Maximum");
			ImGui::TableNextColumn();
			ImGui::Text("%u", light_times_.max);
		}
		ImGui::EndTable();

		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (cam_ae_enable_) { ImGui::BeginDisabled(); }
		if (ImGui::SliderInt("##Light Times", &cam_light_slid_, light_times_.min, light_times_.max, "%u", ImGuiSliderFlags_AlwaysClamp)) {
			light_times_.count = static_cast<uint32_t>(cam_light_slid_);
			if (setLightTimes(light_times_.count)) {
				if (getLightTimes(light_times_)) {
					cam_light_slid_ = static_cast<int>(light_times_.count);
				}
			} else {
				showMessage(MSG_WARN, "Light Times is over the range");
			}
		}
		if (cam_ae_enable_) { ImGui::EndDisabled(); }

		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		if (ImGui::Checkbox("Auto Exposure", &cam_ae_enable_)) {
			if (!setAEState(cam_ae_enable_)) {
				showMessage(MSG_WARN, "Failed to change Auto Exposure");
			}
		}

		if (!cam_ae_enable_) { ImGui::BeginDisabled(); }
		ImGui::Text("Frame Interval");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##AEInterval", &ae_interval,
			static_cast<int>(cam_ae_interval_.min), static_cast<int>(cam_ae_interval_.max),
			"%3u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_ae_interval_.interval = static_cast<uint8_t>(ae_interval);
			if (!setAEInterval(cam_ae_interval_.interval)) {
				showMessage(MSG_WARN, "Failed to change Auto Exposure Interval");
			}
		}
		if (!cam_ae_enable_) { ImGui::EndDisabled(); }
		if (cam_cap_disable_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}
	if (cam_disable_light_time_) { ImGui::EndDisabled(); }

	if (ImGui::TreeNode("Threshold")) {
		int sat_th = static_cast<int>(cam_raw_sat_th_.threshold);
		int dark_th = static_cast<int>(cam_ir_dark_th_.threshold);

		ImGui::Text("Saturation");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##RawSatThresh", &sat_th,
			static_cast<int>(cam_raw_sat_th_.min), static_cast<int>(cam_raw_sat_th_.max),
			"%4u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_raw_sat_th_.threshold = static_cast<uint16_t>(sat_th);
		}

		ImGui::Text("Dark");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IrDarkThresh", &dark_th,
			static_cast<int>(cam_ir_dark_th_.min), static_cast<int>(cam_ir_dark_th_.max),
			"%4u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_ir_dark_th_.threshold = static_cast<uint16_t>(dark_th);
		}

		if (drawButton("Set", ctrl_btn_w_)) {
			if (!setRawSatThreshold(cam_raw_sat_th_.threshold)) {
				showMessage(MSG_WARN, "Failed to change RAW Saturation threshold");
			}
			if (!setIrDarkThreshold(cam_ir_dark_th_.threshold)) {
				showMessage(MSG_WARN, "Failed to change IR Dark threshold");
			}
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("External Trigger")) {
		uint16_t ext_trigger = static_cast<uint16_t>(cam_ext_trigger_ - krm::EXT_TRG_STANDALONE);
		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		if (drawCombo("##ExtTrigger", cam_ext_trg_list_, ext_trigger, cam_ext_trg_disable_)) {
			cam_ext_trigger_ = static_cast<krm::ExtTriggerType>(ext_trigger + krm::EXT_TRG_STANDALONE);
			if (setExtTriggerType(static_cast<uint8_t>(cam_ext_trigger_))) {
				cam_ext_trg_disable_ = (cam_ext_trigger_ == krm::EXT_TRG_SLAVE);
			}
		}
		ImGui::Text("External Trigger Offset");
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		ImGui::SliderFloat("##ExtTriggerOffset", &cam_ext_trg_offset_, 0, 25.5f, "%2.1f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::SameLine();
		ImGui::Text("(us)");
		ImGui::SameLine();
		if (drawButton("Set")) {
			uint8_t offset = static_cast<uint8_t>(cam_ext_trg_offset_ * 10);
			if (!setExtTriggerOffset(offset)) {
				showMessage(MSG_WARN, "Failed to set external trigger offset");
			}
		}

		if (cam_cap_disable_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Interference Suppression")) {

		bool cam_int_supp_mode_disable_ = false;
		uint16_t mode = static_cast<uint16_t>(cam_int_supp_info_.mode);
		int val_m  = static_cast<int>(cam_int_supp_info_.prm_m.value);
		int val_a1 = static_cast<int>(cam_int_supp_info_.prm_a1.value);
		int val_a2 = static_cast<int>(cam_int_supp_info_.prm_a2.value);
		int val_a3 = static_cast<int>(cam_int_supp_info_.prm_a3.value);

		ImGui::Text("Mode");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);

		if (drawCombo("##IntSupp", cam_int_supp_mode_list_, mode, cam_int_supp_mode_disable_)) {
			cam_int_supp_info_.mode = static_cast<krm::IntSuppModeType>(mode);
		}

		ImGui::Text("Manual Param");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IntSuppManParam", &val_m,
			static_cast<int>(cam_int_supp_info_.prm_m.min),
			static_cast<int>(cam_int_supp_info_.prm_m.max),
			"%3u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_int_supp_info_.prm_m.value = static_cast<uint8_t>(val_m);
		}

		ImGui::Text("Auto Param1");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IntSuppAutoParam1", &val_a1,
			static_cast<int>(cam_int_supp_info_.prm_a1.min),
			static_cast<int>(cam_int_supp_info_.prm_a1.max),
			"%3u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_int_supp_info_.prm_a1.value = static_cast<uint8_t>(val_a1);
		}

		ImGui::Text("Auto Param2");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IntSuppAutoParam2", &val_a2,
			static_cast<int>(cam_int_supp_info_.prm_a2.min),
			static_cast<int>(cam_int_supp_info_.prm_a2.max),
			"%3u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_int_supp_info_.prm_a2.value = static_cast<uint8_t>(val_a2);
		}

		ImGui::Text("Auto Param3");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IntSuppAutoParam3", &val_a3,
			static_cast<int>(cam_int_supp_info_.prm_a3.min),
			static_cast<int>(cam_int_supp_info_.prm_a3.max),
			"%3u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_int_supp_info_.prm_a3.value = static_cast<uint8_t>(val_a3);
		}

		if (drawButton("Set", ctrl_btn_w_)) {
			if (!setIntSupp(cam_int_supp_info_)) {
				showMessage(MSG_WARN, "Failed to set interference suppression info");
			}
		}

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Record")) {
		if (drawButton("Directory", ctrl_dir_btn_w_, recording_)) {
			convToFsPath(cam_rec_dir_path_c_, cam_rec_dir_path_);
			dir_browser_.SetPwd(cam_rec_dir_path_);
			dir_browser_.ClearSelected();
			dir_browser_.Open();
			browser_target_ = FILE_BROWS_REC;
		}
		if ((browser_target_ == FILE_BROWS_REC) && dir_browser_.HasSelected()) {
			cam_rec_dir_path_ = dir_browser_.GetSelected();
			RCLCPP_INFO(this->get_logger(), "select dir=%s", cam_rec_dir_path_.string<char>().c_str());
			dir_browser_.ClearSelected();
			browser_target_ = FILE_BROWS_NONE;
			if (!convToCharPath(cam_rec_dir_path_, cam_rec_dir_path_c_)) {
				convToFsPath(cam_rec_dir_path_c_, cam_rec_dir_path_);
				RCLCPP_ERROR(this->get_logger(), "File Path is over");
			}
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
		ImGui::InputTextWithHint("##Record Directory", PATH_HINT.data(), cam_rec_dir_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);

		if (recording_) { ImGui::BeginDisabled(); }
		ImGui::Text("Length");
		ImGui::SameLine(ctrl_dir_idt_, 0);
		ImGui::SetNextItemWidth(cam_rec_len_w_);
		if (ImGui::InputText("(sec)", cam_rec_len_.data(), cam_rec_len_.size(), TEXT_DEC_FLAG)) {
			uint32_t val = convStrVal(cam_rec_len_.data());
			if (val == 0) {
				cam_rec_len_.fill(0);
				std::snprintf(cam_rec_len_.data(), cam_rec_len_.size(), "1");
			} else if (val == UINT32_MAX) {
				cam_rec_len_.fill(0);
				std::snprintf(cam_rec_len_.data(), cam_rec_len_.size(), "%u", val);
			}
		}
		if (recording_) { ImGui::EndDisabled(); }

		if (recording_) {
			if (drawButton("Stop Record", ctrl_btn_w_)) {
				stopRecord();
			}
		} else {
			if (drawButton("Start Record", ctrl_btn_w_)) {
				startRecord();
			}
		}

		ImGui::TreePop();
	}
}

void SampleViewerNode::drawDevCtrlPlay(void)
{
	int tmp_play_time = play_con_timer_;
	if (cam_dev_open_) { ImGui::SetNextItemOpen(cam_dev_open_); }
	if (ImGui::TreeNode("Target Path")) {
		if (drawButton("Directory", ctrl_dir_btn_w_, play_cap_disable_)) {
			convToFsPath(play_target_path_c_, play_target_path_);
			dir_browser_.SetPwd(play_target_path_);
			dir_browser_.ClearSelected();
			dir_browser_.Open();
			browser_target_ = FILE_BROWS_PLAY;
		}
		if ((browser_target_ == FILE_BROWS_PLAY) && dir_browser_.HasSelected()) {
			std::filesystem::path path = dir_browser_.GetSelected();
			RCLCPP_INFO(this->get_logger(), "select dir=%s", path.string<char>().c_str());
			dir_browser_.ClearSelected();
			browser_target_ = FILE_BROWS_NONE;
			if (!convToCharPath(path, play_target_path_c_)) {
				RCLCPP_ERROR(this->get_logger(), "File Path is over");
			} else {
				if (setPlayTarget(path.string<char>())) {
					play_target_path_ = path;
					updateDeviceInfo();
				} else {
					RCLCPP_ERROR(this->get_logger(), "Not exist record files in %s", path.string<char>().c_str());
					showMessage(MSG_CAUTION, "Not exist record files");
				}
			}
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
		ImGui::InputTextWithHint("##Playback Directory", PATH_HINT.data(), play_target_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);

		ImGui::TreePop();
	} else {
		cam_dev_open_ = false;
	}
	if (cam_dev_open_) { ImGui::SetNextItemOpen(cam_dev_open_); }
	if (ImGui::TreeNode("Control")) {
		bool disable_ctrl;
		if (play_cap_disable_) {
			if (drawButton("Stop Capture", ctrl_btn_w_)) {
				stopCapture();
			}
		} else {
			if (drawButton("Start Capture", ctrl_btn_w_)) {
				convToFsPath(play_target_path_c_, play_target_path_);
				if (setPlayTarget(play_target_path_.string<char>())) {
					startCapture();
				} else {
					showMessage(MSG_CAUTION, "Not exist record files");
					play_target_path_.clear();
				}
			}
		}
		disable_ctrl = !play_cap_disable_;
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (disable_ctrl) { ImGui::BeginDisabled(); }
		if (drawButton("Pause", ctrl_btn_w_)) {
			if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_PAUSE)) {
				updatePlayInfo(false);
			}
		}
		if (play_pausing_) { ImGui::BeginDisabled(); }
		if (drawButton("Slow Play", ctrl_btn_w_)) {
			if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_SLOW_PLAY)) {
				updatePlayInfo(false);
			} else {
				showMessage(MSG_ATTENTION, "Reached minimum speed");
			}
		}
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (drawButton("Fast Play", ctrl_btn_w_)) {
			if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_FAST_PLAY)) {
				updatePlayInfo(false);
			} else {
				showMessage(MSG_ATTENTION, "Reached maximum speed");
			}
		}
		if (play_pausing_) { ImGui::EndDisabled(); }
		if (drawButton("Jump Backward", ctrl_btn_w_)) {
			if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_JUMP_BW, play_jump_time_)) {
				updatePlayInfo();
			}
		}
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (drawButton("Jump Forward", ctrl_btn_w_)) {
			if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_JUMP_FW, play_jump_time_)) {
				updatePlayInfo();
			}
		}

		ImGui::SetNextItemWidth(play_time_w_);
		if (ImGui::InputText("##hour", play_jump_[0].data(), play_jump_[0].size(), TEXT_DEC_FLAG)) {
			uint32_t val = convStrVal(play_jump_[0].data());
			if (val == 0) {
				play_jump_[0].fill(0);
				std::snprintf(play_jump_[0].data(), play_jump_[0].size(), "0");
			}
		}
		ImGui::SameLine(0, play_time_space_);
		ImGui::Text(":");
		ImGui::SameLine(0, play_time_space_);
		ImGui::SetNextItemWidth(play_time_w_);
		if (ImGui::InputText("##minutes", play_jump_[1].data(), 3, TEXT_DEC_FLAG)) {
			uint32_t val = convStrVal(play_jump_[1].data());
			if ((val >= 60U) || (val == 0)) {
				play_jump_[1].fill(0);
				std::snprintf(play_jump_[1].data(), play_jump_[1].size(), "0");
			}
		}
		ImGui::SameLine(0, play_time_space_);
		ImGui::Text(":");
		ImGui::SameLine(0, play_time_space_);
		ImGui::SetNextItemWidth(play_time_w_);
		if (ImGui::InputText("##seconds", play_jump_[2].data(), 3, TEXT_DEC_FLAG) ) {
			uint32_t val = convStrVal(play_jump_[2].data());
			if ((val >= 60U) || (val == 0)) {
				play_jump_[2].fill(0);
				std::snprintf(play_jump_[2].data(), play_jump_[2].size(), "0");
			}
		}
		ImGui::SameLine(play_jump_idt_);
		if (drawButton("Jump", play_jump_w_)) {
			krm::PlayBack::PlayTime time;
			if (getPlayTime(time)) {
				uint64_t cur = (((static_cast<uint64_t>(convStrVal(play_jump_[0].data())) * 60U * 60U) +
								(static_cast<uint64_t>(convStrVal(play_jump_[1].data())) * 60U) +
								(static_cast<uint64_t>(convStrVal(play_jump_[2].data())))) *
								static_cast<uint64_t>(cur_mode_info_->fps)) / 100ULL;
				if (cur > static_cast<uint64_t>(time.total)) {
					showMessage(MSG_ATTENTION, "Jump Time is over the total time");
				} else {
					if (cur == static_cast<uint64_t>(time.total)) {
						cur = time.total - 1U;
					}
					time.current = static_cast<uint32_t>(cur);
					if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_PLAY_TIME, time.current)) {
						updatePlayInfo();
					}
				}
			}
		}

		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (ImGui::SliderInt("##Playtime", &tmp_play_time, PLAY_SLIDER_MIN, PLAY_SLIDER_MAX, "%d%%", ImGuiSliderFlags_AlwaysClamp)) {
			krm::PlayBack::PlayStatus	status;
			if (getPlayStatus(status)) {
				if (status.state != krm::PlayBack::STOPPED) {
					krm::PlayBack::PlayTime time;
					if (getPlayTime(time)) {
						time.current = static_cast<uint32_t>((static_cast<uint64_t>(time.total) * static_cast<uint64_t>(tmp_play_time)) / 100ULL);
						if (time.current >= time.total) { time.current = time.total - 1U; }
						if (setPlayCtrl(tof_camera_interface::srv::SetPlayCtrl::Request::CMD_PLAY_TIME, time.current)) {
							updatePlayInfo(false);
						}
					}
				}
			}
		}

		if (disable_ctrl) { ImGui::EndDisabled(); }

		ImGui::Indent(PLAY_TIME_IDT);
		ImGui::Text("%s", play_time_.c_str());

		ImGui::Text("Playing framerate");
		ImGui::SameLine();
		ImGui::Text("%s", play_frame_fps_.data());
		ImGui::SameLine();
		ImGui::Text("fps");
		ImGui::Unindent(PLAY_TIME_IDT);

		ImGui::TreePop();
	} else {
		cam_dev_open_ = false;
	}
	if (ImGui::TreeNode("Motion Mode")) {
		if (!cam_mode_list_.empty()) {
			ImGui::Text("%s", cam_mode_list_[0].c_str());
		}
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Image Kinds")) {
		if (!cam_image_list_.empty()) {
			ImGui::Text("%s", cam_image_list_[0].c_str());
		}
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Information##playback")) {
		if (ImGui::BeginTable("##information", DEV_INF_ROW, INF_TBL_FLAGS)) {
			for (uint8_t i = 0; i < CAM_INF_NUM; i++) {
				ImGui::TableNextRow();
				ImGui::TableNextColumn();
				ImGui::Text("%s", CAM_INF_LIST[i].data());
				ImGui::TableNextColumn();
				ImGui::Text("%s", cam_inf_list_[i].c_str());
			}
			ImGui::EndTable();
		}
		ImGui::TreePop();
	}
}

void SampleViewerNode::drawPostFiltLbl(void)
{
	if (ImGui::CollapsingHeader("Post Filter")) {
		uint16_t m_ksize = 0;
		uint16_t b_ksize = 0;
		double b_sigma_depth = static_cast<double>(pstf_prm_.bil_sigma_depth);
		double min_sigma_depth = 0.001;
		double max_sigma_depth = 1000.0;
		double b_sigma_ir = static_cast<double>(pstf_prm_.bil_sigma_ir);
		double min_sigma_ir = 0.001;
		double max_sigma_ir = 250.0;
		double b_sigma_space = static_cast<double>(pstf_prm_.bil_sigma_space);
		double min_sigma_space = 0.001;
		double max_sigma_space = 10.0;
		uint16_t f_ksize = 0;
		uint16_t f_log = 0;
		uint16_t f_fast_proc = 0;
		int f_thr = static_cast<int>(pstf_prm_.flyp_thr);
		uint16_t F_THR_MAX = 8000U;
		bool disabled = recording_;
		uint16_t i;

		if (ImGui::TreeNode("Median Filter")) {
			if (disabled || pstf_medf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_medf_)) {
				if (!tgt_dev_enable_) {
					setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF, pstf_enable_medf_);
				}
			}
			for (i = 0; sizeof(pstf_ksize_map_); i++) {
				if (pstf_ksize_map_[i] == pstf_prm_.median_ksize) {
					m_ksize = i;
					break;
				}
			}
			ImGui::Text("Filter Size");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##ksize", pstf_ksize_list_, m_ksize, disabled)) {
				pstf_prm_.median_ksize = pstf_ksize_map_[m_ksize];
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			if (disabled || pstf_medf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Bilateral Filter")) {
			if (disabled || pstf_bilf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_bilf_)) {
				if (!tgt_dev_enable_) {
					setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF, pstf_enable_bilf_);
				}
			}
			for (i = 0; sizeof(pstf_ksize_map_); i++) {
				if (pstf_ksize_map_[i] == pstf_prm_.bil_ksize) {
					b_ksize = i;
					break;
				}
			}
			ImGui::Text("Filter Size");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##ksize", pstf_ksize_list_, b_ksize, disabled)) {
				pstf_prm_.bil_ksize = pstf_ksize_map_[b_ksize];
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			ImGui::Text("Sigma Depth");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (ImGui::SliderScalar("##sigma depth", ImGuiDataType_Double, &b_sigma_depth, &min_sigma_depth, &max_sigma_depth, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_depth < min_sigma_depth) {
					pstf_prm_.bil_sigma_depth = min_sigma_depth;
				} else if (b_sigma_depth > max_sigma_depth) {
					pstf_prm_.bil_sigma_depth = max_sigma_depth;
				} else {
					pstf_prm_.bil_sigma_depth = b_sigma_depth;
				}
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			ImGui::Text("Sigma IR");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (ImGui::SliderScalar("##sigma ir", ImGuiDataType_Double, &b_sigma_ir, &min_sigma_ir, &max_sigma_ir, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_ir < min_sigma_ir) {
					pstf_prm_.bil_sigma_ir = min_sigma_ir;
				} else if (b_sigma_ir > max_sigma_ir) {
					pstf_prm_.bil_sigma_ir = max_sigma_ir;
				} else {
					pstf_prm_.bil_sigma_ir = b_sigma_ir;
				}
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			ImGui::Text("Sigma Space");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (ImGui::SliderScalar("##sigma space", ImGuiDataType_Double, &b_sigma_space, &min_sigma_space, &max_sigma_space, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_space < min_sigma_space) {
					pstf_prm_.bil_sigma_space = min_sigma_space;
				} else if (b_sigma_space > max_sigma_space) {
					pstf_prm_.bil_sigma_space = max_sigma_space;
				} else {
					pstf_prm_.bil_sigma_space = b_sigma_space;
				}
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			if (disabled || pstf_bilf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Flying Pixel Filter")) {
			if (disabled || pstf_flypf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_flypf_)) {
				if (!tgt_dev_enable_) {
					setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF, pstf_enable_flypf_);
				}
			}
			for (i = 0; sizeof(pstf_ksize_map_); i++) {
				if (pstf_ksize_map_[i] == pstf_prm_.flyp_ksize) {
					f_ksize = i;
					break;
				}
			}
			ImGui::Text("Filter Size");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##ksize", pstf_ksize_list_, f_ksize, disabled)) {
				pstf_prm_.flyp_ksize = pstf_ksize_map_[f_ksize];
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			f_log = pstf_prm_.flyp_log ? 1U : 0;
			ImGui::Text("Method");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##method", pstf_method_list_, f_log, disabled)) {
				if (f_log != 0) {
					pstf_prm_.flyp_log = true;
				} else {
					pstf_prm_.flyp_log = false;
				}
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			f_fast_proc = pstf_prm_.flyp_fast_proc ? 1U : 0;
			ImGui::Text("Processing Priority");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##priority", pstf_priority_list_, f_fast_proc, disabled)) {
				if (f_fast_proc != 0) {
					pstf_prm_.flyp_fast_proc = true;
				} else {
					pstf_prm_.flyp_fast_proc = false;
				}
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			ImGui::Text("Threshold");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (ImGui::InputInt("##thr", &f_thr)) {
				if (f_thr > F_THR_MAX) { f_thr = F_THR_MAX; }
				if (f_thr < 0) { f_thr = 0; }
				pstf_prm_.flyp_thr = static_cast<uint16_t>(f_thr);
				if (!tgt_dev_enable_) {
					setPostFiltPrm(pstf_prm_);
				}
			}
			if (disabled || pstf_flypf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
	}
}

void SampleViewerNode::drawPostProcLbl(void)
{
	if (ImGui::CollapsingHeader("Post Process")) {
		bool disabled = recording_;
		if (ImGui::TreeNode("Distortion")) {
			if (disabled || pst_dist_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pst_dist_enable_)) {
				if (!tgt_dev_enable_) {
					setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_CONV_DIST, pst_dist_enable_);
				}
			}
			if (disabled || pst_dist_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Point Cloud Conversion")) {
			std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
			if (disabled) { ImGui::BeginDisabled(); }

			if (ImGui::RadioButton("Camera Origin", &pst_pcd_origin_, 0)) {
				pst_pcd_cam_coord_ = true;
				if (!tgt_dev_enable_) {
					bool world_coord = !pst_pcd_cam_coord_;
					setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_PCD_KIND, world_coord);
				}
				grid_view_kind_ = (grid_view_kind_ == GRID_KIND_FORCE_ORIGIN) ? GRID_KIND_FORCE_ORIGIN : GRID_KIND_CAMERA;
				draw_pcd->updateGridSetting();
			}
			if (ImGui::RadioButton("World Origin", &pst_pcd_origin_, 1)) {
				pst_pcd_cam_coord_ = false;
				if (!tgt_dev_enable_) {
					bool world_coord = !pst_pcd_cam_coord_;
					setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_PCD_KIND, world_coord);
				}
				grid_view_kind_ = (grid_view_kind_ == GRID_KIND_FORCE_ORIGIN) ? GRID_KIND_FORCE_ORIGIN : GRID_KIND_ORIGIN;
				draw_pcd->updateGridSetting();
			}

			if (ImGui::TreeNode("Origin Offset")) {
				if (pst_pcd_cam_coord_) { ImGui::BeginDisabled(); }
				ImGui::Text("X-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_ofst_idt_);
				ImGui::SetNextItemWidth(pst_pcd_ofst_w_);
				if (ImGui::InputInt("##X-axis Origin Offset", &pst_pcd_ofst_x_, PST_PCD_OFST_STEP)) {
					if (pst_pcd_ofst_x_ > INT16_MAX) { pst_pcd_ofst_x_ = INT16_MAX; }
					if (pst_pcd_ofst_x_ < INT16_MIN) { pst_pcd_ofst_x_ = INT16_MIN; }
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::SameLine();
				ImGui::Text("(mm)");
				ImGui::Unindent(pst_pcd_ofst_idt_);
				ImGui::Text("Y-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_ofst_idt_);
				ImGui::SetNextItemWidth(pst_pcd_ofst_w_);
				if (ImGui::InputInt("##Y-axis Origin Offset", &pst_pcd_ofst_y_, PST_PCD_OFST_STEP)) {
					if (pst_pcd_ofst_y_ > INT16_MAX) { pst_pcd_ofst_y_ = INT16_MAX; }
					if (pst_pcd_ofst_y_ < INT16_MIN) { pst_pcd_ofst_y_ = INT16_MIN; }
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::SameLine();
				ImGui::Text("(mm)");
				ImGui::Unindent(pst_pcd_ofst_idt_);
				ImGui::Text("Z-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_ofst_idt_);
				ImGui::SetNextItemWidth(pst_pcd_ofst_w_);
				if (ImGui::InputInt("##Z-axis Origin Offset", &pst_pcd_ofst_z_, PST_PCD_OFST_STEP)) {
					if (pst_pcd_ofst_z_ > INT16_MAX) { pst_pcd_ofst_z_ = INT16_MAX; }
					if (pst_pcd_ofst_z_ < INT16_MIN) { pst_pcd_ofst_z_ = INT16_MIN; }
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::SameLine();
				ImGui::Text("(mm)");
				ImGui::Unindent(pst_pcd_ofst_idt_);

				ImGui::TreePop();
				if (pst_pcd_cam_coord_) { ImGui::EndDisabled(); }
			}
			if (ImGui::TreeNode("Rotation")) {
				if (pst_pcd_cam_coord_) { ImGui::BeginDisabled(); }
				ImGui::Text("X-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_rot_idt_);
				ImGui::SetNextItemWidth(pst_pcd_rot_w_);
				if (ImGui::SliderAngle("##X-axis Rotation", &pst_pcd_rot_x_, PST_PCD_ROT_MIN, PST_PCD_ROT_MAX,"%.1fdeg", ImGuiSliderFlags_AlwaysClamp)) {
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::Unindent(pst_pcd_rot_idt_);
				ImGui::Text("Y-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_rot_idt_);
				ImGui::SetNextItemWidth(pst_pcd_rot_w_);
				if (ImGui::SliderAngle("##Y-axis Rotation", &pst_pcd_rot_y_, PST_PCD_ROT_MIN, PST_PCD_ROT_MAX,"%.1fdeg", ImGuiSliderFlags_AlwaysClamp)) {
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::Unindent(pst_pcd_rot_idt_);
				ImGui::Text("Z-axis");
				ImGui::SameLine();
				ImGui::Indent(pst_pcd_rot_idt_);
				ImGui::SetNextItemWidth(pst_pcd_rot_w_);
				if (ImGui::SliderAngle("##Z-axis Rotation", &pst_pcd_rot_z_, PST_PCD_ROT_MIN, PST_PCD_ROT_MAX,"%.1fdeg", ImGuiSliderFlags_AlwaysClamp)) {
					if (!tgt_dev_enable_) { notifyPcdPrm(); }
				}
				ImGui::Unindent(pst_pcd_rot_idt_);

				ImGui::TreePop();
				if (pst_pcd_cam_coord_) { ImGui::EndDisabled(); }
			}
			if (disabled) { ImGui::EndDisabled(); }

			ImGui::TreePop();
		}
	}
}

void SampleViewerNode::drawViewStgLbl(void)
{
	if (ImGui::CollapsingHeader("View Setting")) {
		if (ImGui::TreeNode("Depth Color")) {
			ImGui::Text("%u mm", view_depth_range_.min);
			ImGui::SameLine(view_dpt_rng_idt_);
			ImGui::Text("%5u mm", view_depth_range_.max);

			drawImage(TEXTURE_COLOR_BAR, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);

			ImGui::SetNextItemWidth(view_dpt_w_);
			if (view_dep_range_min_ == view_dep_range_min_max_) { ImGui::BeginDisabled(); }
			if (ImGui::DragInt("##Depth Color Min", &view_dep_min_, VIEW_DEPTH_DRAG_VAL, view_dep_range_min_, view_dep_range_min_max_, "Min : %u mm", ImGuiSliderFlags_AlwaysClamp)) {
				view_dep_range_max_min_ = view_dep_min_ + VIEW_COLOR_DIFF_MIN;
				color_table_->setColorRange({static_cast<uint16_t>(view_dep_min_), static_cast<uint16_t>(view_dep_max_)});
				color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
				if (state_cam_ == ST_PAUSE_PLAY) {
					draw_imgs_[krm::IMG_DEPTH]->refresh();
					draw_imgs_[IMG_PCD]->refresh();
				}
			}
			if (view_dep_range_min_ == view_dep_range_min_max_) { ImGui::EndDisabled(); }
			ImGui::SameLine(view_dpt_btn_idt_, 0);
			ImGui::SetNextItemWidth(view_dpt_w_);
			if (view_dep_range_max_min_ == view_dep_range_max_) { ImGui::BeginDisabled(); }
			if (ImGui::DragInt("##Depth Color Max", &view_dep_max_, VIEW_DEPTH_DRAG_VAL, view_dep_range_max_min_, view_dep_range_max_, "Max : %u mm", ImGuiSliderFlags_AlwaysClamp)) {
				view_dep_range_min_max_ = view_dep_max_ - VIEW_COLOR_DIFF_MIN;
				color_table_->setColorRange({static_cast<uint16_t>(view_dep_min_), static_cast<uint16_t>(view_dep_max_)});
				color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
				if (state_cam_ == ST_PAUSE_PLAY) {
					draw_imgs_[krm::IMG_DEPTH]->refresh();
					draw_imgs_[IMG_PCD]->refresh();
				}
			}
			if (view_dep_range_max_min_ == view_dep_range_max_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Grayscale Color")) {
			ImGui::Text("Gain");
			ImGui::SameLine();
			ImGui::Indent(view_gray_idt_);
			ImGui::SetNextItemWidth(view_gray_w_);
			if (ImGui::SliderFloat("##Gain", &view_gray_gain_, VIEW_GRAY_GAIN_MIN, VIEW_GRAY_GAIN_MAX, "%.1f", ImGuiSliderFlags_AlwaysClamp)) {
				gray_table_->setGain(view_gray_gain_);
				if (state_cam_ == ST_PAUSE_PLAY) {
					draw_imgs_[krm::IMG_IR]->refresh();
					for (uint8_t i = krm::IMG_RAW1; i <= krm::IMG_RAW4; i++) {
						draw_imgs_[i]->refresh();
					}
					draw_imgs_[IMG_PCD]->refresh();
					raw_win_->refresh();
					raw_win_->draw();
					glfwMakeContextCurrent(win_);
				}
			}
			ImGui::Unindent(view_gray_idt_);

			ImGui::Text("Gamma");
			ImGui::SameLine();
			ImGui::Indent(view_gray_idt_);
			ImGui::SetNextItemWidth(view_gray_w_);
			if (ImGui::SliderFloat("##Gamma", &view_gray_gamma_, VIEW_GRAY_GAMMA_MIN, VIEW_GRAY_GAMMA_MAX, "%.1f", ImGuiSliderFlags_AlwaysClamp)) {
				gray_table_->setGamma(view_gray_gamma_);
				if (state_cam_ == ST_PAUSE_PLAY) {
					draw_imgs_[krm::IMG_IR]->refresh();
					for (uint8_t i = krm::IMG_RAW1; i <= krm::IMG_RAW4; i++) {
						draw_imgs_[i]->refresh();
					}
					draw_imgs_[IMG_PCD]->refresh();
					raw_win_->refresh();
					raw_win_->draw();
					glfwMakeContextCurrent(win_);
				}
			}
			ImGui::Unindent(view_gray_idt_);

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("View Image")) {
			ImGui::SetNextItemWidth(ctrl_node_w_);
			if (view_image_pair_.empty()) { ImGui::BeginDisabled(); }
			if (drawCombo("##View Image", view_image_list_, view_image_idx_)) {
				updateShowImgKind();
			}
			if (view_image_pair_.empty()) { ImGui::EndDisabled(); }
			if (drawButton("RAW expand", ctrl_btn_w_, cam_disable_raw_)) {
				if (!raw_win_->toggle()) {
					mouse_enter_raw_ = false;
				}
				glfwMakeContextCurrent(win_);
			}

			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Point Cloud")) {
			std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
			if (ImGui::TreeNode("Auxiliary line")) {
				ImGui::Checkbox("Enable", &view_aux_check_);
				if (!view_aux_check_) { ImGui::BeginDisabled(); }
				ImGui::Text("Gird Interval");
				ImGui::SameLine();
				ImGui::Indent(view_pcd_input_idt_);
				ImGui::SetNextItemWidth(view_pcd_input_w_);
				if (ImGui::DragInt("##Gird Interval", &view_aux_grid_, VIEW_AUX_DRAG_VAL, VIEW_AUX_MIN, VIEW_AUX_MAX, "%u", ImGuiSliderFlags_AlwaysClamp)) {
					draw_pcd->updateGridSetting(view_aux_grid_);
				}
				ImGui::SameLine();
				ImGui::Text("(mm)");
				if (!view_aux_check_) { ImGui::EndDisabled(); }
				ImGui::Unindent(view_pcd_input_idt_);

				ImGui::TreePop();
			}
			if (ImGui::TreeNode("Point Color")) {
				bool ir_invalid = ((out_kind_ == krm::OUT_IMG_DEPTH) || (out_kind_ == krm::OUT_IMG_RAW));
				if (ImGui::RadioButton("Depth Color", &view_radio_button_, PCD_C_DEPTH)) {
					setPcdColor(tof_camera_interface::srv::SetPcdColor::Request::PCD_COLOR_NONE);
					draw_pcd->setKind(false);
				}
				if (ir_invalid) { ImGui::BeginDisabled(); }
				if (ImGui::RadioButton("Grayscale(IR)", &view_radio_button_, PCD_C_IR)) {
					setPcdColor(tof_camera_interface::srv::SetPcdColor::Request::PCD_COLOR_IR);
					draw_pcd->setKind(true);
				}
				if (ir_invalid) { ImGui::EndDisabled(); }
				ImGui::TreePop();
			}
			if (ImGui::TreeNode("View Point")) {
				if (drawButton("Front", view_pcd_ang_btn_)) {
					draw_pcd->setViewPos(VIEW_FRONT);
				}
				ImGui::SameLine();
				if (drawButton("Top", view_pcd_ang_btn_)) {
					draw_pcd->setViewPos(VIEW_TOP);
				}
				ImGui::SameLine();
				if (drawButton("Left", view_pcd_ang_btn_)) {
					draw_pcd->setViewPos(VIEW_LEFT);
				}
				ImGui::SameLine();
				if (drawButton("Right", view_pcd_ang_btn_)) {
					draw_pcd->setViewPos(VIEW_RIGHT);
				}
				ImGui::TreePop();
			}
			if (ImGui::TreeNode("Interleaving")) {
				bool do_set = false;
				if (ImGui::Checkbox("Enable", &view_intlv_enable_)) {
					do_set = true;
				}
				if (!view_intlv_enable_) { ImGui::BeginDisabled(); }
				ImGui::Text("Distance");
				ImGui::SameLine();
				ImGui::Indent(view_pcd_input_idt_);
				ImGui::SetNextItemWidth(view_pcd_input_w_);
				if (ImGui::DragInt("##Intlv Distance", &view_intlv_distance_, VIEW_INTLV_DRAG_VAL, VIEW_INTLV_DIST_MIN, VIEW_INTLV_DIST_MAX, "%d", ImGuiSliderFlags_AlwaysClamp)) {
					do_set = true;
				}
				ImGui::SameLine();
				ImGui::Text("(mm)");
				ImGui::Unindent(view_pcd_input_idt_);
				if (ImGui::RadioButton("Sampling from two pixels", &view_intlv_factor_, 2)) {
					do_set = true;
				}
				if (ImGui::RadioButton("Sampling from four pixels", &view_intlv_factor_, 4)) {
					do_set = true;
				}
				if (!view_intlv_enable_) { ImGui::EndDisabled(); }
				if (do_set) {
					draw_pcd->setInterleave(view_intlv_enable_, static_cast<float>(view_intlv_distance_), static_cast<uint8_t>(view_intlv_factor_));
				}
				ImGui::TreePop();
			}
			ImGui::TreePop();
		}
		if (view_disable_winsize_) { ImGui::BeginDisabled(); ImGui::SetNextItemOpen(false); }
		if (ImGui::TreeNode("Window Size")) {
			if (ImGui::RadioButton("Default Size", &view_radio_winsize_, DEFAULT_WIN_SIZE)) {
				updateLayout();
			}
			if (ImGui::RadioButton("Double Size", &view_radio_winsize_, DOUBLE_WIN_SIZE)) {
				updateLayout();
			}
			ImGui::TreePop();
		}
		if (view_disable_winsize_) { ImGui::EndDisabled(); }
	}
}

void SampleViewerNode::drawViewPanel(void)
{
	ShowDisplayKind disp;
	ImVec2 pos = pnl_view_pos_;
	const std::array<std::string_view, DISP_KINDS>& str = SHOW_DISP_LIST;

	if (updateCapture()) {
		raw_win_->refresh();
		raw_win_->draw();
		glfwMakeContextCurrent(win_);
	}
	for (uint8_t i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
		disp = show_img_[i];
		ImGui::SetNextWindowPos(pos);
		ImGui::SetNextWindowSize(pnl_view_size_);
		ImGui::Begin(str[disp].data(), NULL, VIEW_WIN_FLAG);
		img_win_pos_[i] = ImGui::GetCursorScreenPos();

		switch (disp) {
		case DISP_DEPTH:
			draw_imgs_[krm::IMG_DEPTH]->lock();
			drawImage(i, show_fmt_[i].width, show_fmt_[i].height, draw_imgs_[krm::IMG_DEPTH]->getDrawImage());
			draw_imgs_[krm::IMG_DEPTH]->unlock();
			break;
		case DISP_IR:
			draw_imgs_[krm::IMG_IR]->lock();
			drawImage(i, show_fmt_[i].width, show_fmt_[i].height, draw_imgs_[krm::IMG_IR]->getDrawImage());
			draw_imgs_[krm::IMG_IR]->unlock();
			break;
		case DISP_PCD:
			updatePointCloud(pos);
			break;
		default:
			break;
		}

		ImGui::End();
		pos.x += pnl_view_size_.x;
	}
}

void SampleViewerNode::drawStatusPanel(void)
{
	ImGui::SetNextWindowPos(pnl_sts0_pos_);
	ImGui::SetNextWindowSize(pnl_sts0_size_);
	ImGui::Begin("Status", NULL, STS_WIN_FLAG);
	drawStatusWin();
	ImGui::End();

	ImGui::SetNextWindowPos(pnl_sts1_pos_);
	ImGui::SetNextWindowSize(pnl_sts0_size_);
	ImGui::Begin("Cursor position value", NULL, STS_WIN_FLAG);
	drawCursorWin();
	ImGui::End();

	ImGui::SetNextWindowPos(pnl_sts2_pos_);
	ImGui::SetNextWindowSize(pnl_sts0_size_);
	ImGui::Begin("Other", NULL, STS_WIN_FLAG);
	drawOtherWin();
	ImGui::End();
}

void SampleViewerNode::drawStatusWin(void)
{
	ImGui::Text("Device Status");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", STATE_CAM_LIST[state_cam_].data());

	ImGui::Text("Record Status");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", STATE_REC_LIST[state_rec_].data());

	ImGui::Text("Receive Rate");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", state_fps_.data());
	ImGui::SameLine(sts_fps_idt_);
	ImGui::Text(" fps");
}

void SampleViewerNode::drawCursorWin(void)
{
	char d_str[16];
	char i_str[16];
	char r_str[4][16];
	std::string raw_name;
	const ImGuiIO& io = ImGui::GetIO();
	bool	enable_pos = false;
	krm::ImageKind kind;
	krm::Point2d	mouse_pos;
	uint8_t i;
	float raw_indent = 0;

	ImGui::Text("Cursor Monitor (Average)");
	if (mouse_enter_raw_) {
		for (i = krm::IMG_RAW1; i <= krm::IMG_RAW4; i++) {
			if (std::dynamic_pointer_cast<DrawRaw>(draw_imgs_[i])->convPoint2d(mouse_pos_raw_, mouse_pos)) {
				enable_pos = true;
				break;
			}
		}
	} else {
		for (i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
			switch (show_img_[i]) {
			case DISP_DEPTH:	kind = krm::IMG_DEPTH;	break;
			case DISP_IR:		kind = krm::IMG_IR;		break;
			default:			kind = krm::IMG_KINDS;	break;
			}
			if (kind != krm::IMG_KINDS) {
				if ((io.MousePos.x >= img_win_pos_[i].x) && (io.MousePos.x < (img_win_pos_[i].x + show_fmt_[i].width)) &&
					(io.MousePos.y >= img_win_pos_[i].y) && (io.MousePos.y < (img_win_pos_[i].y + show_fmt_[i].height))) {
					mouse_pos.x = static_cast<uint16_t>(io.MousePos.x - img_win_pos_[i].x);
					mouse_pos.y = static_cast<uint16_t>(io.MousePos.y - img_win_pos_[i].y);
					if (view_radio_winsize_ == DOUBLE_WIN_SIZE) {
						mouse_pos.x /= 2;
						mouse_pos.y /= 2;
					}
					enable_pos = true;
					break;
				}
			}
		}
	}
	if (enable_pos) {
		ImGui::Text("Pos  (%u,%u)", mouse_pos.x, mouse_pos.y);
		if (draw_imgs_[krm::IMG_DEPTH]->getValue(mouse_pos) == krm::INVALID_DEPTH) {
			std::snprintf(d_str, sizeof(d_str), "-----");
		} else {
			std::snprintf(d_str, sizeof(d_str), "%5u", draw_imgs_[krm::IMG_DEPTH]->getValue(mouse_pos));
		}
		std::snprintf(i_str, sizeof(i_str), "%5u", draw_imgs_[krm::IMG_IR]->getValue(mouse_pos));
		for (i = 0; i < RAW_MAX; i++) {
			std::snprintf(r_str[i], sizeof(r_str), "%5u", draw_imgs_[krm::IMG_RAW1 + i]->getValue(mouse_pos));
		}
	} else {
		ImGui::Text("Pos  (---,---)");
	}
	ImGui::Text("Depth");
	ImGui::SameLine(sts_cur_d_val_idt_);
	ImGui::Text("%s", (enable_pos && img_fmts_[krm::IMG_DEPTH].isExist()) ? d_str : "-----");
	ImGui::SameLine(sts_cur_ir_idt_);
	ImGui::Text("IR");
	ImGui::SameLine(sts_cur_ir_val_idt_);
	ImGui::Text("%s", (enable_pos && img_fmts_[krm::IMG_IR].isExist()) ? i_str : "-----");
	ImGui::Text("RAW");
	for (i = 0; i < RAW_MAX; i++) {
		raw_indent += sts_cur_raw_g_idt_;
		ImGui::SameLine(raw_indent);
		raw_name = "G" + std::to_string(i + 1U) + " : ";
		ImGui::Text("%s", raw_name.c_str());
		raw_indent += sts_cur_raw_idt_;
		ImGui::SameLine(raw_indent);
		if (enable_pos && img_fmts_[krm::IMG_RAW1 + i].isExist()) {
			ImGui::Text("%s", r_str[i]);
		} else {
			ImGui::Text("%s", "-----");
		}
	}
}

void SampleViewerNode::drawOtherWin(void)
{
	std::string sdk_ver = makeVerString(krm::SDK_VERSION);
	ImGui::Text("SDK Version");
	ImGui::SameLine(sts_oth_idt_);
	ImGui::Text("%s", sdk_ver.c_str());

	ImGui::Text("Configuration");
	ImGui::SameLine(sts_oth_idt_);
	if (drawButton("Save", sts_oth_btn_w_, stat_disable_cfg_save_)) {
		saveConfig();
	}

	ImGui::SameLine();
	if (drawButton("Reload", sts_oth_btn_w_, stat_disable_cfg_load_)) {
		loadConfig();
		if (state_cam_ == ST_STOPPED) {
			if (tgt_cam_idx_ == PLAYBACK_INDEX) {
				if (setPlayTarget(play_target_path_.string<char>())) {
					updatePlayInfo();
				} else {
					convToFsPath(play_target_path_c_, play_target_path_);
				}
			}
		}
	}

	ImGui::Text("Hide Control Panel");
	ImGui::SameLine(sts_oth_idt_);
	if (drawButton(hide_ctrl_panel_str_, sts_oth_btn_w_)) {
		hide_ctrl_panel_ = !hide_ctrl_panel_;
		if (hide_ctrl_panel_) {
			hide_ctrl_panel_str_ = "Show";
			stat_disable_cfg_save_ = true;
			stat_disable_cfg_load_ = true;
		} else {
			hide_ctrl_panel_str_ = "Hide";
			enableCfgBtn();
		}
		updateLayout();
	}
}

void SampleViewerNode::showMessage(MSG_KIND msg_kind, const std::string& message)
{
	int w;
	float scale = (view_radio_winsize_ == DOUBLE_WIN_SIZE) ? 2.F : 1.F;

	show_message_ = true;
	show_msg_closing_ = (msg_kind != MSG_INFOMATION);
	message_lbl_ = MSG_KIND_STR[msg_kind];
	message_str_ = message;
	w = static_cast<int>((message.length() * POP_MSG_WIN_STR_W) + POP_MSG_WIN_W_OFST);
	pnl_msg_size_.x = static_cast<float>(w) * scale;
	pnl_msg_size_.y = POP_MSG_WIN_H * scale;
	pnl_msg_pos_.x = (static_cast<float>(win_w_) / 2.F) - (pnl_msg_size_.x / 2.F);
	pnl_msg_pos_.y = (static_cast<float>(win_h_) / 2.F) - (pnl_msg_size_.y / 2.F);
}

uint32_t SampleViewerNode::convStrVal(const std::string& str, int base)
{
	uint32_t val;
	try {
		val = static_cast<uint32_t>(std::stoul(str, nullptr, base));
	} catch (const std::out_of_range& /*e*/) {
		val = UINT32_MAX;
	} catch (...) {
		val = 0;
	}
	return val;
}

float SampleViewerNode::convDegRad(float deg)
{
	return ((deg * static_cast<float>(M_PI)) / 180.0f);
}

float SampleViewerNode::convRadDeg(float rad)
{
	float deg = (rad * 180.0f) / static_cast<float>(M_PI);
	deg = round(deg * 10.0f);
	return (deg / 10.0f);
}

std::string SampleViewerNode::makeVerString(const krm::Version& ver)
{
	std::string str =	std::to_string(ver.major) + "." +
						std::to_string(ver.minor) + "." +
						std::to_string(ver.rev);
	return str;
}

std::string SampleViewerNode::makeModeString(const krm::ModeInfo& mode_info)
{
	std::stringstream ss;
	ss << std::to_string(mode_info.id) << " : " << mode_info.description;
	return ss.str();
}

std::string SampleViewerNode::makeDegString(uint16_t deg)
{
	std::stringstream ss;
	uint16_t deg_u = deg / 100U;
	uint16_t deg_l = deg % 100U;
	ss << deg_u << "." << std::setw(2) << std::setfill('0') << deg_l << " (deg)";
	return ss.str();
}

void SampleViewerNode::makeFpsString(uint16_t fps, std::array<char, 16>& fps_str, bool stuff_left)
{
	fps_str.fill(0);
	if (fps == UINT16_MAX) {
		std::snprintf(fps_str.data(), fps_str.size(), "-------");
	} else if (stuff_left) {
		std::snprintf(fps_str.data(), fps_str.size(), "%u.%02u", (fps / 100U), (fps % 100U));
	} else {
		std::snprintf(fps_str.data(), fps_str.size(), "%4u.%02u", (fps / 100U), (fps % 100U));
	}
}

void SampleViewerNode::makePlayTimeString(uint32_t frame, std::array<std::string, 3>& time_str)
{
	uint64_t sec;
	uint32_t min, hour;

	if (cur_mode_info_ != nullptr) {
		sec  = (frame * 100U) / cur_mode_info_->fps;
		min  = static_cast<uint32_t>(sec / 60U);
		hour = min / 60U;
		min %= 60U;
		sec %= 60U;
	} else {
		sec = 0;
		min = 0;
		hour = 0;
	}

	time_str[0] = std::to_string(hour);
	time_str[1] = std::to_string(min);
	time_str[2] = std::to_string(sec);
}

void SampleViewerNode::drawImage(uint8_t texture_idx, uint16_t width, uint16_t height, const std::vector<RgbaColor>& image)
{
	glBindTexture(GL_TEXTURE_2D, texture_[texture_idx]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
	ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(texture_[texture_idx])), ImVec2(width, height));
	glBindTexture(GL_TEXTURE_2D, 0);
}

void SampleViewerNode::changePcdView(const ImGuiIO& io, ImVec2 win_pos, ImVec2 win_size)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	bool is_cursor_inside = ImRect(win_pos, ImVec2(win_pos.x + win_size.x, win_pos.y + win_size.y)).Contains(io.MousePos);
	is_scale_changing_ = (static_cast<int32_t>(io.MouseWheel) != 0) && is_cursor_inside;

	if (is_cursor_inside) {
		if (!is_view_rot_changing_ && !is_view_pos_changing_ && io.MouseDown[ImGuiMouseButton_Left] && ((std::abs(io.MouseDelta.x) > 0.0F) || (std::abs(io.MouseDelta.y) > 0.0F))) {
			is_view_rot_changing_ = true;
		} else if ((is_view_rot_changing_ && !io.MouseDown[ImGuiMouseButton_Left])) {
			is_view_rot_changing_ = false;
		} else if (!is_view_rot_changing_ && !is_view_pos_changing_ && io.MouseDown[ImGuiMouseButton_Right] && ((std::abs(io.MouseDelta.x) > 0.0F) || (std::abs(io.MouseDelta.y) > 0.0F))) {
			is_view_pos_changing_ = true;
		} else if ((is_view_pos_changing_ && !io.MouseDown[ImGuiMouseButton_Right])) {
			is_view_pos_changing_ = false;
		} else {
			/* do nothing */
		}
	} else {
		is_view_rot_changing_ = false;
		is_view_pos_changing_ = false;
	}

	if (is_view_rot_changing_) {
		draw_pcd->setViewCameraRot(io.MouseDelta.x, io.MouseDelta.y);
	} else if (is_view_pos_changing_) {
		draw_pcd->setViewCameraPos(io.MouseDelta.x, io.MouseDelta.y);
	} else {
		/* do nothing */
	}
	if (is_scale_changing_) {
		draw_pcd->scalingView(io.MouseWheel < 0);
	}

	if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle)) {
		draw_pcd->setViewPos();
	}
}

void SampleViewerNode::updatePointCloud(const ImVec2& pos, bool need_update)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	const ImGuiIO& io = ImGui::GetIO();

	changePcdView(io, pos, pnl_view_size_);
	if (view_aux_check_) {
		drawGrid(draw_list);
	}
	drawPointCloud(draw_list, need_update);
}

void SampleViewerNode::drawPointCloud(ImDrawList* draw_list, bool need_update)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	RGBA32* rgba;

	draw_pcd->copyDrawImage(draw_image_);
	draw_pcd->convScreenPcd(&draw_image_, (is_view_rot_changing_ || is_view_pos_changing_ || is_scale_changing_ || need_update), draw_points_);
	for (auto point : draw_points_) {
		rgba = reinterpret_cast<RGBA32*>(&point.color);
		draw_list->AddCircleFilled(ImVec2(point.x, point.y), pcd_point_size_, IM_COL32(rgba->r, rgba->g, rgba->b, 255));
	}
}

void SampleViewerNode::drawGrid(ImDrawList* draw_list)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	std::vector<std::pair<krm::Point3d, krm::Point3d>> lines;
	if (grid_view_kind_ == GRID_KIND_CAMERA) {
		draw_pcd->convScreenGridCamera((is_view_rot_changing_ || is_view_pos_changing_ || is_scale_changing_), lines);
	} else {
		draw_pcd->convScreenGridWorld((is_view_rot_changing_ || is_view_pos_changing_ || is_scale_changing_), lines);
	}
	for (auto line : lines) {
		draw_list->AddLine(ImVec2(line.first.x, line.first.y), ImVec2(line.second.x, line.second.y), grid_color_, grid_line_w_);
	}
}

void SampleViewerNode::updateRawCursor(GLFWwindow* /*window*/, double x, double y)
{
	mouse_pos_raw_ = {static_cast<uint16_t>(x), static_cast<uint16_t>(y)};
}

void SampleViewerNode::enterRawCursor(GLFWwindow* /*window*/, int entered)
{
	mouse_enter_raw_ = (entered == GL_TRUE);
}

void SampleViewerNode::recvFrameData(tof_camera_interface::msg::FrameData::UniquePtr frame)
{
	tof_camera_interface::msg::FrameImage* msg_data[krm::IMG_KINDS] = {&frame->depth, &frame->ir, &frame->raw1, &frame->raw2, &frame->raw3, &frame->raw4};
	tof_camera_interface::msg::FrameImage* src;
	krm::ImageData* dst;
	krm::PcdData* pcd;

	if (frame->stopped) {
		th_mtx_.lock();
		rcv_fps_ = UINT16_MAX;
		rcv_play_time_.current = 0;
		recv_stop_ = true;
		th_mtx_.unlock();
		return;
	}

	for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
		src = msg_data[i];
		dst = &frame_.images[i];
		light_cnt_[i] = src->info.light_cnt;
		std::memcpy(dst->data.data(), src->image.data.data(), src->image.data.size());
	}
	pcd = &frame_.pcd;
	std::memcpy(pcd->data.data(), frame->pcd.data.data(), frame->pcd.row_step * frame->pcd.height);

	for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
		draw_imgs_[i]->setImage(frame_.images[i]);
	}
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setImage(frame_.pcd);

	th_mtx_.lock();
	rcv_fps_ = frame->rcv_fps;
	rcv_play_time_.current = frame->play_time.current;
	rcv_play_time_.total = frame->play_time.total;
	recv_frm_ = true;
	th_mtx_.unlock();
	RCLCPP_INFO(this->get_logger(), "recvFrame");
}

void SampleViewerNode::recvNotify(tof_camera_interface::msg::Notify::SharedPtr notify)
{
	th_mtx_.lock();
	switch (notify->notify) {
	case tof_camera_interface::msg::Notify::ERR_PARAM:
	case tof_camera_interface::msg::Notify::ERR_TIMEOUT:
	case tof_camera_interface::msg::Notify::ERR_SYSTEM:
	case tof_camera_interface::msg::Notify::PLAY_REACHED_EOF:
	case tof_camera_interface::msg::Notify::REC_REACHED_EOF:
	case tof_camera_interface::msg::Notify::REC_ERR_SYSTEM:
		recv_frm_stat_ = notify->notify;
		recv_notify_ = true;
		break;
	default:
		break;
	}
	th_mtx_.unlock();
	RCLCPP_INFO(this->get_logger(), "recvNotify (%d)", notify->notify);
}

bool SampleViewerNode::getDevList(uint8_t type, std::vector<krm::ConnDevice>& dev_list)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetDevList::Request>();

	request->type.type = type;

	clt_res_.wait = false;
	clt_get_dev_list_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetDevList, this, std::placeholders::_1));

	if (waitSrvRes()) {
		dev_list = clt_res_.srv_result.dev_list;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetDevList(
	rclcpp::Client<tof_camera_interface::srv::GetDevList>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		uint16_t num = static_cast<uint16_t>(future.get()->dev_list.size());
		std::vector<krm::ConnDevice>* dev_list = &clt_res_.srv_result.dev_list;
		dev_list->resize(num);
		for (uint16_t i = 0; i < num; i++) {
			dev_list->at(i).id = future.get()->dev_list[i].id;
			dev_list->at(i).name = future.get()->dev_list[i].name;
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::openDev(uint8_t type, uint16_t dev_id)
{
	auto request = std::make_shared<tof_camera_interface::srv::OpenDev::Request>();

	request->type.type = type;
	request->dev_id = dev_id;

	clt_res_.wait = false;
	clt_open_dev_->async_send_request(request,
		std::bind(&SampleViewerNode::cbOpenDev, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbOpenDev(
	rclcpp::Client<tof_camera_interface::srv::OpenDev>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::closeDev(void)
{
	auto request = std::make_shared<tof_camera_interface::srv::CloseDev::Request>();

	clt_res_.wait = false;
	clt_close_dev_->async_send_request(request,
		std::bind(&SampleViewerNode::cbCloseDev, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbCloseDev(
	rclcpp::Client<tof_camera_interface::srv::CloseDev>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setTofCtrl(uint8_t cmd)
{
	auto request = std::make_shared<tof_camera_interface::srv::TofCtrl::Request>();

	request->cmd = cmd;

	clt_res_.wait = false;
	clt_tof_ctrl_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetTofCtrl, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetTofCtrl(
	rclcpp::Client<tof_camera_interface::srv::TofCtrl>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setExtTriggerType(uint8_t type)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetExtTriggerType::Request>();

	request->type.ext_trigger_type = type;

	clt_res_.wait = false;
	clt_set_ext_trigger_type_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetExtTriggerType, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetExtTriggerType(
	rclcpp::Client<tof_camera_interface::srv::SetExtTriggerType>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setExtTriggerOffset(uint8_t offset)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetExtTriggerOffset::Request>();

	request->offset = offset;

	clt_res_.wait = false;
	clt_set_ext_trigger_offset_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetExtTriggerOffset, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetExtTriggerOffset(
	rclcpp::Client<tof_camera_interface::srv::SetExtTriggerOffset>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setMode(uint8_t mode)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetMode::Request>();

	request->mode = mode;

	clt_res_.wait = false;
	clt_set_mode_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetMode, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetMode(
	rclcpp::Client<tof_camera_interface::srv::SetMode>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setImgKinds(uint8_t kind)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetImgKinds::Request>();

	request->img_out.img_out_kind = kind;

	clt_res_.wait = false;
	clt_set_img_kinds_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetImgKinds, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetImgKinds(
	rclcpp::Client<tof_camera_interface::srv::SetImgKinds>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setLightTimes(uint32_t count)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetLightTimes::Request>();

	request->count = count;

	clt_res_.wait = false;
	clt_set_light_times_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetLightTimes, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetLightTimes(
	rclcpp::Client<tof_camera_interface::srv::SetLightTimes>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setAEState(bool enable)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetAEState::Request>();

	request->enable = enable;

	clt_res_.wait = false;
	clt_set_ae_state_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetAEState, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetAEState(
	rclcpp::Client<tof_camera_interface::srv::SetAEState>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setAEInterval(uint8_t interval)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetAEInterval::Request>();

	request->interval = interval;

	clt_res_.wait = false;
	clt_set_ae_interval_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetAEInterval, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetAEInterval(
	rclcpp::Client<tof_camera_interface::srv::SetAEInterval>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setIntSupp(const krm::IntSuppInfo& prm)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetIntSupp::Request>();

	request->int_supp_mode.int_supp_mode_type = static_cast<uint8_t>(prm.mode);
	request->int_supp_prm_m = prm.prm_m.value;
	request->int_supp_prm_a1 = prm.prm_a1.value;
	request->int_supp_prm_a2 = prm.prm_a2.value;
	request->int_supp_prm_a3 = prm.prm_a3.value;

	clt_res_.wait = false;
	clt_set_int_supp_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetIntSupp, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetIntSupp(
	rclcpp::Client<tof_camera_interface::srv::SetIntSupp>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setIrDarkThreshold(uint16_t th)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetIrDarkThreshold::Request>();

	request->ir_threshold = th;

	clt_res_.wait = false;
	clt_set_ir_dark_threshold_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetIrDarkThreshold, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetIrDarkThreshold(
	rclcpp::Client<tof_camera_interface::srv::SetIrDarkThreshold>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setRawSatThreshold(uint16_t th)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetRawSatThreshold::Request>();

	request->raw_threshold = th;

	clt_res_.wait = false;
	clt_set_raw_sat_threshold_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetRawSatThreshold, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetRawSatThreshold(
	rclcpp::Client<tof_camera_interface::srv::SetRawSatThreshold>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPlayCtrl(uint8_t cmd, uint32_t time)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPlayCtrl::Request>();

	request->cmd = cmd;
	request->time = time;

	clt_res_.wait = false;
	clt_set_play_ctrl_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetPlayCtrl, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetPlayCtrl(
	rclcpp::Client<tof_camera_interface::srv::SetPlayCtrl>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPlayTarget(const std::string& directory)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPlayTarget::Request>();

	request->directory = directory;

	clt_res_.wait = false;
	clt_set_play_target_->async_send_request(request,
		std::bind(&SampleViewerNode::cbCltSetPlayTarget, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbCltSetPlayTarget(
	rclcpp::Client<tof_camera_interface::srv::SetPlayTarget>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getDevInfo(krm::DeviceInfo& dev_info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetDevInfo::Request>();

	clt_res_.wait = false;
	clt_get_dev_info_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetDevInfo, this, std::placeholders::_1));

	if (waitSrvRes()) {
		dev_info = clt_res_.srv_result.dev_info;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetDevInfo(
	rclcpp::Client<tof_camera_interface::srv::GetDevInfo>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::DeviceInfo* dev_info = &clt_res_.srv_result.dev_info;
		tof_camera_interface::msg::DeviceInfo msg_dev_info = future.get()->dev_info;
		dev_info->hw_kind = msg_dev_info.hw_kind;
		dev_info->serial_no = msg_dev_info.serial_no;
		dev_info->map_ver.major = msg_dev_info.map_ver.major;
		dev_info->map_ver.minor = msg_dev_info.map_ver.minor;
		dev_info->map_ver.rev = msg_dev_info.map_ver.rev;
		dev_info->firm_ver.major = msg_dev_info.firm_ver.major;
		dev_info->firm_ver.minor = msg_dev_info.firm_ver.minor;
		dev_info->firm_ver.rev = msg_dev_info.firm_ver.rev;
		dev_info->adjust_no = msg_dev_info.adjust_no;
		dev_info->ld_wave = msg_dev_info.ld_wave;
		dev_info->ld_enable = msg_dev_info.ld_enable;
		dev_info->correct_calib = msg_dev_info.correct_calib;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getCamFov(krm::CamFov& fov)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetFov::Request>();

	clt_res_.wait = false;
	clt_get_fov_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetCamFov, this, std::placeholders::_1));

	if (waitSrvRes()) {
		fov = clt_res_.srv_result.cam_fov;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetCamFov(
	rclcpp::Client<tof_camera_interface::srv::GetFov>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::CamFov* fov = &clt_res_.srv_result.cam_fov;
		tof_camera_interface::msg::CamFov msg_cam_fov = future.get()->cam_fov;
		fov->horz = msg_cam_fov.horz;
		fov->vert = msg_cam_fov.vert;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getExtTriggerType(uint8_t& type)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetExtTriggerType::Request>();

	clt_res_.wait = false;
	clt_get_ext_trigger_type_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetExtTriggerType, this, std::placeholders::_1));

	if (waitSrvRes()) {
		type = clt_res_.srv_result.ext_trg_type;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetExtTriggerType(
	rclcpp::Client<tof_camera_interface::srv::GetExtTriggerType>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.ext_trg_type = future.get()->type.ext_trigger_type;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getExtTriggerOffset(uint8_t& offset)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetExtTriggerOffset::Request>();

	clt_res_.wait = false;
	clt_get_ext_trigger_offset_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetExtTriggerOffset, this, std::placeholders::_1));

	if (waitSrvRes()) {
		offset = clt_res_.srv_result.ext_trg_offset;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetExtTriggerOffset(
	rclcpp::Client<tof_camera_interface::srv::GetExtTriggerOffset>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.ext_trg_offset = future.get()->offset;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getModeList(krm::ModeList& mode_list)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetModeList::Request>();

	clt_res_.wait = false;
	clt_get_mode_list_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetModeList, this, std::placeholders::_1));

	if (waitSrvRes()) {
		mode_list = clt_res_.srv_result.mode_list;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetModeList(
	rclcpp::Client<tof_camera_interface::srv::GetModeList>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::ModeList& mode_list = clt_res_.srv_result.mode_list;
		krm::ModeInfo*	mode_info = nullptr;
		tof_camera_interface::msg::ModeInfo* msg_mode_info;
		krm::ImgOutKind out_kind;
		mode_list.resize(future.get()->mode_list.size());
		for (uint8_t i = 0; i < mode_list.size(); i++) {
			mode_info = &mode_list[i];
			msg_mode_info = &future.get()->mode_list[i];
			mode_info->id = msg_mode_info->id;
			mode_info->description = msg_mode_info->description;
			mode_info->img_out.clear();
			for (auto msg_out_kind : msg_mode_info->img_out) {
				out_kind = static_cast<krm::ImgOutKind>(msg_out_kind.img_out_kind);
				mode_info->img_out.push_back(out_kind);
			}
			mode_info->dist_range.max = msg_mode_info->dist_range.max;
			mode_info->dist_range.min = msg_mode_info->dist_range.min;
			mode_info->fps = msg_mode_info->fps;
			mode_info->thin_w = msg_mode_info->thin_w;
			mode_info->thin_h = msg_mode_info->thin_h;
			mode_info->crop.x = msg_mode_info->crop.x;
			mode_info->crop.y = msg_mode_info->crop.y;
			mode_info->light_times = msg_mode_info->light_times;
			mode_info->range_calib = msg_mode_info->range_calib;
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getMode(uint8_t& mode)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetMode::Request>();

	clt_res_.wait = false;
	clt_get_mode_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetMode, this, std::placeholders::_1));

	if (waitSrvRes()) {
		mode = clt_res_.srv_result.mode;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetMode(
	rclcpp::Client<tof_camera_interface::srv::GetMode>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.mode = future.get()->mode;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getImgKinds(krm::ImgOutKind& img_out)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetImgKinds::Request>();

	clt_res_.wait = false;
	clt_get_img_kinds_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetImgKinds, this, std::placeholders::_1));

	if (waitSrvRes()) {
		img_out = clt_res_.srv_result.img_out;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetImgKinds(
	rclcpp::Client<tof_camera_interface::srv::GetImgKinds>::SharedFuture future)
{
	bool ret = future.get()->result;
	RCLCPP_INFO(this->get_logger(), __func__);

	if (ret) {
		krm::ImgOutKind kind;
		switch (future.get()->img_out.img_out_kind) {
		case tof_camera_interface::msg::ImgOutKind::OUT_IMG_DEPTH:			kind = krm::OUT_IMG_DEPTH;			break;
		case tof_camera_interface::msg::ImgOutKind::OUT_IMG_IR:				kind = krm::OUT_IMG_IR;				break;
		case tof_camera_interface::msg::ImgOutKind::OUT_IMG_DEPTH_IR:		kind = krm::OUT_IMG_DEPTH_IR;		break;
		case tof_camera_interface::msg::ImgOutKind::OUT_IMG_DEPTH_IR_RAW:	kind = krm::OUT_IMG_DEPTH_IR_RAW;	break;
		case tof_camera_interface::msg::ImgOutKind::OUT_IMG_RAW:			kind = krm::OUT_IMG_RAW;			break;
		default:	ret = false;	break;
		}
		if (ret) {
			clt_res_.srv_result.img_out = kind;
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getImgFormat(krm::ImageFormats& fmts)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetImgFormat::Request>();

	clt_res_.wait = false;
	clt_get_img_format_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetImgFormat, this, std::placeholders::_1));

	if (waitSrvRes()) {
		fmts = clt_res_.srv_result.img_fmts;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetImgFormat(
	rclcpp::Client<tof_camera_interface::srv::GetImgFormat>::SharedFuture future)
{
	bool ret = future.get()->result;
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->img_fmts.data.size() != krm::IMG_KINDS) {
		RCLCPP_ERROR(this->get_logger(), "%s : size mismatch(%zu:%u)", __func__,
					future.get()->img_fmts.data.size(), krm::IMG_KINDS);
		ret = false;
	}
	if (ret) {
		krm::ImageFormat* img_fmt;
		tof_camera_interface::msg::ImageFormat*	msg_fmt;
		tof_camera_interface::msg::ImageFormats msg_fmts = future.get()->img_fmts;
		for (uint8_t i = 0; i < krm::IMG_KINDS; i++) {
			msg_fmt = &msg_fmts.data[i];
			img_fmt = &clt_res_.srv_result.img_fmts[i];
			img_fmt->set(msg_fmt->width, msg_fmt->height, msg_fmt->bpp);
			img_fmt->active_h = msg_fmt->active_h;
			img_fmt->active_w = msg_fmt->active_w;
			img_fmt->active_start.x = msg_fmt->active_start.x;
			img_fmt->active_start.y = msg_fmt->active_start.y;
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = ret;
	notifyCbFin();
}

bool SampleViewerNode::getPostFiltInfo(bool& cam_med_filt, bool& cam_bil_filt, bool& cam_fly_p_filt)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetPostFiltInfo::Request>();

	clt_res_.wait = false;
	clt_get_post_filt_info_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetPostFiltInfo, this, std::placeholders::_1));

	if (waitSrvRes()) {
		cam_med_filt = clt_res_.srv_result.pf_info.cam_med_filt;
		cam_bil_filt = clt_res_.srv_result.pf_info.cam_bil_filt;
		cam_fly_p_filt = clt_res_.srv_result.pf_info.cam_fly_p_filt;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetPostFiltInfo(
	rclcpp::Client<tof_camera_interface::srv::GetPostFiltInfo>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		tof_camera_interface::msg::PostFiltInfo pf_info = future.get()->post_filt_info;
		clt_res_.srv_result.pf_info.cam_med_filt = pf_info.cam_med_filt;
		clt_res_.srv_result.pf_info.cam_bil_filt = pf_info.cam_bil_filt;
		clt_res_.srv_result.pf_info.cam_fly_p_filt = pf_info.cam_fly_p_filt;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getLensInfo(krm::LensInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetLensInfo::Request>();

	clt_res_.wait = false;
	clt_get_lens_info_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetLensInfo, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.lens_info;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetLensInfo(
	rclcpp::Client<tof_camera_interface::srv::GetLensInfo>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::LensInfo* lens_info = &clt_res_.srv_result.lens_info;
		tof_camera_interface::msg::LensInfo msg_lens_info = future.get()->lens_info;
		lens_info->cam_dist = msg_lens_info.cam_dist;
		lens_info->lens_calib = msg_lens_info.lens_calib;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getLightTimes(krm::LightTimesInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetLightTimes::Request>();

	clt_res_.wait = false;
	clt_get_light_times_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetLightTimes, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.light_times;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetLightTimes(
	rclcpp::Client<tof_camera_interface::srv::GetLightTimes>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::LightTimesInfo* info = &clt_res_.srv_result.light_times;
		tof_camera_interface::msg::MinMaxValue32 light_times = future.get()->light_times;
		info->min = light_times.min;
		info->max = light_times.max;
		info->count = light_times.value;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getAEState(bool& enable)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetAEState::Request>();

	clt_res_.wait = false;
	clt_get_ae_state_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetAEState, this, std::placeholders::_1));

	if (waitSrvRes()) {
		enable = clt_res_.srv_result.ae_state;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetAEState(
	rclcpp::Client<tof_camera_interface::srv::GetAEState>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.ae_state = future.get()->enable;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getAEInterval(krm::AEIntervalInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetAEInterval::Request>();

	clt_res_.wait = false;
	clt_get_ae_interval_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetAEInterval, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.ae_interval;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetAEInterval(
	rclcpp::Client<tof_camera_interface::srv::GetAEInterval>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::AEIntervalInfo* info = &clt_res_.srv_result.ae_interval;
		tof_camera_interface::msg::MinMaxValue8 ae_interval = future.get()->interval;
		info->min = ae_interval.min;
		info->max = ae_interval.max;
		info->interval = ae_interval.value;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getRawSatThreshold(krm::SignalThresholdInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetRawSatThreshold::Request>();

	clt_res_.wait = false;
	clt_get_raw_sat_th_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetRawSatThreshold, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.raw_sat_th;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetRawSatThreshold(
	rclcpp::Client<tof_camera_interface::srv::GetRawSatThreshold>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::SignalThresholdInfo* info = &clt_res_.srv_result.raw_sat_th;
		tof_camera_interface::msg::MinMaxValue16 raw_sat_th = future.get()->raw_sat_th;
		info->min = raw_sat_th.min;
		info->max = raw_sat_th.max;
		info->threshold = raw_sat_th.value;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getIrDarkThreshold(krm::SignalThresholdInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetIrDarkThreshold::Request>();

	clt_res_.wait = false;
	clt_get_ir_dark_th_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetIrDarkThreshold, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.ir_dark_th;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetIrDarkThreshold(
	rclcpp::Client<tof_camera_interface::srv::GetIrDarkThreshold>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::SignalThresholdInfo* info = &clt_res_.srv_result.ir_dark_th;
		tof_camera_interface::msg::MinMaxValue16 ir_dark_th = future.get()->ir_dark_th;
		info->min = ir_dark_th.min;
		info->max = ir_dark_th.max;
		info->threshold = ir_dark_th.value;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getIntSuppInfo(krm::IntSuppInfo& info)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetIntSuppInfo::Request>();

	clt_res_.wait = false;
	clt_get_int_supp_info_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetIntSuppInfo, this, std::placeholders::_1));

	if (waitSrvRes()) {
		info = clt_res_.srv_result.int_supp_info;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetIntSuppInfo(
	rclcpp::Client<tof_camera_interface::srv::GetIntSuppInfo>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		krm::IntSuppInfo* info = &clt_res_.srv_result.int_supp_info;
		tof_camera_interface::msg::IntSuppInfo int_supp_info = future.get()->int_supp_info;
		info->mode = static_cast<krm::IntSuppModeType>(int_supp_info.mode.int_supp_mode_type);
		info->prm_m.min = int_supp_info.prm_m.min;
		info->prm_m.max = int_supp_info.prm_m.max;
		info->prm_m.value = int_supp_info.prm_m.value;
		info->prm_a1.min = int_supp_info.prm_a1.min;
		info->prm_a1.max = int_supp_info.prm_a1.max;
		info->prm_a1.value = int_supp_info.prm_a1.value;
		info->prm_a2.min = int_supp_info.prm_a2.min;
		info->prm_a2.max = int_supp_info.prm_a2.max;
		info->prm_a2.value = int_supp_info.prm_a2.value;
		info->prm_a3.min = int_supp_info.prm_a3.min;
		info->prm_a3.max = int_supp_info.prm_a3.max;
		info->prm_a3.value = int_supp_info.prm_a3.value;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getPlayTarget(std::string& target)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetPlayTarget::Request>();

	clt_res_.wait = false;
	clt_get_play_target_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetPlayTarget, this, std::placeholders::_1));

	if (waitSrvRes()) {
		target = clt_res_.srv_result.play_target;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetPlayTarget(
	rclcpp::Client<tof_camera_interface::srv::GetPlayTarget>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.play_target = future.get()->directory;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getPlayTime(krm::PlayBack::PlayTime& play_time)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetPlayTime::Request>();

	clt_res_.wait = false;
	clt_get_play_time_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetPlayTime, this, std::placeholders::_1));

	if (waitSrvRes()) {
		play_time = clt_res_.srv_result.play_time;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetPlayTime(
	rclcpp::Client<tof_camera_interface::srv::GetPlayTime>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.play_time.total = future.get()->play_time.total;
		clt_res_.srv_result.play_time.current = future.get()->play_time.current;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::getPlayStatus(krm::PlayBack::PlayStatus& status)
{
	auto request = std::make_shared<tof_camera_interface::srv::GetPlayStatus::Request>();

	clt_res_.wait = false;
	clt_get_play_status_->async_send_request(request,
		std::bind(&SampleViewerNode::cbGetPlayStatus, this, std::placeholders::_1));

	if (waitSrvRes()) {
		status = clt_res_.srv_result.play_status;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbGetPlayStatus(
	rclcpp::Client<tof_camera_interface::srv::GetPlayStatus>::SharedFuture future)
{
	bool ret = future.get()->result;
	RCLCPP_INFO(this->get_logger(), __func__);

	if (ret) {
		krm::PlayBack::PlayState st;
		switch (future.get()->state) {
		case tof_camera_interface::srv::GetPlayStatus::Response::STOPPED:	st = krm::PlayBack::STOPPED;	break;
		case tof_camera_interface::srv::GetPlayStatus::Response::PLAYING:	st = krm::PlayBack::PLAYING;	break;
		case tof_camera_interface::srv::GetPlayStatus::Response::PAUSE:		st = krm::PlayBack::PAUSE;		break;
		case tof_camera_interface::srv::GetPlayStatus::Response::FAST:		st = krm::PlayBack::FAST;		break;
		case tof_camera_interface::srv::GetPlayStatus::Response::SLOW:		st = krm::PlayBack::SLOW;		break;
		default:	ret = false;	break;
		}
		if (ret) {
			clt_res_.srv_result.play_status.state = st;
			clt_res_.srv_result.play_status.playing_fps = future.get()->playing_fps;
		}
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::psblPostFilt(uint8_t type, bool& possible)
{
	auto request = std::make_shared<tof_camera_interface::srv::PsblPostFilt::Request>();

	request->filt_type = type;

	clt_res_.wait = false;
	clt_psbl_post_filt_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPsblPostFilt, this, std::placeholders::_1));

	if (waitSrvRes()) {
		possible = clt_res_.srv_result.psbl_pf;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbPsblPostFilt(
	rclcpp::Client<tof_camera_interface::srv::PsblPostFilt>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.psbl_pf = future.get()->possible;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPostFilt(uint8_t type, bool enable)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPostFilt::Request>();

	request->filt_type = type;
	request->enable = enable;

	clt_res_.wait = false;
	clt_set_post_filt_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPostFilt, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbPostFilt(
	rclcpp::Client<tof_camera_interface::srv::SetPostFilt>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPostFiltPrm(const PostFilterPrm& prm)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPostFiltPrm::Request>();

	request->param.median_ksize = prm.median_ksize;
	request->param.bil_ksize = prm.bil_ksize;
	request->param.bil_sigma_depth = prm.bil_sigma_depth;
	request->param.bil_sigma_ir = prm.bil_sigma_ir;
	request->param.bil_sigma_space = prm.bil_sigma_space;
	request->param.flyp_ksize = prm.flyp_ksize;
	request->param.flyp_log = prm.flyp_log;
	request->param.flyp_thr = prm.flyp_thr;
	request->param.flyp_fast_proc = prm.flyp_fast_proc;

	clt_res_.wait = false;
	clt_set_post_filt_prm_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPostFiltPrm, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbPostFiltPrm(
	rclcpp::Client<tof_camera_interface::srv::SetPostFiltPrm>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::psblLensConv(uint8_t type, bool& possible)
{
	auto request = std::make_shared<tof_camera_interface::srv::PsblLensConv::Request>();

	request->conv_type = type;

	clt_res_.wait = false;
	clt_psbl_lens_conv_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPsblLensConv, this, std::placeholders::_1));

	if (waitSrvRes()) {
		possible = clt_res_.srv_result.psbl_lens;
	} else {
		return false;
	}
	return true;
}

void SampleViewerNode::cbPsblLensConv(
	rclcpp::Client<tof_camera_interface::srv::PsblLensConv>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
		clt_res_.srv_result.psbl_lens = future.get()->possible;
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setLensConv(uint8_t type, bool enable)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetLensConv::Request>();

	request->conv_type = type;
	request->enable = enable;

	clt_res_.wait = false;
	clt_set_lens_conv_->async_send_request(request,
		std::bind(&SampleViewerNode::cbLensConv, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbLensConv(
	rclcpp::Client<tof_camera_interface::srv::SetLensConv>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPcdColor(uint8_t color)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPcdColor::Request>();

	request->color = color;

	clt_res_.wait = false;
	clt_set_pcd_color_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPcdColor, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbPcdColor(
	rclcpp::Client<tof_camera_interface::srv::SetPcdColor>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setPcdPos(const PosOrgRotation& pos)
{
	auto request = std::make_shared<tof_camera_interface::srv::SetPcdPos::Request>();

	request->pos.offset_x = pos.offset.x;
	request->pos.offset_y = pos.offset.y;
	request->pos.offset_z = pos.offset.z;
	request->pos.rotation_pitch = pos.rotation.pitch;
	request->pos.rotation_yaw = pos.rotation.yaw;
	request->pos.rotation_roll = pos.rotation.roll;

	clt_res_.wait = false;
	clt_set_pcd_pos_->async_send_request(request,
		std::bind(&SampleViewerNode::cbPcdPos, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbPcdPos(
	rclcpp::Client<tof_camera_interface::srv::SetPcdPos>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}

bool SampleViewerNode::setRecCtrl(uint8_t cmd, const std::string dir, uint32_t save_frms, uint16_t packing_frms,
								bool is_crct_dist, bool is_filt_med, bool is_filt_bil, bool is_filt_fly_p)
{
	auto request = std::make_shared<tof_camera_interface::srv::RecordCtrl::Request>();

	request->cmd = cmd;
	request->directory = dir;
	request->save_frames = save_frms;
	request->packing_frames = packing_frms;
	request->is_crct_dist = is_crct_dist;
	request->is_filt_med = is_filt_med;
	request->is_filt_bil = is_filt_bil;
	request->is_filt_fly_p = is_filt_fly_p;

	clt_res_.wait = false;
	clt_set_rec_ctrl_->async_send_request(request,
		std::bind(&SampleViewerNode::cbSetRecCtrl, this, std::placeholders::_1));

	return waitSrvRes();
}

void SampleViewerNode::cbSetRecCtrl(rclcpp::Client<tof_camera_interface::srv::RecordCtrl>::SharedFuture future)
{
	RCLCPP_INFO(this->get_logger(), __func__);

	if (future.get()->result) {
	} else {
		RCLCPP_ERROR(this->get_logger(), "%s : %d", __func__, future.get()->result);
	}

	clt_res_.result = future.get()->result;
	notifyCbFin();
}


bool SampleViewerNode::waitSrvRes(void)
{
	std::unique_lock<std::mutex> lock(mtx_);
	cond_.wait(lock, [this] { return clt_res_.wait; });
	return clt_res_.result;
}

void SampleViewerNode::notifyCbFin(void)
{
	std::unique_lock<std::mutex> lock(mtx_);
	clt_res_.wait = true;
	cond_.notify_one();
}

void SampleViewerNode::guiThread(void)
{
	int display_w, display_h;

	loadConfig();

	if (!init_device()) { return; }

	initialize();

	while (running_ && !glfwWindowShouldClose(win_)) {
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		if (!hide_ctrl_panel_) { drawCtrlPanel(); }
		drawViewPanel();
		drawStatusPanel();
		drawPopUpMessage();
		dir_browser_.Display();
		file_browser_.Display();

		// Rendering
		ImGui::Render();

		glfwGetFramebufferSize(win_, &display_w, &display_h);
		glViewport(0, 0, display_w, display_h);
		glClear(GL_COLOR_BUFFER_BIT);
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(win_);
	}

	saveConfig();
	terminate();
}

}	// namespace tof_camera_example

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_example::SampleViewerNode)
