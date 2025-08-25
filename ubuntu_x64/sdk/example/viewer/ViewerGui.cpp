/*------------------------------------------------------------------*/
/// @file		ViewerGui.cpp
/// @brief		Viewer Graphic User Interface
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <sstream>
#include <fstream>

#include "imgui_internal.h"

#include "ViewerGui.h"
#include "RecordThread.h"
#include "LensConvThread.h"
#include "PostFilterThread.h"
#include "RegCsvList.h"
#include "SaveSnapshot.h"
#include "CommonLog.h"

namespace krm
{

Point2d ViewerGui::mouse_pos_raw_ = {UINT16_MAX, UINT16_MAX};
bool	ViewerGui::mouse_enter_raw_ = false;

ViewerGui::ViewerGui(void) :
	running_(false), win_(NULL),
	dir_browser_(DIR_BROW), file_browser_(FILE_BROW), browser_target_(FILE_BROWS_NONE),
	cam_obj_(nullptr), play_obj_(nullptr), mode_(0), cur_mode_info_(nullptr),
	out_kind_(OUT_IMG_DEPTH_IR), light_times_({0, 100U, 50U}),
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
	cam_reg_tgt_w_(CAM_REG_TGT_W), cam_reg_idt_(CAM_REG_IDT),
	cam_reg_w_(CAM_REG_W), cam_rec_len_w_(CAM_REC_LEN_W),
	cap_dev_con_(false), cam_dev_open_(false), cam_cap_disable_(false),
	cam_mode_sel_(0), ini_mode_(0), cam_image_idx_(0), cam_disable_raw_(true),
	cam_light_slid_(50), cam_disable_light_time_(true), cam_light_cnt_img_(IMG_DEPTH),
	cam_ae_enable_(false), cam_ae_interval_({0, 255U, 4U}),
	cam_raw_sat_th_({0, 4095U, 3500U}), cam_ir_dark_th_({0, 4095U, 20U}),
	cam_ext_trg_list_({"Standalone", "Slave", "Master"}), cam_ext_trigger_(EXT_TRG_STANDALONE),
	cam_ext_trg_disable_(false), cam_ext_trg_offset_(0),
	cam_int_supp_mode_list_({"Off", "Manual", "Auto"}),
	cam_int_supp_info_({INT_SUPP_MODE_OFF, {0, 255U, 0}, {0, 255U, 31U}, {0, 255U, 65U}, {0, 7U, 4U}}),
	cam_rst_dev_(false), cam_reg_target_idx_(0), recording_(false), cam_rec_packing_(60),
	/* PlayBack */
	play_time_w_(PLAY_TIME_W), play_time_space_(PLAY_TIME_SPACE),
	play_jump_w_(PLAY_JUMP_W), play_jump_idt_(PLAY_JUMP_IDT), play_time_idt_(PLAY_TIME_IDT),
	play_cap_disable_(false), play_pausing_(false),
	play_jump_time_sec_(10), play_jump_time_(0), play_jump_val_(0),
	play_time_(""), play_time_total_(), play_con_timer_(0),
	/* PostFilter */
	pstf_ofst_w_(PSTF_OFST_W),
	pstf_inp_w_(PSTF_INP_W),
	pstf_ksize_map_({3, 5}),
	pstf_ksize_list_({"3", "5"}),
	pstf_method_list_({"Differential", "Ratio"}), pstf_priority_list_({"Accuracy" ,"Speed"}),
	pstf_enable_medf_(false), pstf_medf_check_disable_(false),
	pstf_enable_bilf_(false), pstf_bilf_check_disable_(false),
	pstf_enable_flypf_(false), pstf_flypf_check_disable_(false),
	pstf_prm_({3U, 3U, 500.0, 100.0, 1.0, 3U , true, 130U, true}),

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
	/* Snapshot */
	snap_frms_idx_(0), snap_file_fmt_(SNAP_FILE_BIN), snap_took_frms_(0), snap_taking_(false),
	snap_disable_take_(true), snap_disable_exit_(true), snap_disable_save_(true),
	/* Camera Parameter */
	cam_prm_disabled_(true),
	cam_prm_w_selected_(0), cam_prm_w_part_(false),
	cam_prm_r_selected_(0), cam_prm_r_part_(false),
	/* Firmware */
	firmware_w_selected_(0),
	firmware_w_kind_(CAM_PRM_ARTIX_CONF),
	firmware_r_selected_(0),
	firmware_r_kind_(CAM_PRM_ARTIX_CONF),
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
	DrawRaws raws;
	PlayBack::PlayTime time;
	time.current = 0;
	time.total = 0;

	updatePlayingTime(time, true);

	btn_color_[0] = ImColor::HSV(3.0F / 7.0F, 0.6F, 0.6F).Value;
	btn_color_[1] = ImColor::HSV(3.0F / 7.0F, 0.7F, 0.7F).Value;
	btn_color_[2] = ImColor::HSV(3.0F / 7.0F, 0.8F, 0.8F).Value;

	grid_color_ = IM_COL32(255, 255, 255, 255);

	mode_list_.clear();
	for (auto& fmt : img_fmts_) { fmt.set(); }
	cam_reg_file_path_c_.fill(0);
	cam_reg_file_path_.clear();
	cam_rec_dir_path_c_.fill(0);
	cam_rec_dir_path_.clear();
	cam_reg_addr_.fill(0);
	std::snprintf(cam_reg_addr_.data(), cam_reg_addr_.size(), "0");
	cam_reg_val_.fill(0);
	std::snprintf(cam_reg_val_.data(), cam_reg_val_.size(), "0");
	cam_rec_len_.fill(0);
	std::snprintf(cam_rec_len_.data(), cam_rec_len_.size(), "1");
	cam_rec_dir_path_c_.fill(0);
	play_target_path_.clear();
	for (auto& t : play_jump_) {
		t.fill(0);
		std::snprintf(t.data(), t.size(), "0");
	}
	play_target_path_c_.fill(0);
	cam_prm_w_file_path_c_.fill(0);
	cam_prm_w_file_path_.clear();
	cam_prm_w_id_.fill(0);
	std::snprintf(cam_prm_w_id_.data(), cam_prm_w_id_.size(), "0");
	cam_prm_r_id_.fill(0);
	std::snprintf(cam_prm_r_id_.data(), cam_prm_r_id_.size(), "0");
	cam_prm_r_dir_path_c_.fill(0);
	cam_prm_r_dir_path_.clear();
	img_win_pos_.fill(ImVec2(0,0));

	makeFpsString(UINT16_MAX, state_fps_);
	makeFpsString(UINT16_MAX, play_frame_fps_);

	cam_mode_list_.clear();
	cam_image_list_.clear();
	cam_reg_target_list_.clear();
	reg_devs_.clear();
	view_image_list_.clear();
	view_image_list_.push_back("None");
	view_image_pair_.clear();
	show_img_.fill(DISP_NONE);
	for (auto & fmt : show_fmt_) { fmt.set(); }

	snap_frms_list_.resize(SNAP_FRMS_NUM);
	for (uint8_t i = 0; i < SNAP_FRMS_NUM; i++) { snap_frms_list_[i] = std::to_string(SNAP_FRMS[i]); }
	snap_tgt_dir_path_c_.fill(0);
	snap_tgt_dir_path_.clear();

	draw_imgs_[IMG_DEPTH] = std::make_shared<DrawDepth>(color_table_);
	draw_imgs_[IMG_IR]    = std::make_shared<DrawGray>(gray_table_);
	for (uint8_t i = IMG_RAW1; i <= IMG_RAW4; i++) {
		draw_imgs_[i] = std::make_shared<DrawRaw>(i - IMG_RAW1, gray_table_);
		raws[i - IMG_RAW1] = std::dynamic_pointer_cast<DrawRaw>(draw_imgs_[i]);
	}
	draw_imgs_[IMG_PCD]   = std::make_shared<DrawPcd>(color_table_, gray_table_);
	color_table_->setColorRange(view_depth_range_);
	color_table_->createColorBar(view_depth_range_, view_dpt_bar_w_, view_dpt_bar_h_, color_bar_);
	gray_table_->setGain(view_gray_gain_);
	gray_table_->setGamma(view_gray_gamma_);
	raw_win_ = std::make_shared<DisplayRaw>(raws);
	draw_points_.clear();
	draw_image_.clear();
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(view_radio_button_ != PCD_C_DEPTH);
}

ViewerGui::~ViewerGui(void)
{

}

void ViewerGui::run(void)
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

	terminate();
	saveConfig();
}

bool ViewerGui::initialize(void)
{
	glfwSetErrorCallback(ViewerGui::gl_error);
	if (glfwInit() == GL_FALSE) { return false; }
	running_ = createWindow();
	return running_;
}

void ViewerGui::terminate(void)
{
	glfwSetErrorCallback(NULL);
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	destroyWindow();
	glfwTerminate();
}

void ViewerGui::gl_error(int error, const char* description)
{
	(void)error;
	(void)description;
	LOG_WRN("Glfw Error %d: %s\n", error, description);
}

bool ViewerGui::init_device(void)
{
	Result ret;
	PlayBack::ConfigParam param;
	param.path = play_target_path_;
	std::vector<PlFw*>	pl;

	cam_obj_ = std::make_unique<PlFw>(C11_USB, nullptr, ret);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to create Camera");
		showMessage(MSG_WARN, "Failed to initialization");
		return false;
	}
	pl.push_back(cam_obj_.get());
	play_obj_ = std::make_unique<PlFw>(PLAYBACK, &param, ret);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to create PlayBack");
		showMessage(MSG_WARN, "Failed to initialization");
		return false;
	}
	pl.push_back(play_obj_.get());

	for (uint8_t i = 0; i < static_cast<uint8_t>(proc_ids_.size()); i++) {
		std::shared_ptr<EvtThread>	filter = std::make_shared<PostFilterThread>(pstf_enable_medf_, pstf_enable_bilf_, pstf_enable_flypf_);
		std::shared_ptr<EvtThread>	lens = std::make_shared<LensConvThread>(true,pst_dist_enable_);
		std::shared_ptr<EvtThread>	rec = std::make_shared<RecordThread>();
		std::shared_ptr<EvtThread>	draw = std::make_shared<DrawThread>(draw_imgs_);
		proc_ids_[i].resize(PLT_NUM);
		pl[i]->addPlProc(filter, 0, proc_ids_[i][PLT_POSTFILTER]);
		pl[i]->addPlProc(lens, 0, proc_ids_[i][PLT_LENSCONV]);
		pl[i]->addPlProc(rec,  0, proc_ids_[i][PLT_RECORD]);
		pl[i]->addPlProc(draw, 0, proc_ids_[i][PLT_DRAW]);
	}

	reloadDevice();
	return true;
}

void ViewerGui::reloadDevice(void)
{
	Result ret;

	dev_list_.clear();
	dev_list_.push_back("Please select target");
	cam_list_.clear();

	ret = cam_obj_->getCamDeviceList(cam_list_);
	if (ret == SUCCESS) {
		for (auto dev : cam_list_) {
			dev_list_.push_back(std::to_string(dev.id) + " : " + dev.name);
		}
	} else {
		LOG_ERR("Camera::getDeviceList : %d\n", ret);
	}
	ret = play_obj_->getCamDeviceList(play_list_);
	if (ret == SUCCESS) {
		dev_list_.push_back(play_list_[0].name);
	} else {
		LOG_ERR("PlayBack::getDeviceList : %d\n", ret);
	}
}

PlFw* ViewerGui::getCamera(void)
{
	return (tgt_cam_idx_ < PLAYBACK_INDEX) ? cam_obj_.get() : play_obj_.get();
}

uint16_t ViewerGui::getProcId(PlThreadKind thd)
{
	uint8_t i = (tgt_cam_idx_ < PLAYBACK_INDEX) ? 0U : 1U;
	return proc_ids_[i][thd];
}

bool ViewerGui::openDevice(void)
{
	Result ret;
	PlFw* camera = getCamera();
	uint16_t dev_id;
	bool world_coord = !pst_pcd_cam_coord_;
	PcdColorKind pcd_color = (view_radio_button_ == PCD_C_DEPTH) ? PCD_COLOR_NONE : PCD_COLOR_IR;

	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		dev_id = cam_list_[tgt_cam_idx_].id;
	} else {
		dev_id = play_list_[0].id;
	}

	ret = camera->wakeupPl(dev_id);
	if (ret == SUCCESS) {
		updateDeviceInfo();
		if (tgt_cam_idx_ < PLAYBACK_INDEX) {
			cam_cap_disable_ = false;
			cap_dev_con_ = false;
			// set initial motion mode
			if (ini_mode_ != mode_) {
				for (uint8_t i = 0; i < mode_list_.size(); i++) {
					if (ini_mode_ == mode_list_[i].id) {
						if (setDeviceProperty(CMD_MODE, &ini_mode_)) {
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
		}
		view_disable_winsize_ = true;
		// notify current settings
		if (!pst_dist_check_disable_) {
			(void)camera->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_DIST, &pst_dist_enable_);
		}
		(void)camera->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_KIND, &world_coord);
		notifyPcdPrm();
		(void)camera->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_COLOR, &pcd_color);
		if (!pstf_medf_check_disable_) {
			(void)camera->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_MEDF, &pstf_enable_medf_);
		}
		if (!pstf_bilf_check_disable_) {
			(void)camera->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_BILF, &pstf_enable_bilf_);
		}
		if (!pstf_flypf_check_disable_) {
			(void)camera->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_FLYPF, &pstf_enable_flypf_);
		}
		(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
	} else {
		LOG_ERR("failed to openDevice : %d\n", ret);
		return false;
	}

	return true;
}

void ViewerGui::closeDevice(void)
{
	Result ret;
	if (state_cam_ != ST_STOPPED) { stopCapture(); }
	if (recording_) { stopRecord(); }
	if (snap_taking_) { exitSnapshot(); }
	ret = getCamera()->shutdownPl();
	if (ret != SUCCESS) {
		LOG_ERR("Failed to close device : %d\n", ret);
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
	cam_prm_disabled_ = true;
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

void ViewerGui::startCapture(void)
{
	Result ret = getCamera()->startCapture();
	if (ret != SUCCESS) {
		LOG_ERR("Failed to start capture : %d\n", ret);
		if ((tgt_cam_idx_ == PLAYBACK_INDEX) && (ret == ERR_NOT_EXIST)) {
			showMessage(MSG_ATTENTION, "PlayBack target file is not exist");
		} else {
			showMessage(MSG_WARN, "Failed to Start Capture.");
		}
	} else {
		if (tgt_cam_idx_ < PLAYBACK_INDEX) {
			cam_cap_disable_ = true;
			cap_dev_con_ = true;
			cam_prm_disabled_ = true;
		} else {
			play_cap_disable_ = true;
			updatePlayInfo();
		}
		if (!recording_) { snap_disable_take_ = false; }
		stat_disable_cfg_load_ = true;
		state_cam_ = ST_STREAMING;
	}
}

void ViewerGui::stopCapture(void)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	ImVec2 pos = pnl_view_pos_;
	Result ret = getCamera()->stopCapture();
	if (ret != SUCCESS) {
		LOG_ERR("Failed to stop capture : %d\n", ret);
		showMessage(MSG_WARN, "Failed to Stop Capture.\n");
	}

	if (recording_) { stopRecord(); }
	if (snap_taking_) { exitSnapshot(); }

	raw_win_->clear();
	raw_win_->draw();
	glfwMakeContextCurrent(win_);
	for (auto& win : draw_imgs_) {
		win->clear();
	}

	for (uint8_t i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
		if (show_img_[i] == DISP_PCD) {
			updatePointCloud(pos, true);
		}
		pos.x += pnl_view_size_.x;
	}

	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		cam_cap_disable_ = false;
		cap_dev_con_ = false;
		cam_prm_disabled_ = false;
	} else {
		play_cap_disable_ = false;
		updatePlayInfo();
	}

	makeFpsString(UINT16_MAX, state_fps_);
	snap_disable_take_ = true;
	stat_disable_cfg_load_ = hide_ctrl_panel_;

	state_cam_ = ST_STOPPED;
}

bool ViewerGui::getDeviceProperty(uint16_t cmd, void* param)
{
	Result ret = getCamera()->getCamProperty(cmd, param);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to get Property(%u) : %d\n", cmd, ret);
		return false;
	}
	return true;
}

bool ViewerGui::setDeviceProperty(uint16_t cmd, void* param)
{
	Result ret = getCamera()->setCamProperty(cmd, param);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to set Property(%u) : %d\n", cmd, ret);
		return false;
	}
	return true;
}

void ViewerGui::updateDeviceInfo(void)
{
	DeviceInfo dev_info;
	CamFov fov;
	PostFiltInfo post_filt_info;
	LensInfo lens_info;
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

	if (getDeviceProperty(CMD_DEV_INFO, &dev_info)) {
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
	if (getDeviceProperty(CMD_FOV, &fov)) {
		cam_inf_list_[8] = makeDegString(fov.horz);
		cam_inf_list_[9] = makeDegString(fov.vert);
	}
	if (getDeviceProperty(CMD_POSTFILT_INFO, &post_filt_info)) {
		cam_inf_list_[10] = std::to_string(post_filt_info.cam_med_filt);
		cam_inf_list_[11] = std::to_string(post_filt_info.cam_bil_filt);
		cam_inf_list_[12] = std::to_string(post_filt_info.cam_fly_p_filt);
	}
	if (getDeviceProperty(CMD_LENS_INFO, &lens_info)) {
		cam_inf_list_[13] = std::to_string(lens_info.cam_dist);
		cam_inf_list_[14] = std::to_string(lens_info.lens_calib);
	}
	cam_ext_trg_disable_ = false;
	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		if (getDeviceProperty(CMD_EXT_TRG_TYPE, &cam_ext_trigger_)) {
			cam_ext_trg_disable_ = (cam_ext_trigger_ == EXT_TRG_SLAVE);
		}
		if (getDeviceProperty(CMD_EXT_TRG_OFFSET, &trg_offset)) {
			cam_ext_trg_offset_ = static_cast<float>(trg_offset) / 10.0F;
		}
	}

	pstf_medf_check_disable_ = false;
	pstf_bilf_check_disable_ = false;
	pstf_flypf_check_disable_ = false;
	pst_dist_check_disable_ = false;

	(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PSBL_DIST, &is_possible_dist);
	pst_dist_check_disable_ = !is_possible_dist;
	if (pst_dist_check_disable_) {
		pst_dist_enable_ = true;
		pstf_flypf_check_disable_ = true;
		pstf_bilf_check_disable_ = true;
		pstf_medf_check_disable_ = true;
	} else {
		if (!tgt_dev_enable_) {
			(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_DIST, &pst_dist_enable_);
		}
	}

	(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PSBL_FLYPF, &is_possible_flypf);
	if (pstf_flypf_check_disable_) {
		pstf_enable_flypf_ = !is_possible_flypf;
		if (!tgt_dev_enable_ && is_possible_flypf) {
			(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_FLYPF, &pstf_enable_flypf_);
		}
	} else {
		pstf_flypf_check_disable_ = !is_possible_flypf;
		if (pstf_flypf_check_disable_) {
			pstf_enable_flypf_ = true;
			pstf_bilf_check_disable_ = true;
			pstf_medf_check_disable_ = true;
		} else {
			if (!tgt_dev_enable_) {
				(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_FLYPF, &pstf_enable_flypf_);
			}
		}
	}
	(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PSBL_BILF, &is_possible_bilf);
	if (pstf_bilf_check_disable_) {
		pstf_enable_bilf_ = !is_possible_bilf;
		if (!tgt_dev_enable_ && is_possible_bilf) {
			(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_BILF, &pstf_enable_bilf_);
		}
	} else {
		pstf_bilf_check_disable_ = !is_possible_bilf;
		if (pstf_bilf_check_disable_) {
			pstf_enable_bilf_ = true;
			pstf_medf_check_disable_ = true;
		} else {
			if (!tgt_dev_enable_) {
				(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_BILF, &pstf_enable_bilf_);
			}
		}
	}

	(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PSBL_MEDF, &is_possible_medf);
	if (pstf_medf_check_disable_) {
		pstf_enable_medf_ = !is_possible_medf;
		if (!tgt_dev_enable_ && is_possible_medf) {
			(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_MEDF, &pstf_enable_medf_);
		}
	} else {
		pstf_medf_check_disable_ = !is_possible_medf;
		if (pstf_medf_check_disable_) {
			pstf_enable_medf_ = true;
		} else {
			if (!tgt_dev_enable_) {
				(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_MEDF, &pstf_enable_medf_);
			}
		}
	}

	if (getDeviceProperty(CMD_MODE_LIST, &mode_list_)) {
		if (mode_list_.size() > 0) {
			cam_mode_list_.resize(mode_list_.size());
			for (uint8_t i = 0; i < mode_list_.size(); i++) {
				cam_mode_list_[i] = makeModeString(mode_list_[i]);
			}
			updateModeInfo();
		}
	}
	cam_reg_target_list_.clear();

#ifdef DEVELOP
	if (tgt_cam_idx_ < PLAYBACK_INDEX) {
		if (getDeviceProperty(CMD_REG_DEVS, &reg_devs_)) {
			for (auto dev : reg_devs_) {
				cam_reg_target_list_.push_back(REG_TARGET_LIST[dev.target].data());
			}
		}
	} else {
		reg_devs_.clear();
	}
#endif	/* DEVELOP */
	if (fov.horz > 18000U) { grid_view_kind_ = GRID_KIND_FORCE_ORIGIN; }
	std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setFov(fov.horz);
}

void ViewerGui::updateModeInfo(void)
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

	if (getDeviceProperty(CMD_MODE, &mode_)) {
		for (uint8_t i = 0; i < static_cast<uint8_t>(mode_list_.size()); i++) {
			if (mode_ == mode_list_[i].id) {
				std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
				std::array<char, 16> fps_str;
				LOG_INF("Mode Info found : %u\n", mode_);
				cam_mode_sel_ = i;
				cur_mode_info_ = &mode_list_[i];
				cam_image_idx_ = 0;
				if (cur_mode_info_->img_out.size() > 1) {
					ImgOutKind kind;
					if (getDeviceProperty(CMD_IMG_KINDS, &kind)) {
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
					if (getDeviceProperty(CMD_LIGHT_TIMES, &light_times_)) {
						cam_light_slid_ = static_cast<int>(light_times_.count);
						cam_disable_light_time_ = false;
					}
					if (getDeviceProperty(CMD_AE_STATE, &cam_ae_enable_)) {
						if (cam_ae_enable_) {
							LOG_WRN("AE is still enable. force disable.\n");
							cam_ae_enable_ = false;
							(void)setDeviceProperty(CMD_AE_STATE, &cam_ae_enable_);
						}
					}
					(void)getDeviceProperty(CMD_AE_INTERVAL, &cam_ae_interval_);
				}
				updateColorRange(cur_mode_info_->dist_range);

				if (tgt_cam_idx_ < PLAYBACK_INDEX) {
					if (!getDeviceProperty(CMD_RAW_SAT_TH, &cam_raw_sat_th_)) {
						cam_raw_sat_th_ = {0, 0, 0};
					}
					if (!getDeviceProperty(CMD_IR_DARK_TH, &cam_ir_dark_th_)) {
						cam_ir_dark_th_ = {0, 0, 0};
					}
					if (!getDeviceProperty(CMD_INT_SUPP_INFO, &cam_int_supp_info_)) {
						cam_int_supp_info_ = {INT_SUPP_MODE_OFF, {0, 255U, 0}, {0, 255U, 0}, {0, 255U, 0}, {0, 7U, 0}};
					}
				}

				play_jump_time_ = (static_cast<uint32_t>(play_jump_time_sec_) * cur_mode_info_->fps) / 100U;
				if (tgt_cam_idx_ == PLAYBACK_INDEX) { updatePlayInfo(); }
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

void ViewerGui::updateImgKind(void)
{
	view_image_list_.clear();
	view_image_pair_.clear();
	view_image_idx_ = 0;

	if (cur_mode_info_ == nullptr) {
		LOG_ERR("Mode information is empty\n");
		return;
	}

	out_kind_ = cur_mode_info_->img_out[cam_image_idx_];
	switch (out_kind_) {
	case OUT_IMG_DEPTH:
		show_img_ = {DISP_DEPTH, DISP_PCD};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		if (view_radio_button_ == PCD_C_IR) {
			std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(false);
			view_radio_button_ = PCD_C_DEPTH;
		}
		cam_light_cnt_img_ = IMG_DEPTH;
		break;
	case OUT_IMG_IR:
		show_img_ = {DISP_IR, DISP_NONE};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_IR);
		cam_light_cnt_img_ = IMG_IR;
		break;
	case OUT_IMG_DEPTH_IR:
		show_img_ = {DISP_DEPTH, DISP_IR};
		cam_disable_raw_ = true;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_IR);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_IR_PCD);
		cam_light_cnt_img_ = IMG_DEPTH;
		break;
	case OUT_IMG_DEPTH_IR_RAW:
		show_img_ = {DISP_DEPTH, DISP_IR};
		cam_disable_raw_ = false;
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_IR].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_IR);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_DEPTH_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_DEPTH_PCD);
		view_image_list_.push_back(SHOW_PAIR_LIST[SHOW_PAIR_IR_PCD].data());
		view_image_pair_.push_back(SHOW_PAIR_IR_PCD);
		cam_light_cnt_img_ = IMG_RAW1;
		break;
	case OUT_IMG_RAW:
		show_img_.fill(DISP_NONE);
		cam_disable_raw_ = false;
		view_image_list_.push_back("None");
		if (view_radio_button_ == PCD_C_IR) {
			std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD])->setKind(false);
			view_radio_button_ = PCD_C_DEPTH;
		}
		cam_light_cnt_img_ = IMG_RAW1;
		break;
	default:
		LOG_ERR("Unknown image output kind : %u\n", out_kind_);
		show_img_.fill(DISP_NONE);
		view_image_list_.push_back("None");
		cam_disable_raw_ = true;
		break;
	}
	updateImgFmts();
}

void ViewerGui::updateImgFmts(void)
{
	if (getDeviceProperty(CMD_IMG_FORMAT, &img_fmts_)) {
		for (uint8_t i = 0; i < IMG_KINDS; i++) {
			draw_imgs_[i]->setFormat(img_fmts_[i]);
		}
		updateShowImgKind();
		raw_win_->setFormat(img_fmts_);
		raw_win_->setMouseCb(static_cast<GLFWcursorenterfun>(&this->enterRawCursor), static_cast<GLFWcursorposfun>(&this->updateRawCursor));
		glfwMakeContextCurrent(win_);
		if (recording_) { stopRecord(); }
	}
}

void ViewerGui::updateShowImgKind(void)
{
	if (view_image_pair_.empty()) {
		show_img_.fill(DISP_NONE);
		show_fmt_[0].set();
		show_fmt_[1].set();
	} else {
		switch (view_image_pair_[view_image_idx_]) {
		case SHOW_PAIR_DEPTH_IR:
			show_img_ = {DISP_DEPTH, DISP_IR};
			show_fmt_[0] = img_fmts_[IMG_DEPTH];
			show_fmt_[1] = img_fmts_[IMG_IR];
			break;
		case SHOW_PAIR_DEPTH_PCD:
			show_img_ = {DISP_DEPTH, DISP_PCD};
			show_fmt_[0] = img_fmts_[IMG_DEPTH];
			show_fmt_[1].set();
			break;
		case SHOW_PAIR_IR_PCD:
			show_img_ = {DISP_IR, DISP_PCD};
			show_fmt_[0] = img_fmts_[IMG_IR];
			show_fmt_[1].set();
			break;
		case SHOW_PAIR_IR:
			show_img_ = {DISP_IR, DISP_NONE};
			show_fmt_[0] = img_fmts_[IMG_IR];
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

void ViewerGui::updateColorRange(const Range& d_range)
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

void ViewerGui::updatePlayInfo(bool update_time)
{
	PlayBack::PlayStatus	status;
	PlayBack::PlayTime		time;

	if (getDeviceProperty(PlayBack::CMD_PLAY_STATUS, &status)) {
		makeFpsString(status.playing_fps, play_frame_fps_);
		switch (status.state) {
		case PlayBack::STOPPED:
			play_pausing_ = false;
			state_cam_ = ST_STOPPED;
			snap_disable_take_ = true;
			break;
		case PlayBack::PLAYING:
			play_pausing_ = false;
			state_cam_ = ST_STREAMING;
			snap_disable_take_ = false;
			break;
		case PlayBack::PAUSE:
			play_pausing_ = true;
			state_cam_ = ST_PAUSE_PLAY;
			makeFpsString(UINT16_MAX, state_fps_);
			snap_disable_take_ = true;
			break;
		case PlayBack::FAST:
			state_cam_ = ST_FAST_PLAY;
			snap_disable_take_ = false;
			break;
		case PlayBack::SLOW:
			state_cam_ = ST_SLOW_PLAY;
			snap_disable_take_ = false;
			break;
		default:
			LOG_ERR("Unknown state : %d\n", status.state);
			break;
		}
	}

	if (update_time) {
		if (getDeviceProperty(PlayBack::CMD_PLAY_TIME, &time)) {
			updatePlayingTime(time, true);
		}
	}
}

void ViewerGui::updatePlayingTime(const PlayBack::PlayTime& time, bool update_total)
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

bool ViewerGui::updateCapture(void)
{
	uint16_t proc_id;
	FrameData* frame = nullptr;
	Result ret;

	if (state_cam_ == ST_CLOSED) { return false; }

	ret = getCamera()->getEvent(proc_id, &frame, false);
	switch (ret) {
	case SUCCESS:
		if (proc_id == PlFw::PROC_PIPELINE) {
			FrameExtInfo* info = frame->getFrameExt();
			makeFpsString(info->rcv_fps, state_fps_);
			if (tgt_cam_idx_ == PLAYBACK_INDEX) {
				updatePlayingTime(info->play_time);
			} else {
				const uint32_t light_cnt = frame->getFrame()->images[cam_light_cnt_img_].info.light_cnt;
				if (!cam_disable_light_time_) {
					cam_light_slid_ = static_cast<int>(light_cnt);
				}
			}
			getCamera()->releaseBuf(&frame);
			return true;
		}
		break;
	case REACH_EOF:
		if (proc_id == getProcId(PLT_RECORD)) {
			setRecState(ST_REC_FINISH);
			showMessage(MSG_ATTENTION, "Record is finished");
		} else if (proc_id == getProcId(PLT_DRAW)) {
			if (snap_taking_ && (cur_mode_info_ != nullptr)) {
				snap_disable_save_ = false;
			}
		} else if (proc_id == PlFw::PROC_PIPELINE) {
			if (tgt_cam_idx_ == PLAYBACK_INDEX) {
				stopCapture();
				updatePlayInfo();
				showMessage(MSG_ATTENTION, "Reached End of record file");
			}
		}
		break;
	case ERR_NOT_EXIST:
		break;
	case ERR_TIMEOUT:
		if (proc_id == PlFw::PROC_PIPELINE) {
			stopCapture();
			state_cam_ = ST_TIMEOUT;
			showMessage(MSG_CAUTION, "Time out receiving image");
		}
		break;
	case ERR_EMPTY:
		stopCapture();
		showMessage(MSG_CAUTION, "Receiving Buffer is empty");
		break;
	default:
		if (proc_id == getProcId(PLT_RECORD)) {
			setRecState(ST_REC_STOP);
			LOG_ERR("Failed to record : %d\n", ret);
			showMessage(MSG_CAUTION, "Failed to Record");
		} else {
			LOG_ERR("unknown : proc = %u  ret = %d\n", proc_id, ret);
			showMessage(MSG_CAUTION, "Failed to receive image");
		}
		break;
	}
	return false;
}

void ViewerGui::notifyPcdPrm(void)
{
	PosOrgRotation pos = {
		{static_cast<int16_t>(pst_pcd_ofst_x_), static_cast<int16_t>(pst_pcd_ofst_y_), static_cast<int16_t>(pst_pcd_ofst_z_)},
		{convRadDeg(pst_pcd_rot_x_), convRadDeg(pst_pcd_rot_y_), convRadDeg(pst_pcd_rot_z_)}
	};
	(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_ORG_POS, &pos);
}

void ViewerGui::startRecord(void)
{
	Result		ret;
	RecordParam param;
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
		LOG_WRN("Record Length is over : len=%llu fps=%.2f\n",
			static_cast<unsigned long long>(len), static_cast<float>(cur_mode_info_->fps) / 100.0F);
		showMessage(MSG_ATTENTION, "Record Length is over");
		return;
	}
	pack = (pack * static_cast<uint32_t>(cur_mode_info_->fps)) / 100UL;
	if (pack > UINT16_MAX) {
		LOG_WRN("Record Packing size is over : len=%u fps=%.2f\n", pack, static_cast<float>(cur_mode_info_->fps) / 100.0F);
		showMessage(MSG_ATTENTION, "Record Packing size is over");
		return;
	}

	param.path = cam_rec_dir_path_;
	param.save_frames = static_cast<uint32_t>(len);
	param.packing_frames = static_cast<uint16_t>(pack);
	param.is_crct_dist = pst_dist_enable_;
	param.is_filt_med = pstf_enable_medf_;
	param.is_filt_bil= pstf_enable_bilf_;
	param.is_filt_fly_p = pstf_enable_flypf_;

	ret = getCamera()->notifyEvent(getProcId(PLT_RECORD), EV_REC_START, &param);
	switch (ret) {
	case SUCCESS:
		break;
	case ERR_BAD_ARG:
		showMessage(MSG_ATTENTION, "Record target is already exist (retry later)");
		break;
	case ERR_NOT_EXIST:
		showMessage(MSG_ATTENTION, "Record directory is not exist");
		break;
	case ERR_FULL:
		state_rec_= ST_REC_FULL;
		showMessage(MSG_ATTENTION, "Record target storage is full");
		break;
	default:
		showMessage(MSG_WARN, "Failed to start recording");
		break;
	}

	if (ret == SUCCESS) {
		setRecState(ST_REC_RECORD);
	}
	return;
}

void ViewerGui::stopRecord(void)
{
	if (getCamera()->notifyEvent(getProcId(PLT_RECORD), EV_REC_STOP) != SUCCESS) {
		showMessage(MSG_WARN, "Failed to stop recording");
	}
	setRecState(ST_REC_STOP);
}

void ViewerGui::setRecState(RecState state)
{
	state_rec_ = state;
	switch (state) {
	case ST_REC_STOP:
	case ST_REC_FINISH:
	case ST_REC_FULL:
		recording_ = false;
		if (isCapturing()) {
			snap_disable_take_ = false;
		} else {
			stat_disable_cfg_load_ = false;
		}
		stat_disable_cfg_save_ = false;
		break;
	case ST_REC_RECORD:
		recording_ = true;
		if (snap_taking_) { exitSnapshot(); }
		snap_disable_take_ = true;
		stat_disable_cfg_save_ = true;
		stat_disable_cfg_load_ = true;
		break;
	default:
		break;
	}
}

void ViewerGui::takeSnapshot(void)
{
	ImVec2 pos = pnl_view_pos_;

	snap_took_frms_ = SNAP_FRMS[snap_frms_idx_];
	(void)getCamera()->notifyEvent(getProcId(PLT_DRAW), EV_SNAP_TAKE, &snap_took_frms_);
	snap_taking_ = true;
	snap_disable_exit_ = false;

	for (uint8_t i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
		if (show_img_[i] == DISP_PCD) {
			updatePointCloud(pos, true);
		}
		pos.x += pnl_view_size_.x;
	}
	disableCfgBtn();
}

void ViewerGui::exitSnapshot(void)
{
	(void)getCamera()->notifyEvent(getProcId(PLT_DRAW), EV_SNAP_EXIT);
	snap_taking_ = false;
	snap_disable_exit_ = true;
	snap_disable_save_ = true;
	enableCfgBtn();
}

void ViewerGui::saveSnapshot(void)
{
	SaveSnapshot sv_snap;
	SnapshotParams snap_prms;
	Result ret;

	snap_prms.file_fmt = snap_file_fmt_;
	snap_prms.took_frms = snap_took_frms_;
	snap_prms.medf = pstf_enable_medf_;
	snap_prms.bilf = pstf_enable_bilf_;
	snap_prms.flypf = pstf_enable_flypf_;
	snap_prms.dist = pst_dist_enable_;
	snap_prms.pcd_type = pst_pcd_origin_;
	snap_prms.pcd_ofst = { static_cast<int16_t>(pst_pcd_ofst_x_), static_cast<int16_t>(pst_pcd_ofst_y_), static_cast<int16_t>(pst_pcd_ofst_z_) };
	snap_prms.pcd_rot = { pst_pcd_rot_x_, pst_pcd_rot_y_, pst_pcd_rot_z_ };

	ret = sv_snap.save(snap_tgt_dir_path_, snap_prms, img_fmts_, draw_imgs_, getCamera());
	switch (ret) {
	case SUCCESS:
		showMessage(MSG_ATTENTION, "Saving snapshot is finished");
		break;
	case ERR_NOT_EXIST:
		showMessage(MSG_ATTENTION, "Snapshot directory is not exist");
		break;
	case ERR_FULL:
		showMessage(MSG_ATTENTION, "Snapshot target storage is full");
		break;
	default:
		showMessage(MSG_WARN, "Failed to start saving snapshot");
		break;
	}
}

void ViewerGui::loadConfig(void)
{
	ViewerConfigData cfg;

	if (config_.loadConfig(cfg) != SUCCESS) {
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

void ViewerGui::saveConfig(void)
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
	cfg.postfilter.bil.enable = pstf_enable_bilf_ ;
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

	if (config_.saveConfig(cfg) != SUCCESS) {
		showMessage(MSG_CAUTION, "Failed to save Viewer configuration");
	}
}

void ViewerGui::disableCfgBtn(void)
{
	stat_disable_cfg_save_ = true;
	stat_disable_cfg_load_ = true;
}

void ViewerGui::enableCfgBtn(void)
{
	stat_disable_cfg_save_ = (recording_ || snap_taking_);
	stat_disable_cfg_load_ = !isCaptureStopped();
}

bool ViewerGui::createWindow(void)
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

	win_ = glfwCreateWindow(win_w_, win_h_, WIN_NAME.data(), NULL, NULL);
	if (win_ == NULL) {
		LOG_ERR("failed to CreateWindow\n");
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
	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL(win_, true);
	ImGui_ImplOpenGL3_Init(glsl_version);

	glGenTextures(TEXTURE_NUM, texture_);

	setFont();
	updateLayout();

	return true;
}

void ViewerGui::destroyWindow(void)
{
	glDeleteTextures(TEXTURE_NUM, texture_);

	if (win_ != NULL) {
		glfwDestroyWindow(win_);
		win_ = NULL;
	}
	dir_browser_.Close();
	file_browser_.Close();
}

void ViewerGui::setFont(void)
{
	std::error_code ec;
	bool exist = std::filesystem::exists(JP_FONT.data(), ec);
	if (!ec && exist) {
		ImGuiIO& io = ImGui::GetIO();
		ImFontConfig config;
		config.MergeMode = true;
		io.Fonts->AddFontDefault();
		io.Fonts->AddFontFromFileTTF(JP_FONT.data(), 9.0f, &config, io.Fonts->GetGlyphRangesJapanese());
	} else {
		LOG_INF("Japanese font is not installed : %s\n", JP_FONT.data());
	}
}

bool ViewerGui::drawButton(const std::string& str, float width, bool disable)
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

bool ViewerGui::drawCombo(const std::string& label, const std::vector<std::string>& list, uint16_t& index, bool disable)
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

bool ViewerGui::convToCharPath(const std::filesystem::path& path, std::array<char, MAX_PATH_LEN>& c_path)
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

void ViewerGui::convToFsPath(const std::array<char, MAX_PATH_LEN>& c_path, std::filesystem::path& path)
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

void ViewerGui::drawPopUpMessage(void)
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

void ViewerGui::updateLayout(void)
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
	cam_reg_idt_ = CAM_REG_IDT;
	cam_reg_w_ = CAM_REG_W;
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
		cam_reg_idt_ *= scale;
		cam_reg_w_ *= scale;
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
	cam_reg_tgt_w_ = ctrl_dir_txt_w_ - 6.F + (INDENT_SPACE * scale);
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
	for (uint8_t i = 0; i < IMG_KINDS; i++) {
		draw_imgs_[i]->setScale(scale_u);
	}
}

void ViewerGui::drawCtrlPanel(void)
{
	ImGui::SetNextWindowPos(pnl_ctrl_pos_);
	ImGui::SetNextWindowSize(pnl_ctrl_size_);
	ImGui::Begin("Control", NULL, CTRL_WIN_FLAG);

	drawTgtDevLbl();
	drawDevCtrlLbl();
	drawPostFiltLbl();
	drawPostProcLbl();
	drawViewStgLbl();
	drawSnapShotLbl();
	drawCamParaLbl();

	ImGui::End();
}

void ViewerGui::drawTgtDevLbl(void)
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
					cam_prm_disabled_ = (tgt_cam_idx_ == PLAYBACK_INDEX);
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

void ViewerGui::drawDevCtrlLbl(void)
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

void ViewerGui::drawDevCtrlCam(void)
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
					if (setDeviceProperty(CMD_MODE, &mode_list_[i].id)) {
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

#ifdef VIEWER_ADV
	if (ImGui::TreeNode("Image Kinds")) {
		uint16_t cur_img_idx = cam_image_idx_;
		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (drawCombo("##Image Kinds", cam_image_list_, cam_image_idx_, (cam_image_list_.size() == 0))) {
			if (setDeviceProperty(CMD_IMG_KINDS, &cur_mode_info_->img_out[cam_image_idx_])) {
				updateImgKind();
			} else {
				cam_image_idx_ = cur_img_idx;
				showMessage(MSG_WARN, "Failed to change Image Kind");
			}
		}
		if (cam_cap_disable_) { ImGui::EndDisabled(); }
		ImGui::TreePop();
	}
#endif /* VIEWER_ADV */

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

		if (snap_taking_) { ImGui::BeginDisabled(); }
		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (cam_ae_enable_) { ImGui::BeginDisabled(); }
		if (ImGui::SliderInt("##Light Times", &cam_light_slid_, light_times_.min, light_times_.max, "%u", ImGuiSliderFlags_AlwaysClamp)) {
			light_times_.count = static_cast<uint32_t>(cam_light_slid_);
			if (setDeviceProperty(CMD_LIGHT_TIMES, &light_times_)) {
				if (getDeviceProperty(CMD_LIGHT_TIMES, &light_times_)) {
					cam_light_slid_ = static_cast<int>(light_times_.count);
				}
			} else {
				showMessage(MSG_WARN, "Light Times is over the range");
			}
		}
		if (cam_ae_enable_) { ImGui::EndDisabled(); }

		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		if (ImGui::Checkbox("Auto Exposure", &cam_ae_enable_)) {
			if (!setDeviceProperty(CMD_AE_STATE, &cam_ae_enable_)) {
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
			if (!setDeviceProperty(CMD_AE_INTERVAL, &cam_ae_interval_)) {
				showMessage(MSG_WARN, "Failed to change Auto Exposure Interval");
			}
		}
		if (!cam_ae_enable_) { ImGui::EndDisabled(); }
		if (cam_cap_disable_) { ImGui::EndDisabled(); }

		if (snap_taking_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}
	if (cam_disable_light_time_) { ImGui::EndDisabled(); }

	if (ImGui::TreeNode("Threshold")) {
		int sat_th = static_cast<int>(cam_raw_sat_th_.threshold);
		int dark_th = static_cast<int>(cam_ir_dark_th_.threshold);

		if (snap_taking_) { ImGui::BeginDisabled(); }

#ifdef VIEWER_ADV
		ImGui::Text("Saturation");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##RawSatThresh", &sat_th,
			static_cast<int>(cam_raw_sat_th_.min), static_cast<int>(cam_raw_sat_th_.max),
			"%4u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_raw_sat_th_.threshold = static_cast<uint16_t>(sat_th);
		}
#endif /* VIEWER_ADV */

		ImGui::Text("Dark");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);
		if (ImGui::SliderInt("##IrDarkThresh", &dark_th,
			static_cast<int>(cam_ir_dark_th_.min), static_cast<int>(cam_ir_dark_th_.max),
			"%4u", ImGuiSliderFlags_AlwaysClamp)) {
			cam_ir_dark_th_.threshold = static_cast<uint16_t>(dark_th);
		}

		if (drawButton("Set", ctrl_btn_w_)) {
			if (!setDeviceProperty(CMD_RAW_SAT_TH, &cam_raw_sat_th_)) {
				showMessage(MSG_WARN, "Failed to change RAW Saturation threshold");
			}
			if (!setDeviceProperty(CMD_IR_DARK_TH, &cam_ir_dark_th_)) {
				showMessage(MSG_WARN, "Failed to change IR Dark threshold");
			}
		}

		if (snap_taking_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("External Trigger")) {
		uint16_t ext_trigger = static_cast<uint16_t>(cam_ext_trigger_ - EXT_TRG_STANDALONE);
		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (cam_cap_disable_) { ImGui::BeginDisabled(); }
		if (drawCombo("##ExtTrigger", cam_ext_trg_list_, ext_trigger, cam_ext_trg_disable_)) {
			cam_ext_trigger_ = static_cast<ExtTriggerType>(ext_trigger + EXT_TRG_STANDALONE);
			if (setDeviceProperty(CMD_EXT_TRG_TYPE, &cam_ext_trigger_)) {
				cam_ext_trg_disable_ = (cam_ext_trigger_ == EXT_TRG_SLAVE);
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
			if(!setDeviceProperty(CMD_EXT_TRG_OFFSET, &offset)){
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

		if (snap_taking_) { ImGui::BeginDisabled(); }

		ImGui::Text("Mode");
		ImGui::SameLine(ctrl_btn_idt_, 0);
		ImGui::SetNextItemWidth(ctrl_btn_w_);

		if (drawCombo("##IntSupp", cam_int_supp_mode_list_, mode, cam_int_supp_mode_disable_)) {
			cam_int_supp_info_.mode = static_cast<IntSuppModeType>(mode);
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

#ifdef VIEWER_ADV
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
#endif /* VIEWER_ADV */

		if (drawButton("Set", ctrl_btn_w_)) {
			if (!setDeviceProperty(CMD_INT_SUPP_INFO, &cam_int_supp_info_)) {
				showMessage(MSG_WARN, "Failed to set interference suppression info");
			}
		}

		if (snap_taking_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}

#ifdef DEVELOP
	if (cam_reg_target_list_.empty()) { ImGui::BeginDisabled(); ImGui::SetNextItemOpen(false); }
	if (ImGui::TreeNode("Register")) {
		if (snap_taking_) { ImGui::BeginDisabled(); }
		ImGui::Text("Target");
		ImGui::SameLine(ctrl_dir_idt_, 0);
		ImGui::SetNextItemWidth(cam_reg_tgt_w_);
		drawCombo("##Target Register", cam_reg_target_list_, cam_reg_target_idx_, (cam_reg_target_list_.size() == 0));

		ImGui::Text("Address     (HEX)");
		ImGui::SameLine(cam_reg_idt_);
		ImGui::SetNextItemWidth(cam_reg_w_);
		if (ImGui::InputText("##Register Address", cam_reg_addr_.data(), cam_reg_addr_.size(), TEXT_HEX_FLAG)) {
			uint16_t val = static_cast<uint16_t>(convStrVal(cam_reg_addr_.data(), 16));
			if (val == 0) {
				cam_reg_addr_.fill(0);
				std::snprintf(cam_reg_addr_.data(), cam_reg_addr_.size(), "0");
			}
		}
		ImGui::Text("Value       (HEX)");
		ImGui::SameLine(cam_reg_idt_);
		ImGui::SetNextItemWidth(cam_reg_w_);
		if (ImGui::InputText("##Register Value", cam_reg_val_.data(), cam_reg_val_.size(), TEXT_HEX_FLAG)) {
			uint16_t val = static_cast<uint16_t>(convStrVal(cam_reg_val_.data(), 16));
			if (val == 0) {
				cam_reg_val_.fill(0);
				std::snprintf(cam_reg_val_.data(), cam_reg_val_.size(), "0");
			}
		}

		if (drawButton("Read", ctrl_dir_btn_w_)) {
			RegInfo reg;
			reg.target = reg_devs_[cam_reg_target_idx_].target;
			reg.addr = static_cast<uint16_t>(convStrVal(cam_reg_addr_.data(), 16));
			if (getDeviceProperty(CMD_REG, &reg)) {
				cam_reg_val_.fill(0);
				std::snprintf(cam_reg_val_.data(), cam_reg_val_.size(), "%X", reg.value);
			} else {
				showMessage(MSG_CAUTION, "Failed to read register.");
			}
		}
		ImGui::SameLine();
		if (drawButton("Write", ctrl_dir_btn_w_, cam_cap_disable_)) {
			RegInfo reg;
			reg.target = reg_devs_[cam_reg_target_idx_].target;
			reg.addr  = static_cast<uint16_t>(convStrVal(cam_reg_addr_.data(), 16));
			reg.value = static_cast<uint16_t>(convStrVal(cam_reg_val_.data(), 16));
			if (!setDeviceProperty(CMD_REG, &reg)) {
				showMessage(MSG_CAUTION, "Failed to write register.");
			}
		}
		if (snap_taking_) { ImGui::EndDisabled(); }

		ImGui::TreePop();
	}

	if (ImGui::TreeNode("Register list")) {
		if (snap_taking_) { ImGui::BeginDisabled(); }
		if (drawButton("File", ctrl_dir_btn_w_)) {
			std::filesystem::path path;
			convToFsPath(cam_reg_file_path_c_, cam_reg_file_path_);
			path = cam_reg_file_path_;
			path.remove_filename();
			file_browser_.SetPwd(path);
			file_browser_.ClearSelected();
			file_browser_.SetTypeFilters({".csv"});
			file_browser_.Open();
			browser_target_ = FILE_BROWS_REGLIST;
		}
		if ((browser_target_ == FILE_BROWS_REGLIST) & file_browser_.HasSelected()) {
			cam_reg_file_path_ = file_browser_.GetSelected();
			file_browser_.ClearSelected();
			browser_target_ = FILE_BROWS_NONE;
			if (!convToCharPath(cam_reg_file_path_, cam_reg_file_path_c_)) {
				convToFsPath(cam_reg_file_path_c_, cam_reg_file_path_);
				LOG_ERR("File Path is over\n");
				showMessage(MSG_CAUTION, "File Path length is over\n");
			}
		}
		ImGui::SameLine();
		ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
		ImGui::InputTextWithHint("##File Path", FILE_PATH_HINT.data(), cam_reg_file_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);

		if (drawButton("Read", ctrl_dir_btn_w_)) {
			RegCsvList csv;
			if (csv.readRegList(getCamera(), cam_reg_file_path_)) {
				showMessage(MSG_CAUTION, "Failed to read register list.");
			}
		}
		ImGui::SameLine();
		if (drawButton("Write", ctrl_dir_btn_w_, cam_cap_disable_)) {
			RegCsvList csv;
			if (csv.writeRegList(getCamera(), cam_reg_file_path_)) {
				showMessage(MSG_CAUTION, "Failed to write register list.");
			}
		}

		if (snap_taking_) { ImGui::EndDisabled(); }
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Register Refresh")) {
		if (drawButton("Refresh", ctrl_dir_btn_w_, cam_cap_disable_)) {
			if (setDeviceProperty(CMD_REG_REFRESH)) {
				updateDeviceInfo();
			}
		}
		ImGui::TreePop();
	}
	if (cam_reg_target_list_.empty()) { ImGui::EndDisabled(); }
#endif	/* DEVELOP */

	if (ImGui::TreeNode("Record")) {
		if (snap_taking_) { ImGui::BeginDisabled(); }
		if (drawButton("Directory", ctrl_dir_btn_w_, recording_)) {
			convToFsPath(cam_rec_dir_path_c_, cam_rec_dir_path_);
			dir_browser_.SetPwd(cam_rec_dir_path_);
			dir_browser_.ClearSelected();
			dir_browser_.Open();
			browser_target_ = FILE_BROWS_REC;
		}
		if ((browser_target_ == FILE_BROWS_REC) && dir_browser_.HasSelected()) {
			cam_rec_dir_path_ = dir_browser_.GetSelected();
			LOG_INF("select dir=%s\n", cam_rec_dir_path_.string<char>().c_str());
			dir_browser_.ClearSelected();
			browser_target_ = FILE_BROWS_NONE;
			if (!convToCharPath(cam_rec_dir_path_, cam_rec_dir_path_c_)) {
				convToFsPath(cam_rec_dir_path_c_, cam_rec_dir_path_);
				LOG_ERR("File Path is over\n");
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

		if (snap_taking_) { ImGui::EndDisabled(); }
		ImGui::TreePop();
	}
}

void ViewerGui::drawDevCtrlPlay(void)
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
			LOG_INF("select dir=%s\n", path.string<char>().c_str());
			dir_browser_.ClearSelected();
			browser_target_ = FILE_BROWS_NONE;
			if (!convToCharPath(path, play_target_path_c_)) {
				LOG_ERR("File Path is over\n");
			} else {
				if (setDeviceProperty(PlayBack::CMD_PLAY_TARGET, &path)) {
					play_target_path_ = path;
					updateDeviceInfo();
				} else {
					LOG_ERR("Not exist record files in %s\n", path.string<char>().c_str());
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
				if (setDeviceProperty(PlayBack::CMD_PLAY_TARGET, &play_target_path_)) {
					startCapture();
				} else {
					showMessage(MSG_CAUTION, "Not exist record files");
					play_target_path_.clear();
				}
			}
		}
		disable_ctrl = !play_cap_disable_ || snap_taking_;
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (disable_ctrl) { ImGui::BeginDisabled(); }
		if (drawButton("Pause", ctrl_btn_w_)) {
			if (setDeviceProperty(PlayBack::CMD_PAUSE)) {
				updatePlayInfo(false);
			}
		}
		if (play_pausing_) { ImGui::BeginDisabled(); }
		if (drawButton("Slow Play", ctrl_btn_w_)) {
			if (setDeviceProperty(PlayBack::CMD_SLOW_PLAY)) {
				updatePlayInfo(false);
			} else {
				showMessage(MSG_ATTENTION, "Reached minimum speed");
			}
		}
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (drawButton("Fast Play", ctrl_btn_w_)) {
			if (setDeviceProperty(PlayBack::CMD_FAST_PLAY)) {
				updatePlayInfo(false);
			} else {
				showMessage(MSG_ATTENTION, "Reached maximum speed");
			}
		}
		if (play_pausing_) { ImGui::EndDisabled(); }
		if (drawButton("Jump Backward", ctrl_btn_w_)) {
			if (setDeviceProperty(PlayBack::CMD_JUMP_BW, &play_jump_time_)) {
				updatePlayInfo();
			}
		}
		ImGui::SameLine(ctrl_btn_idt_, 0);
		if (drawButton("Jump Forward", ctrl_btn_w_)) {
			if (setDeviceProperty(PlayBack::CMD_JUMP_FW, &play_jump_time_)) {
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
			PlayBack::PlayTime time;
			if (getDeviceProperty(PlayBack::CMD_PLAY_TIME, &time)) {
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
					if (setDeviceProperty(PlayBack::CMD_PLAY_TIME, &time)) {
						updatePlayInfo();
					}
				}
			}
		}

		ImGui::SetNextItemWidth(ctrl_node_w_);
		if (ImGui::SliderInt("##Playtime", &tmp_play_time, PLAY_SLIDER_MIN, PLAY_SLIDER_MAX, "%d%%", ImGuiSliderFlags_AlwaysClamp)) {
			PlayBack::PlayStatus	status;
			if (getDeviceProperty(PlayBack::CMD_PLAY_STATUS, &status)) {
				if (status.state != PlayBack::STOPPED) {
					PlayBack::PlayTime time;
					if (getDeviceProperty(PlayBack::CMD_PLAY_TIME, &time)) {
						time.current = static_cast<uint32_t>((static_cast<uint64_t>(time.total) * static_cast<uint64_t>(tmp_play_time)) / 100ULL);
						if (time.current >= time.total) { time.current = time.total - 1U; }
						if (setDeviceProperty(PlayBack::CMD_PLAY_TIME, &time)) {
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
#ifdef VIEWER_ADV
	if (ImGui::TreeNode("Image Kinds")) {
		if (!cam_image_list_.empty()) {
			ImGui::Text("%s", cam_image_list_[0].c_str());
		}
		ImGui::TreePop();
	}
#endif /* VIEWER_ADV */
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

void ViewerGui::drawPostFiltLbl(void)
{
	if (ImGui::CollapsingHeader("Post Filter")) {
		uint16_t m_ksize = 0;
		uint16_t b_ksize = 0;
		double b_sigma_depth = static_cast<double>(pstf_prm_.bil_sigma_depth);
		double b_sigma_ir = static_cast<double>(pstf_prm_.bil_sigma_ir);
		double b_sigma_space = static_cast<double>(pstf_prm_.bil_sigma_space);
		uint16_t f_ksize = 0;
		uint16_t f_log = 0;
		uint16_t f_fast_proc = 0;
		int f_thr = static_cast<int>(pstf_prm_.flyp_thr);
		bool disabled = recording_ || snap_taking_;
		uint16_t i;

		if (ImGui::TreeNode("Median Filter")) {
			if (disabled || pstf_medf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_medf_)) {
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_MEDF, &pstf_enable_medf_);
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
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
			if (disabled || pstf_medf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Bilateral Filter")) {
			if (disabled || pstf_bilf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_bilf_)) {
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_BILF, &pstf_enable_bilf_);
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
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
#ifdef VIEWER_ADV
			ImGui::Text("Sigma Depth");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			double min_sigma_depth = 0.001;
			double max_sigma_depth = 1000.0;
			if (ImGui::SliderScalar("##sigma depth", ImGuiDataType_Double, &b_sigma_depth, &min_sigma_depth, &max_sigma_depth, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_depth < 0.001) {
					pstf_prm_.bil_sigma_depth = 0.001;
				}
				else if (b_sigma_depth > 1000.0) {
					pstf_prm_.bil_sigma_depth = 1000.0;
				}
				else if (b_sigma_depth == -0.001) {
					pstf_prm_.bil_sigma_depth = 0.001;
				}
				else {
					pstf_prm_.bil_sigma_depth = static_cast<double>(b_sigma_depth);
				}
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
			ImGui::Text("Sigma IR");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			double min_sigma_ir = 0.001;
			double max_sigma_ir = 250.0;
			if (ImGui::SliderScalar("##sigma ir", ImGuiDataType_Double, &b_sigma_ir, &min_sigma_ir, &max_sigma_ir, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_ir < 0.001) {
					pstf_prm_.bil_sigma_ir = 0.001;
				}
				else if (b_sigma_ir > 250.0) {
					pstf_prm_.bil_sigma_ir = 250.0;
				}
				else if (b_sigma_ir == -0.001) {
					pstf_prm_.bil_sigma_ir = 0.001;
				}
				else {
					pstf_prm_.bil_sigma_ir = static_cast<double>(b_sigma_ir);
				}
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
			ImGui::Text("Sigma Space");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			double min_sigma_space = 0.001;
			double max_sigma_space = 10.0;
			if (ImGui::SliderScalar("##sigma space", ImGuiDataType_Double, &b_sigma_space, &min_sigma_space, &max_sigma_space, "%.3f", ImGuiSliderFlags_AlwaysClamp)) {
				if (b_sigma_space < 0.001) {
					pstf_prm_.bil_sigma_space = 0.001;
				}
				else if (b_sigma_space > 10.0) {
					pstf_prm_.bil_sigma_space = 10.0;
				}
				else if (b_sigma_space == -0.001) {
					pstf_prm_.bil_sigma_space = 0.001;
				}
				else {
					pstf_prm_.bil_sigma_space = static_cast<double>(b_sigma_space);
				}
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
#endif /* VIEWER_ADV */
			if (disabled || pstf_bilf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Flying Pixel Filter")) {
			if (disabled || pstf_flypf_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pstf_enable_flypf_)) {
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_FLYPF, &pstf_enable_flypf_);
				}
			}
			for (i = 0; sizeof(pstf_ksize_map_); i++) {
				if (pstf_ksize_map_[i] == pstf_prm_.flyp_ksize) {
					f_ksize = i;
					break;
				}
			}
#ifndef VIEWER_ADV
			ImGui::BeginDisabled();
#endif /* VIEWER_ADV */
			ImGui::Text("Filter Size");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (drawCombo("##ksize", pstf_ksize_list_, f_ksize, disabled)) {
				pstf_prm_.flyp_ksize = pstf_ksize_map_[f_ksize];
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
#ifndef VIEWER_ADV
			ImGui::EndDisabled();
#endif /* VIEWER_ADV */
#ifdef VIEWER_ADV
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
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
			if (pstf_prm_.flyp_fast_proc) {
				f_fast_proc = 1;
			} else {
				f_fast_proc = 0;
			}
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
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
#endif /* VIEWER_ADV */
			ImGui::Text("Threshold");
			ImGui::SameLine(pstf_ofst_w_, 0);
			ImGui::SetNextItemWidth(pstf_inp_w_);
			if (ImGui::InputInt("##thr", &f_thr)) {
				if (f_thr > 8000U) { f_thr = 8000U; }
				if (f_thr < 0) { f_thr = 0; }
				pstf_prm_.flyp_thr = static_cast<uint16_t>(f_thr);
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_POSTFILTER), EV_POSTFILT_PRM, &pstf_prm_);
				}
			}
			if (disabled || pstf_flypf_check_disable_) { ImGui::EndDisabled(); }
			ImGui::TreePop();
		}
	}
}

void ViewerGui::drawPostProcLbl(void)
{
	if (ImGui::CollapsingHeader("Post Process")) {
		bool disabled = recording_ || snap_taking_;
		if (ImGui::TreeNode("Distortion")) {
			if (disabled || pst_dist_check_disable_) { ImGui::BeginDisabled(); }
			if (ImGui::Checkbox("Enable", &pst_dist_enable_)) {
				if (!tgt_dev_enable_) {
					(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_DIST, &pst_dist_enable_);
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
					(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_KIND, &world_coord);
				}
				grid_view_kind_ = (grid_view_kind_ == GRID_KIND_FORCE_ORIGIN) ? GRID_KIND_FORCE_ORIGIN : GRID_KIND_CAMERA;
				draw_pcd->updateGridSetting();
			}
			if (ImGui::RadioButton("World Origin", &pst_pcd_origin_, 1)) {
				pst_pcd_cam_coord_ = false;
				if (!tgt_dev_enable_) {
					bool world_coord = !pst_pcd_cam_coord_;
					(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_KIND, &world_coord);
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

void ViewerGui::drawViewStgLbl(void)
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
					draw_imgs_[IMG_DEPTH]->refresh();
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
					draw_imgs_[IMG_DEPTH]->refresh();
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
					draw_imgs_[IMG_IR]->refresh();
					for (uint8_t i = IMG_RAW1; i <= IMG_RAW4; i++) {
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
					draw_imgs_[IMG_IR]->refresh();
					for (uint8_t i = IMG_RAW1; i <= IMG_RAW4; i++) {
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
#ifdef VIEWER_ADV
			if (drawButton("RAW expand", ctrl_btn_w_, cam_disable_raw_)) {
				if (!raw_win_->toggle()) {
					mouse_enter_raw_ = false;
				}
				glfwMakeContextCurrent(win_);
			}
#endif /* VIEWER_ADV */

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
				bool ir_invalid = ((out_kind_ == OUT_IMG_DEPTH) || (out_kind_ == OUT_IMG_RAW));
				if (ImGui::RadioButton("Depth Color", &view_radio_button_, PCD_C_DEPTH)) {
					PcdColorKind pcd_color = PCD_COLOR_NONE;
					(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_COLOR, &pcd_color);
					draw_pcd->setKind(false);
				}
				if (ir_invalid) { ImGui::BeginDisabled(); }
				if (ImGui::RadioButton("Grayscale(IR)", &view_radio_button_, PCD_C_IR)) {
					PcdColorKind pcd_color = PCD_COLOR_IR;
					(void)getCamera()->notifyEvent(getProcId(PLT_LENSCONV), EV_LENS_PCD_COLOR, &pcd_color);
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

void ViewerGui::drawSnapShotLbl(void)
{
	if (ImGui::CollapsingHeader("Snapshot Setting")) {
		if (ImGui::TreeNode("Snapshot")) {
			ImGui::Text("Frame Count");
			ImGui::SameLine(ctrl_btn_idt_, 0);
			ImGui::SetNextItemWidth(ctrl_btn_w_);
			drawCombo("##Snapshot Frame Count", snap_frms_list_, snap_frms_idx_);

			if (drawButton("Take Snapshot", ctrl_btn_w_, snap_disable_take_)) {
				takeSnapshot();
			}
			ImGui::SameLine(ctrl_btn_idt_, 0);
			if (drawButton("Exit Snapshot", ctrl_btn_w_, snap_disable_exit_)) {
				exitSnapshot();
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Save Snapshot")) {
			if (drawButton("Directory", ctrl_dir_btn_w_)) {
				convToFsPath(snap_tgt_dir_path_c_, snap_tgt_dir_path_);
				dir_browser_.SetPwd(snap_tgt_dir_path_);
				dir_browser_.ClearSelected();
				dir_browser_.Open();
				browser_target_ = FILE_BROWS_SNAP;
			}
			if ((browser_target_ == FILE_BROWS_SNAP) && dir_browser_.HasSelected()) {
				std::filesystem::path path = dir_browser_.GetSelected();
				LOG_INF("select dir=%s\n", path.string<char>().c_str());
				dir_browser_.ClearSelected();
				browser_target_ = FILE_BROWS_NONE;
				if (!convToCharPath(path, snap_tgt_dir_path_c_)) {
					LOG_ERR("File Path is over\n");
				} else {
					snap_tgt_dir_path_ = path;
				}
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			ImGui::InputTextWithHint("##Snapshot Directory", PATH_HINT.data(), snap_tgt_dir_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);

			ImGui::Text("Image File Format");
			ImGui::RadioButton("Binary", &snap_file_fmt_, SNAP_FILE_BIN);
			ImGui::SameLine();
			ImGui::RadioButton("CSV", &snap_file_fmt_, SNAP_FILE_CSV);

			if (drawButton("Save Snapshot", ctrl_btn_w_, snap_disable_save_)) {
				saveSnapshot();
			}

			ImGui::TreePop();
		}
	}
}

void ViewerGui::drawCamParaLbl(void)
{
#ifdef DEVELOP
	bool disable_cam_prm = !tgt_dev_selected_ || (tgt_cam_idx_ == PLAYBACK_INDEX);
	if (disable_cam_prm) { ImGui::BeginDisabled(); ImGui::SetNextItemOpen(false); }
	if (ImGui::CollapsingHeader("Camera Parameter")) {
		if (cam_prm_disabled_) { ImGui::BeginDisabled(); }
		if (ImGui::TreeNode("Write Parameter")) {
			if (ImGui::RadioButton("All Parameters", &cam_prm_w_selected_, 0)) {
				cam_prm_w_part_ = false;
			}
			if (ImGui::RadioButton("Partial Parameters", &cam_prm_w_selected_, 1)) {
				cam_prm_w_part_ = true;
			}
			if (!cam_prm_w_part_) { ImGui::BeginDisabled(); }
			ImGui::Text("ID (HEX)");
			ImGui::SameLine(ctrl_dir_idt_, 0);
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			if (ImGui::InputText("##Write parameter ID", cam_prm_w_id_.data(), cam_prm_w_id_.size(), TEXT_HEX_FLAG)) {
				uint16_t val = static_cast<uint16_t>(convStrVal(cam_prm_w_id_.data(), 16));
				if (val == 0) {
					cam_prm_w_id_.fill(0);
					std::snprintf(cam_prm_w_id_.data(), cam_prm_w_id_.size(), "0");
				}
			}
			if (!cam_prm_w_part_) { ImGui::EndDisabled(); }
			if (drawButton("File##CameraPara", ctrl_dir_btn_w_)) {
				std::filesystem::path path;
				convToFsPath(cam_prm_w_file_path_c_, cam_prm_w_file_path_);
				path = cam_prm_w_file_path_;
				path.remove_filename();
				file_browser_.SetPwd(path);
				file_browser_.ClearSelected();
				file_browser_.SetTypeFilters({".bin"});
				file_browser_.Open();
				browser_target_ = FILE_BROWS_CAM_W;
			}
			if ((browser_target_ == FILE_BROWS_CAM_W) && file_browser_.HasSelected()) {
				cam_prm_w_file_path_ = file_browser_.GetSelected();
				file_browser_.ClearSelected();
				browser_target_ = FILE_BROWS_NONE;
				if (!convToCharPath(cam_prm_w_file_path_, cam_prm_w_file_path_c_)) {
					convToFsPath(cam_prm_w_file_path_c_, cam_prm_w_file_path_);
					LOG_ERR("File Path is over\n");
				}
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			ImGui::InputTextWithHint("##Write Para File Path", FILE_PATH_HINT.data(), cam_prm_w_file_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);

			if (drawButton("Write##CameraPara", ctrl_btn_w_)) {
				convToFsPath(cam_prm_w_file_path_c_, cam_prm_w_file_path_);
				if (writeCamPrm()) {
					showMessage(MSG_ATTENTION, "Updated Camera parameter. Please reboot Camera module.");
				} else {
					showMessage(MSG_CAUTION, "Failed to updated Camera parameter.");
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Read Parameter")) {
			if (ImGui::RadioButton("All Parameters", &cam_prm_r_selected_, 0)) {
				cam_prm_r_part_ = false;
			}
			if (ImGui::RadioButton("Partial Parameters", &cam_prm_r_selected_, 1)) {
				cam_prm_r_part_ = true;
			}
			if (!cam_prm_r_part_) { ImGui::BeginDisabled(); }
			ImGui::Text("ID (HEX)");
			ImGui::SameLine(ctrl_dir_idt_, 0);
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			if (ImGui::InputText("##Read parameter ID", cam_prm_r_id_.data(), cam_prm_r_id_.size(), TEXT_HEX_FLAG)) {
				uint16_t val = static_cast<uint16_t>(convStrVal(cam_prm_r_id_.data(), 16));
				if (val == 0) {
					cam_prm_r_id_.fill(0);
					std::snprintf(cam_prm_r_id_.data(), cam_prm_r_id_.size(), "0");
				}
			}
			if (!cam_prm_r_part_) { ImGui::EndDisabled(); }
			if (drawButton("Directory", ctrl_dir_btn_w_)) {
				convToFsPath(cam_prm_r_dir_path_c_, cam_prm_r_dir_path_);
				dir_browser_.SetPwd(cam_prm_r_dir_path_);
				dir_browser_.ClearSelected();
				dir_browser_.Open();
				browser_target_ = FILE_BROWS_CAM_R;
			}
			if ((browser_target_ == FILE_BROWS_CAM_R) && dir_browser_.HasSelected()) {
				cam_prm_r_dir_path_ = dir_browser_.GetSelected();
				LOG_INF("select dir=%s\n", cam_prm_r_dir_path_.string<char>().c_str());
				dir_browser_.ClearSelected();
				browser_target_ = FILE_BROWS_NONE;
				if (!convToCharPath(cam_prm_r_dir_path_, cam_prm_r_dir_path_c_)) {
					convToFsPath(cam_prm_r_dir_path_c_, cam_prm_r_dir_path_);
					LOG_ERR("File Path is over\n");
				}
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			ImGui::InputTextWithHint("##Read Para Directory", PATH_HINT.data(), cam_prm_r_dir_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);
			if (drawButton("Read", ctrl_btn_w_)) {
				convToFsPath(cam_prm_r_dir_path_c_, cam_prm_r_dir_path_);
				if (readCamPrm()) {
					showMessage(MSG_ATTENTION, "Finish to get Camera parameter.");
				} else {
					showMessage(MSG_CAUTION, "Failed to get Camera parameter.");
				}
			}
			ImGui::TreePop();
		}
		if (cam_prm_disabled_) { ImGui::EndDisabled(); }
	}
	if (ImGui::CollapsingHeader("Firmware")) {
		if (cam_prm_disabled_) { ImGui::BeginDisabled(); }
		if (ImGui::TreeNode("Write")) {
			if (ImGui::RadioButton("Configuration and Firmware image 1", &firmware_w_selected_, 0)) {
				firmware_w_kind_ = CAM_PRM_ARTIX_CONF;
			}
			if (ImGui::RadioButton("Firmware image 2", &firmware_w_selected_, 1)) {
				firmware_w_kind_ = CAM_PRM_FIRM_2;
			}
			if (drawButton("File##Firmware", ctrl_dir_btn_w_)) {
				std::filesystem::path path;
				convToFsPath(firmware_w_file_path_c_, firmware_w_file_path_);
				path = firmware_w_file_path_;
				path.remove_filename();
				file_browser_.SetPwd(path);
				file_browser_.ClearSelected();
				file_browser_.SetTypeFilters({".bin"});
				file_browser_.Open();
				browser_target_ = FILE_BROWS_FIRM_W;
			}
			if ((browser_target_ == FILE_BROWS_FIRM_W) && file_browser_.HasSelected()) {
				firmware_w_file_path_ = file_browser_.GetSelected();
				file_browser_.ClearSelected();
				browser_target_ = FILE_BROWS_NONE;
				if (!convToCharPath(firmware_w_file_path_, firmware_w_file_path_c_)) {
					convToFsPath(firmware_w_file_path_c_, firmware_w_file_path_);
					LOG_ERR("File Path is over\n");
				}
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			ImGui::InputTextWithHint("##Write Firmware File Path", FILE_PATH_HINT.data(), firmware_w_file_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);
			if (drawButton("Write##Firmware", ctrl_btn_w_)) {
				convToFsPath(firmware_w_file_path_c_, firmware_w_file_path_);
				if (writeFirmware()) {
					showMessage(MSG_ATTENTION, "Updated firmware. Please reboot Camera module.");
				} else {
					showMessage(MSG_CAUTION, "Failed to updated firmware.");
				}
			}
			ImGui::TreePop();
		}
		if (ImGui::TreeNode("Read")) {
			if (ImGui::RadioButton("Configuration and Firmware image 1", &firmware_r_selected_, 0)) {
				firmware_r_kind_ = CAM_PRM_ARTIX_CONF;
			}
			if (ImGui::RadioButton("Firmware image 2", &firmware_r_selected_, 1)) {
				firmware_r_kind_ = CAM_PRM_FIRM_2;
			}
			if (drawButton("Directory", ctrl_dir_btn_w_)) {
				convToFsPath(firmware_r_dir_path_c_, firmware_r_dir_path_);
				dir_browser_.SetPwd(firmware_r_dir_path_);
				dir_browser_.ClearSelected();
				dir_browser_.Open();
				browser_target_ = FILE_BROWS_FIRM_R;
			}
			if ((browser_target_ == FILE_BROWS_FIRM_R) && dir_browser_.HasSelected()) {
				firmware_r_dir_path_ = dir_browser_.GetSelected();
				LOG_INF("select dir=%s\n", firmware_r_dir_path_.string<char>().c_str());
				dir_browser_.ClearSelected();
				browser_target_ = FILE_BROWS_NONE;
				if (!convToCharPath(firmware_r_dir_path_, firmware_r_dir_path_c_)) {
					convToFsPath(firmware_r_dir_path_c_, firmware_r_dir_path_);
					LOG_ERR("File Path is over\n");
				}
			}
			ImGui::SameLine();
			ImGui::SetNextItemWidth(ctrl_dir_txt_w_);
			ImGui::InputTextWithHint("##Read Firmware Directory", PATH_HINT.data(), firmware_r_dir_path_c_.data(), MAX_PATH_LEN, ImGuiInputTextFlags_ReadOnly);
			if (drawButton("Read", ctrl_btn_w_)) {
				convToFsPath(firmware_r_dir_path_c_, firmware_r_dir_path_);
				if (readFirmware()) {
					showMessage(MSG_ATTENTION, "Finish to get firmware.");
				} else {
					showMessage(MSG_CAUTION, "Failed to get firmware.");
				}
			}
			ImGui::TreePop();
		}
		if (cam_prm_disabled_) { ImGui::EndDisabled(); }
	}
	if (disable_cam_prm) { ImGui::EndDisabled(); }
#endif	/* DEVELOP */
}

void ViewerGui::drawViewPanel(void)
{
	ShowDisplayKind disp;
	ImVec2 pos = pnl_view_pos_;
	const std::array<std::string_view, DISP_KINDS>& str = snap_taking_ ? SHOW_SNAP_DISP_LIST : SHOW_DISP_LIST;;

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
			draw_imgs_[IMG_DEPTH]->lock();
			drawImage(i, show_fmt_[i].width, show_fmt_[i].height, draw_imgs_[IMG_DEPTH]->getDrawImage());
			draw_imgs_[IMG_DEPTH]->unlock();
			break;
		case DISP_IR:
			draw_imgs_[IMG_IR]->lock();
			drawImage(i, show_fmt_[i].width, show_fmt_[i].height, draw_imgs_[IMG_IR]->getDrawImage());
			draw_imgs_[IMG_IR]->unlock();
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

void ViewerGui::drawStatusPanel(void)
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

void ViewerGui::drawStatusWin(void)
{
	ImGui::Text("Device Status");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", STATE_CAM_LIST[state_cam_].data());

	ImGui::Text("Record Status");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", STATE_REC_LIST[state_rec_].data());

	ImGui::Text("Snapshot Status");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", snap_taking_ ? "Snapshot" : "Stopped");

	ImGui::Text("Receive Rate");
	ImGui::SameLine(sts_item_idt_);
	ImGui::Text("%s", state_fps_.data());
	ImGui::SameLine(sts_fps_idt_);
	ImGui::Text(" fps");
}

void ViewerGui::drawCursorWin(void)
{
	char d_str[16];
	char i_str[16];
	char r_str[4][16];
	std::string raw_name;
	const ImGuiIO& io = ImGui::GetIO();
	bool	enable_pos = false;
	ImageKind kind;
	Point2d	mouse_pos;
	uint8_t i;
	float raw_indent = 0;

	ImGui::Text("Cursor Monitor (Average)");
	if (mouse_enter_raw_) {
		for (i = IMG_RAW1; i <= IMG_RAW4; i++) {
			if (std::dynamic_pointer_cast<DrawRaw>(draw_imgs_[i])->convPoint2d(mouse_pos_raw_, mouse_pos)) {
				enable_pos = true;
				break;
			}
		}
	} else {
		for (i = 0; i < static_cast<uint8_t>(show_img_.size()); i++) {
			switch (show_img_[i]) {
			case DISP_DEPTH:	kind = IMG_DEPTH;	break;
			case DISP_IR:		kind = IMG_IR;		break;
			default:			kind = IMG_KINDS;	break;
			}
			if (kind != IMG_KINDS) {
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
		if (draw_imgs_[IMG_DEPTH]->getValue(mouse_pos) == INVALID_DEPTH) {
			std::snprintf(d_str, sizeof(d_str), "-----");
		} else {
			std::snprintf(d_str, sizeof(d_str), "%5u", draw_imgs_[IMG_DEPTH]->getValue(mouse_pos));
		}
		std::snprintf(i_str, sizeof(i_str), "%5u", draw_imgs_[IMG_IR]->getValue(mouse_pos));
		for (i = 0; i < RAW_MAX; i++) {
			std::snprintf(r_str[i], sizeof(r_str), "%5u", draw_imgs_[IMG_RAW1 + i]->getValue(mouse_pos));
		}
	} else {
		ImGui::Text("Pos  (---,---)");
	}
	ImGui::Text("Depth");
	ImGui::SameLine(sts_cur_d_val_idt_);
	ImGui::Text("%s", (enable_pos && img_fmts_[IMG_DEPTH].isExist()) ? d_str : "-----");
	ImGui::SameLine(sts_cur_ir_idt_);
	ImGui::Text("IR");
	ImGui::SameLine(sts_cur_ir_val_idt_);
	ImGui::Text("%s", (enable_pos && img_fmts_[IMG_IR].isExist()) ? i_str : "-----");
#ifdef VIEWER_ADV
	ImGui::Text("RAW");
	for (i = 0; i < RAW_MAX; i++) {
		raw_indent += sts_cur_raw_g_idt_;
		ImGui::SameLine(raw_indent);
		raw_name = "G" + std::to_string(i + 1U) + " : ";
		ImGui::Text("%s", raw_name.c_str());
		raw_indent += sts_cur_raw_idt_;
		ImGui::SameLine(raw_indent);
		if (enable_pos && img_fmts_[IMG_RAW1 + i].isExist()) {
			ImGui::Text("%s", r_str[i]);
		} else {
			ImGui::Text("%s", "-----");
		}
	}
#endif /* VIEWER_ADV */
}

void ViewerGui::drawOtherWin(void)
{
	std::string sdk_ver = makeVerString(SDK_VERSION);
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
				if (setDeviceProperty(PlayBack::CMD_PLAY_TARGET, &play_target_path_)) {
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

void ViewerGui::showMessage(MSG_KIND msg_kind, const std::string& message)
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

uint32_t ViewerGui::convStrVal(const std::string& str, int base)
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

float ViewerGui::convDegRad(float deg)
{
	return ((deg * static_cast<float>(M_PI)) / 180.0f);
}

float ViewerGui::convRadDeg(float rad)
{
	float deg = (rad * 180.0f) / static_cast<float>(M_PI);
	deg = round(deg * 10.0f);
	return (deg / 10.0f);
}

std::string ViewerGui::makeVerString(const Version& ver)
{
	std::string str =	std::to_string(ver.major) + "." +
						std::to_string(ver.minor) + "." +
						std::to_string(ver.rev);
	return str;
}

std::string ViewerGui::makeModeString(const ModeInfo& mode_info)
{
	std::stringstream ss;
	ss << std::to_string(mode_info.id) << " : " << mode_info.description;
	return ss.str();
}

std::string ViewerGui::makeDegString(uint16_t deg)
{
	std::stringstream ss;
	uint16_t deg_u = deg / 100U;
	uint16_t deg_l = deg % 100U;
	ss << deg_u << "." << std::setw(2) << std::setfill('0') << deg_l << " (deg)";
	return ss.str();
}

void ViewerGui::makeFpsString(uint16_t fps, std::array<char, 16>& fps_str, bool stuff_left)
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

void ViewerGui::makePlayTimeString(uint32_t frame, std::array<std::string, 3>& time_str)
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

void ViewerGui::drawImage(uint8_t texture_idx, uint16_t width, uint16_t height, const std::vector<RgbaColor>& image)
{
	glBindTexture(GL_TEXTURE_2D, texture_[texture_idx]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data());
	ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(texture_[texture_idx])), ImVec2(width, height));
	glBindTexture(GL_TEXTURE_2D, 0);
}

void ViewerGui::changePcdView(const ImGuiIO& io, ImVec2 win_pos, ImVec2 win_size)
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

void ViewerGui::updatePointCloud(const ImVec2& pos, bool need_update)
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

void ViewerGui::drawPointCloud(ImDrawList* draw_list, bool need_update)
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

void ViewerGui::drawGrid(ImDrawList* draw_list)
{
	std::shared_ptr<DrawPcd> draw_pcd = std::dynamic_pointer_cast<DrawPcd>(draw_imgs_[IMG_PCD]);
	std::vector<std::pair<Point3d, Point3d>> lines;
	if (grid_view_kind_ == GRID_KIND_CAMERA) {
		draw_pcd->convScreenGridCamera((is_view_rot_changing_ || is_view_pos_changing_ || is_scale_changing_), lines);
	} else {
		draw_pcd->convScreenGridWorld((is_view_rot_changing_ || is_view_pos_changing_ || is_scale_changing_), lines);
	}
	for (auto line : lines) {
		draw_list->AddLine(ImVec2(line.first.x, line.first.y), ImVec2(line.second.x, line.second.y), grid_color_, grid_line_w_);
	}
}

void ViewerGui::updateRawCursor(GLFWwindow* /*window*/, double x, double y)
{
	mouse_pos_raw_ = {static_cast<uint16_t>(x), static_cast<uint16_t>(y)};
}

void ViewerGui::enterRawCursor(GLFWwindow* /*window*/, int entered)
{
	mouse_enter_raw_ = (entered == GL_TRUE);
}

bool ViewerGui::writeCamPrm(void)
{
	std::ifstream		ifs;
	std::error_code		err;
	CamPrmRequest		req;
	std::uintmax_t		size;

	if (cam_prm_w_file_path_.empty()) {
		LOG_ERR("Path is empty\n");
		return false;
	}

	ifs.open(cam_prm_w_file_path_, (std::ios::in | std::ios::binary));
	if (ifs.bad() || ifs.eof() || ifs.fail()) {
		LOG_ERR("Failed to open : %s\n", cam_prm_w_file_path_.string<char>().c_str());
		return false;
	}
	size = std::filesystem::file_size(cam_prm_w_file_path_, err);
	if (err || (size == 0)) {
		LOG_ERR("Failed to get filesize : %s\n", cam_prm_w_file_path_.string<char>().c_str());
		return false;
	}
	req.data.resize(size);
	ifs.read(reinterpret_cast<char*>(req.data.data()), size);
	if (ifs.bad()) {
		LOG_ERR("Failed to read : %s\n", cam_prm_w_file_path_.string<char>().c_str());
		ifs.close();
		return false;
	}
	ifs.close();

	req.kind = cam_prm_w_part_ ? CAM_PRM_PART : CAM_PRM_ALL;
	req.id = cam_prm_w_part_ ? static_cast<uint16_t>(convStrVal(cam_prm_w_id_.data(), 16)) : 0;

	return setDeviceProperty(CMD_CAM_PRM, &req);
}

bool ViewerGui::readCamPrm(void)
{
	bool				result = true;
	std::ofstream		ofs;
	CamPrmRequest		req;
	std::filesystem::path	dst = cam_prm_r_dir_path_;

	if (dst.empty()) {
		LOG_ERR("Path is empty\n");
		return false;
	}
	if (!std::filesystem::exists(dst)) {
		LOG_ERR("%s is not exist\n", dst.string<char>().c_str());
		return false;
	}

	req.kind = cam_prm_r_part_ ? CAM_PRM_PART : CAM_PRM_ALL;
	req.id = cam_prm_r_part_ ? static_cast<uint16_t>(convStrVal(cam_prm_r_id_.data(), 16)) : 0;

	if (!getDeviceProperty(CMD_CAM_PRM, &req)) {
		return false;
	}

	if (req.kind == CAM_PRM_ALL) {
		dst.append("AllParameter.bin");
	} else {
		std::array<char, 32>	dst_file;
		dst_file.fill(0);
		std::snprintf(dst_file.data(), dst_file.size(), "PartialParameter_%04X.bin", req.id);
		dst.append(dst_file.data());
	}
	try {
		if (std::filesystem::exists(dst)) {
			LOG_ERR("%s is already exist\n", dst.string<char>().c_str());
			return false;
		}
	} catch (...) {
		LOG_ERR("Failed to check %s\n", dst.string<char>().c_str());
		return false;
	}

	ofs.open(dst, (std::ios::out | std::ios::binary | std::ios::trunc));
	if (ofs.bad()) {
		LOG_ERR("Failed to open : %s\n", dst.string<char>().c_str());
		return false;
	}
	ofs.write(reinterpret_cast<char*>(req.data.data()), req.data.size());
	if (ofs.bad()) {
		LOG_ERR("Failed to write : %s\n", dst.string<char>().c_str());
		result = false;
	}
	ofs.close();

	return result;
}

bool ViewerGui::writeFirmware(void)
{
	std::ifstream		ifs;
	std::error_code		err;
	CamPrmRequest		req;
	std::uintmax_t		size;

	if (firmware_w_file_path_.empty()) {
		LOG_ERR("Path is empty\n");
		return false;
	}

	ifs.open(firmware_w_file_path_, (std::ios::in | std::ios::binary));
	if (ifs.bad() || ifs.eof() || ifs.fail()) {
		LOG_ERR("Failed to open : %s\n", firmware_w_file_path_.string<char>().c_str());
		return false;
	}
	size = std::filesystem::file_size(firmware_w_file_path_, err);
	if (err || (size == 0)) {
		LOG_ERR("Failed to get filesize : %s\n", firmware_w_file_path_.string<char>().c_str());
		return false;
	}
	req.data.resize(size);
	ifs.read(reinterpret_cast<char*>(req.data.data()), size);
	if (ifs.bad()) {
		LOG_ERR("Failed to read : %s\n", firmware_w_file_path_.string<char>().c_str());
		ifs.close();
		return false;
	}
	ifs.close();

	req.kind = firmware_w_kind_;
	req.id = 0;

	return setDeviceProperty(CMD_CAM_PRM, &req);
}

bool ViewerGui::readFirmware(void)
{
	bool				result = true;
	std::ofstream		ofs;
	CamPrmRequest		req;
	std::filesystem::path	dst = firmware_r_dir_path_;

	if (dst.empty()) {
		LOG_ERR("Path is empty\n");
		return false;
	}
	if (!std::filesystem::exists(dst)) {
		LOG_ERR("%s is not exist\n", dst.string<char>().c_str());
		return false;
	}

	req.kind = firmware_r_kind_;
	req.id = 0;

	if (!getDeviceProperty(CMD_CAM_PRM, &req)) {
		return false;
	}

	if (req.kind == CAM_PRM_ARTIX_CONF) {
		dst.append("Firmware1.bin");
	} else {
		dst.append("Firmware2.bin");
	}
	try {
		if (std::filesystem::exists(dst)) {
			LOG_ERR("%s is already exist\n", dst.string<char>().c_str());
			return false;
		}
	} catch (...) {
		LOG_ERR("Failed to check %s\n", dst.string<char>().c_str());
		return false;
	}

	ofs.open(dst, (std::ios::out | std::ios::binary | std::ios::trunc));
	if (ofs.bad()) {
		LOG_ERR("Failed to open : %s\n", dst.string<char>().c_str());
		return false;
	}
	ofs.write(reinterpret_cast<char*>(req.data.data()), req.data.size());
	if (ofs.bad()) {
		LOG_ERR("Failed to write : %s\n", dst.string<char>().c_str());
		result = false;
	}
	ofs.close();

	return result;
}

} // namespace krm
