/*------------------------------------------------------------------*/
/// @file		ViewerGui.h
/// @brief		Viewer Graphic User Interface
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <string_view>
#include <memory>
#include <filesystem>
#include <unordered_map>
#include <future>
#include <thread>

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h>
#include "imfilebrowser.h"

#include "ViewerConfig.h"
#include "ColorTable.h"
#include "GrayTable.h"
#include "DrawDepth.h"
#include "DrawGray.h"
#include "DrawRaw.h"
#include "DisplayRaw.h"
#include "DrawPcd.h"

#include "PostFilterType.h"
#include "PlFw.h"
#include "DrawThread.h"

/* #undef VIEWER_ADV */

namespace krm
{

class ViewerGui
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	ViewerGui(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~ViewerGui(void);
	/*------------------------------------------------------------------*/
	/// @brief	execute UI processing
	/*------------------------------------------------------------------*/
	void run(void);

	/* delete copy */
	ViewerGui(const ViewerGui&) = delete;
	ViewerGui& operator=(const ViewerGui& ) = delete;

private:
	static constexpr std::string_view WIN_NAME = "TOPPAN ToF Viewer";
#ifdef LNX_FUNC
	static constexpr std::string_view JP_FONT = "/usr/share/fonts/opentype/ipafont-gothic/ipag.ttf";
#else	/*  LNX_FUNC */
	static constexpr std::string_view JP_FONT = "ipag.ttf";
#endif	/*  LNX_FUNC */
	static constexpr int DIR_BROW = (
		ImGuiFileBrowserFlags_SelectDirectory |
		ImGuiFileBrowserFlags_EnterNewFilename |
		ImGuiFileBrowserFlags_NoStatusBar |
		ImGuiFileBrowserFlags_CloseOnEsc
	);
	static constexpr int FILE_BROW = (
		ImGuiFileBrowserFlags_EnterNewFilename |
		ImGuiFileBrowserFlags_NoStatusBar |
		ImGuiFileBrowserFlags_CloseOnEsc
	);
	static constexpr ImGuiInputTextFlags TEXT_DEC_FLAG =
		(ImGuiInputTextFlags_CharsDecimal | ImGuiInputTextFlags_CharsNoBlank);
	static constexpr ImGuiInputTextFlags TEXT_HEX_FLAG =
		(ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);

	static const size_t MAX_PATH_LEN = 512U;
	static constexpr std::string_view PATH_HINT = "(Enter target path)";
	static constexpr std::string_view FILE_PATH_HINT = "(Enter target file path)";

	enum TextureIndex : uint8_t {
		TEXTURE_RIGHT,
		TEXTURE_LEFT,
		TEXTURE_COLOR_BAR,
		TEXTURE_NUM
	};

	enum PlThreadKind : uint8_t {
		PLT_POSTFILTER,	/* PostFilter Thread */
		PLT_LENSCONV,	/* LensConv Thread */
		PLT_RECORD,		/* Record Thread */
		PLT_DRAW,		/* Convert Drawing Color Thread */
		PLT_NUM
	};

	enum FileBrowserTarget {
		FILE_BROWS_NONE,	/* Using Browser : none */
		FILE_BROWS_REC,		/* Using Browser : record */
		FILE_BROWS_PLAY,	/* Using Browser : playback */
		FILE_BROWS_REGLIST,	/* Using Browser : register list */
		FILE_BROWS_SNAP,	/* Using Browser : snapshot */
		FILE_BROWS_CAM_W,	/* Using Browser : camera param (write) */
		FILE_BROWS_CAM_R,	/* Using Browser : camera param (read) */
		FILE_BROWS_FIRM_W,	/* Using Browser : firmware (write) */
		FILE_BROWS_FIRM_R,	/* Using Browser : firmware (read) */
	};

	bool				running_;		/* runnning flag */
	GLFWwindow*			win_;			/* drawing window */
	GLuint				texture_[TEXTURE_NUM];		/* drawing texture */
	ImGui::FileBrowser	dir_browser_;	/* directory browser */
	ImGui::FileBrowser	file_browser_;	/* file browser */
	FileBrowserTarget	browser_target_;	/* target browser */

	std::unique_ptr<PlFw>	cam_obj_;		/* Camera pipeline */
	std::unique_ptr<PlFw>	play_obj_;		/* PlayBack pipeline */
	std::array<std::vector<uint16_t>, 2>	proc_ids_;	/* Processing IDs : 0..Camera, 1..PlayBack */

	std::vector<std::string>	dev_list_;	/* List of Target Device */
	std::vector<ConnDevice>		cam_list_;	/* Connected camera list */
	std::vector<ConnDevice>		play_list_;	/* Connected PlayBack list */

	ModeList					mode_list_;		/* Motion Mode list */
	uint8_t						mode_;			/* Motion Mode */
	ModeInfo*					cur_mode_info_;	/* Current Motion Mode information */
	ImgOutKind					out_kind_;		/* Current output image kind */
	ImageFormats				img_fmts_;		/* Current image formats */
	LightTimesInfo				light_times_;	/* Current light times */
	RegDevs						reg_devs_;		/* Register information */

	/* Panel Layout */
	static const int CTRL_WIN_W = 300;							/* Control panel : width */
	static constexpr int IMG_WIN_W = 640 + 16;					/* Image panel : width */
	static constexpr int IMG_WIN_H = 480 + 36;					/* Image panel : height */
	static constexpr int STS_WIN_W = (IMG_WIN_W * 2) / 3;		/* Status panel : width */
	static const int STS_WIN_H = 84;							/* Status panel : height */
	static constexpr int WIN_W = CTRL_WIN_W + (IMG_WIN_W * 2);	/* Window : width */
	static constexpr int WIN_H = IMG_WIN_H + STS_WIN_H;			/* Window : height */
	static const int FILE_WIN_X = 0;
	static const int FILE_WIN_Y = 0;
	static const int FILE_WIN_W = 700;
	static const int FILE_WIN_H = 450;
	int			win_w_;
	int			win_h_;
	ImVec2		pnl_ctrl_pos_;		/* Control panel : pos */
	ImVec2		pnl_ctrl_size_;		/* Control panel : size */
	ImVec2		pnl_view_pos_;		/* Pos : View panel : pos */
	ImVec2		pnl_view_size_;		/* Pos : View panel : size */
	ImVec2		pnl_sts0_pos_;		/* Pos : Status panel 0 : pos */
	ImVec2		pnl_sts1_pos_;		/* Pos : Status panel 1 : pos */
	ImVec2		pnl_sts2_pos_;		/* Pos : Status panel 2 : pos */
	ImVec2		pnl_sts0_size_;		/* Pos : Status panel 0/1 : size */
	ImVec2		pnl_sts2_size_;		/* Pos : Status panel 2 : size */
	ImVec2		pnl_msg_pos_;		/* Pos : View panel : pos */
	ImVec2		pnl_msg_size_;		/* Pos : View panel : size */

	/* Popup messages */
	static constexpr float POP_MSG_WIN_H = 72.F;
	static const int POP_MSG_WIN_W_OFST = 30;
	static const int POP_MSG_WIN_STR_W = 8;

	enum MSG_KIND : uint8_t {
		MSG_WARN = 0,
		MSG_CAUTION,
		MSG_ATTENTION,
		MSG_INFOMATION,
		MSG_KINDS
	};
	static constexpr std::array<std::string_view, MSG_KINDS> MSG_KIND_STR = {
		"Warning",
		"Caution",
		"Attention",
		"Information"
	};
	bool			show_message_;		/* showing message */
	bool			show_msg_closing_;	/* show close button in message */
	bool			exit_viewer_;		/* exit viewer after message closed */
	std::string		message_lbl_;		/* message window label */
	std::string		message_str_;		/* message strings */


	/*--- Control Panel Variables ---*/
	static constexpr ImGuiWindowFlags CTRL_WIN_FLAG = (
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
	ImVec4 btn_color_[3];
	static constexpr ImGuiTableFlags INF_TBL_FLAGS = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp;
	static constexpr ImGuiTableFlags TBL_FLAGS = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg;

	static constexpr float UI_HEADER_W = 10.F;
	static constexpr float UI_NODE_W = 23.F;
	static constexpr float CTRL_PARTS_W = static_cast<float>(CTRL_WIN_W) - UI_HEADER_W;
	float ctrl_parts_w_;
	static constexpr float CTRL_NODE_W = CTRL_PARTS_W - UI_NODE_W;
	float ctrl_node_w_;
	static constexpr float CTRL_BTN_W = 130.F;					/* Button width */
	float ctrl_btn_w_;
	static constexpr float INDENT_LINE = 35.F;
	static constexpr float INDENT_SPACE = 2.F;
	static constexpr float CTRL_BTN_INDENT = CTRL_BTN_W + INDENT_LINE + INDENT_SPACE;
	float ctrl_btn_idt_;
	static constexpr float CTRL_DIR_BTN_W = 80.F;
	static constexpr float CTRL_DIR_TXT_W = CTRL_NODE_W - CTRL_DIR_BTN_W - INDENT_SPACE;
	static constexpr float CTRL_DIR_IDT = CTRL_DIR_BTN_W + 38.F - 1.F;
	float ctrl_dir_btn_w_;
	float ctrl_dir_txt_w_;
	float ctrl_dir_idt_;

	/* Target Device */
	static constexpr float TGT_RELOAD_IDT = CTRL_PARTS_W - CTRL_BTN_W - UI_NODE_W;
	float	tgt_reload_idt_;

	static constexpr uint32_t PLAYBACK_INDEX = UINT16_MAX + 1U;
	bool		tgt_dev_open_;
	bool		tgt_dev_enable_;			/* Combo : Target Device is enabled */
	bool		tgt_dev_selected_;
	uint16_t	tgt_dev_sel_idx_;			/* Combo : Selecting index */
	uint32_t	tgt_cam_idx_;				/* Combo : Camera index / PLAYBACK_INDEX:PlayBack */
	bool		tgt_btn_reload_;			/* Button : Reload Target Device(true) / Close Device(false) */

	/* Device Control */
	static const int DEV_INF_ROW = 2;		/* Information Table : Row */
	static const uint8_t CAM_INF_NUM = 23U;	/* Information Table : Column */
	/* Control Camera device */
	static constexpr float CAM_REG_TGT_W = CTRL_DIR_TXT_W - INDENT_SPACE;
	static constexpr float CAM_REG_IDT = 160.F;
	static constexpr float CAM_REG_W = 100.F;
	static constexpr float CAM_REC_LEN_W = 100.F;
	float cam_reg_tgt_w_;
	float cam_reg_idt_;
	float cam_reg_w_;
	float cam_rec_len_w_;

	static constexpr std::array<std::string_view, CAM_INF_NUM> CAM_INF_LIST = {
		"HW kind(HEX)",
		"Serial No.",
		"Map Version",
		"Adjust No.",
		"Firm Version",
		"LD wave",
		"Number of LDs",
		"Correct Calib Rev",
		"Field of View(H)",
		"Field of View(V)",
		"Camera Median Filter",
		"Camera Bilateral Filter",
		"Camera Flying Pixel Filt",
		"Camera Dist",
		"Lens Calib Rev",
		"Distance Range (Min)",
		"Distance Range (Max)",
		"Framerate",
		"Thinning out(H)",
		"Thinning out(V)",
		"Crop start(X,Y)",
		"Range Calib Rev",
	};
	static constexpr std::array<std::string_view, 5> IMG_OUT_KIND_LIST = {
		"Depth only",
		"IR only",
		"Depth + IR",
		"Depth + IR + RAW",
		"RAW only",
	};
	enum ShowImagePair {
		SHOW_PAIR_DEPTH_IR,
		SHOW_PAIR_DEPTH_PCD,
		SHOW_PAIR_IR_PCD,
		SHOW_PAIR_IR,
		SHOW_PAIR_KINDS,
	};
	static constexpr std::array<std::string_view, SHOW_PAIR_KINDS> SHOW_PAIR_LIST = {
		"Depth + IR",
		"Depth + Point Cloud",
		"IR + Point Cloud",
		"IR",
	};
	enum ShowDisplayKind {
		DISP_DEPTH,
		DISP_IR,
		DISP_PCD,
		DISP_RAW,
		DISP_NONE,
		DISP_KINDS
	};
	static constexpr std::array<std::string_view, DISP_KINDS> SHOW_DISP_LIST = {
		"Depth Image",
		"IR Image",
		"Point Cloud",
		"RAW Image",
		"None Image",
	};
	static constexpr std::array<std::string_view, DISP_KINDS> SHOW_SNAP_DISP_LIST = {
		"Depth Image (Snapshot)",
		"IR Image (Snapshot)",
		"Point Cloud (Snapshot)",
		"RAW Image (Snapshot)",
		"None Image",
	};
	static constexpr std::array<std::string_view, 2> REG_TARGET_LIST = {
		"Image Sensor",
		"Image Processor",
	};

	bool		cap_dev_con_;						/* Button : Start Capture(true) / Close Capture(false)*/
	bool		cam_dev_open_;						/* Open a label */
	bool		cam_cap_disable_;					/* Device Control "write button" : able or unable */
	std::vector<std::string>	cam_mode_list_;		/* Camera device mode selection list*/
	uint8_t						cam_mode_sel_;		/* Camera mode selection */
	uint8_t						ini_mode_;			/* initial motion mode */
	std::vector<std::string>	cam_image_list_;	/* Combo : Camera Image List*/
	std::array<std::string, CAM_INF_NUM>	cam_inf_list_;	/* Table : Device information */
	std::vector<std::string>	cam_reg_target_list_;		/* List of Register target */
	uint16_t	cam_image_idx_;						/* Combo : Selecting index */
	std::array<ShowDisplayKind, 2>	show_img_;		/* Showing display kinds */
	std::array<ImageFormat, 2>		show_fmt_;		/* Showing display image format */
	bool		cam_disable_raw_;					/* enable RAW expand */
	int			cam_light_slid_;
	bool		cam_disable_light_time_;
	ImageKind	cam_light_cnt_img_;
	bool		cam_ae_enable_;
	AEIntervalInfo		cam_ae_interval_;
	SignalThresholdInfo	cam_raw_sat_th_;
	SignalThresholdInfo	cam_ir_dark_th_;
	const std::vector<std::string>	cam_ext_trg_list_;
	ExtTriggerType	cam_ext_trigger_;
	bool		cam_ext_trg_disable_;
	float		cam_ext_trg_offset_;
	const std::vector<std::string>	cam_int_supp_mode_list_;
	IntSuppInfo		cam_int_supp_info_;

	bool		cam_rst_dev_;						/* Combo : Register is enabled */
	uint16_t	cam_reg_target_idx_;				/* Combo : Selecting index */
	bool		recording_;							/* Button : Start Record(true) / Stop Record(false) */

	std::array<char, MAX_PATH_LEN>	cam_reg_file_path_c_;
	std::filesystem::path			cam_reg_file_path_;
	std::array<char, MAX_PATH_LEN>	cam_rec_dir_path_c_;
	std::filesystem::path			cam_rec_dir_path_;
	std::array<char, 5>		cam_reg_addr_;			/* register address */
	std::array<char, 5>		cam_reg_val_;			/* register value */
	std::array<char, 11>	cam_rec_len_;			/* Record length [sec] : uint32_t */
	uint8_t					cam_rec_packing_;


	/* PlayBack */
	static constexpr float PLAY_TIME_W = 50.F;
	static constexpr float PLAY_TIME_SPACE = 2.F;
	static constexpr float PLAY_JUMP_W = 70.F;
	static constexpr float PLAY_JUMP_IDT = CTRL_NODE_W - PLAY_JUMP_W - 29.F;
	static constexpr float PLAY_TIME_IDT = 32.F;
	float play_time_w_;
	float play_time_space_;
	float play_jump_w_;
	float play_jump_idt_;
	float play_time_idt_;

	static const int PLAY_SLIDER_MIN = 0;
	static const int PLAY_SLIDER_MAX = 100;

	std::array<char, MAX_PATH_LEN>	play_target_path_c_;
	std::filesystem::path			play_target_path_;
	bool			play_cap_disable_;
	bool			play_pausing_;
	uint8_t			play_jump_time_sec_;
	uint32_t		play_jump_time_;
	uint16_t		play_jump_val_;
	std::array<std::array<char, 9>, 3>	play_jump_;
	std::string		play_time_;
	std::array<std::string, 3>	play_time_total_;
	int				play_con_timer_;
	std::array<char, 16>	play_frame_fps_;

	/* PostFilter */
	std::vector<uint8_t>		pstf_ksize_map_;
	std::vector<std::string>	pstf_ksize_list_;
	std::vector<std::string>	pstf_method_list_;
	std::vector<std::string>	pstf_priority_list_;
	bool						pstf_enable_medf_;
	bool						pstf_medf_check_disable_;
	bool	                    pstf_enable_bilf_;
	bool						pstf_bilf_check_disable_;
	bool						pstf_enable_flypf_;
	bool						pstf_flypf_check_disable_;

	PostFilterPrm				pstf_prm_;
	static constexpr float PSTF_OFST_W = 170.F;
	static constexpr float PSTF_INP_W = 120.F;
	float pstf_ofst_w_;
	float pstf_inp_w_;

	/* Post Proccess */
	static constexpr float PST_PCD_OFST_W = 140.F;
	static constexpr float PST_PCD_OFST_IDT = 60.F;
	static constexpr float PST_PCD_ROT_W = 140.F;
	static constexpr float PST_PCD_ROT_IDT = 60.F;
	static constexpr float PST_PFT_OFST_W = 170.F;
	static constexpr float PST_PFT_INP_W = 80.F;
	float pst_pcd_ofst_w_;
	float pst_pcd_ofst_idt_;
	float pst_pcd_rot_w_;
	float pst_pcd_rot_idt_;

	static const int PST_PCD_OFST_STEP = 100;
	static constexpr float PST_PCD_ROT_MIN = -180.F;
	static constexpr float PST_PCD_ROT_MAX = 180.F;

	
	bool	pst_dist_check_disable_;
	bool	pst_dist_enable_;		/* CheckBox : Distortion correction(Set to Depth, IR, RAW) */
	int		pst_pcd_origin_;		/* RadioButton : Point Cloud origin */
	bool	pst_pcd_cam_coord_;		/* RadioButton : enable Origin Offset & Rotation */
	int		pst_pcd_ofst_x_;		/* Origin Offset[X] Value */
	int		pst_pcd_ofst_y_;		/* Origin Offset[Y] Value */
	int		pst_pcd_ofst_z_;		/* Origin Offset[Z] Value */
	float	pst_pcd_rot_x_;			/* Rotation[X] Value */
	float	pst_pcd_rot_y_;			/* Rotation[Y] Value */
	float	pst_pcd_rot_z_;			/* Rotation[Z] Value */

	/* View Settings */
	static constexpr float VIEW_DPT_RNG_W = 58.F;
	static constexpr float VIEW_DPT_RNG_IDT = CTRL_NODE_W - VIEW_DPT_RNG_W;
	static constexpr float VIEW_DPT_W = 130.F;
	static const uint16_t VIEW_DPT_BAR_W = static_cast<uint16_t>(CTRL_NODE_W);
	static const uint16_t VIEW_DPT_BAR_H = 20U;
	static constexpr float VIEW_GRAY_IDT = 100.F;
	static constexpr float VIEW_GRAY_W = 130.F;
	static constexpr float VIEW_PCD_INPUT_IDT = 100.F;
	static constexpr float VIEW_PCD_INPUT_W = 80.F;
	static constexpr float VIEW_PCD_ANG_BTN = 50.F;
	float view_dpt_rng_idt_;
	float view_dpt_w_;
	float view_dpt_btn_idt_;
	uint16_t view_dpt_bar_w_;
	uint16_t view_dpt_bar_h_;
	float view_gray_idt_;
	float view_gray_w_;
	float view_pcd_grid_idt_;
	float view_pcd_input_idt_;
	float view_pcd_input_w_;
	float view_pcd_ang_btn_;

	static const int VIEW_DEPTH_DRAG_VAL = 100;
	static constexpr int VIEW_DEPTH_MAX = static_cast<int>(INVALID_DEPTH - 1U);
	static const int VIEW_COLOR_DIFF_MIN = 8;
	static constexpr float VIEW_GRAY_GAIN_MIN = 0.1F;
	static constexpr float VIEW_GRAY_GAIN_MAX = 50.0F;
	static constexpr float VIEW_GRAY_GAMMA_MIN = 0.1F;
	static constexpr float VIEW_GRAY_GAMMA_MAX = 5.0F;
	static const int VIEW_AUX_DRAG_VAL = 10;
	static const int VIEW_AUX_MIN = 100;
	static const int VIEW_AUX_MAX = 10000;
	static const int VIEW_INTLV_DIST_MIN = -100000;
	static const int VIEW_INTLV_DIST_MAX = 100000;
	static const int VIEW_INTLV_DRAG_VAL = 10;
	enum PcdColor {
		PCD_C_DEPTH = 0,	/* Depth Color */
		PCD_C_IR			/* Grayscale(IR) Color */
	};
	enum WinSize {
		DEFAULT_WIN_SIZE = 0,	/* default size */
		DOUBLE_WIN_SIZE			/* double size */
	};
	int			view_dep_range_min_;
	int			view_dep_range_max_;
	int			view_dep_range_min_max_;
	int			view_dep_range_max_min_;
	int			view_dep_min_;
	int			view_dep_max_;
	Range		view_depth_range_;
	std::vector<RgbaColor>	color_bar_;
	float		view_gray_gain_;
	float		view_gray_gamma_;
	std::vector<std::string>	view_image_list_;		/* Combo : View Images list */
	std::vector<ShowImagePair>	view_image_pair_;		/* viewing image pair */
	std::vector<std::string>	view_ima_show_list_;	/* Combo : View Images show list */
	std::string	view_image_show_r_;
	std::string	view_image_show_l_;
	uint16_t	view_image_idx_;
	bool		view_aux_check_;					/* CheckBox : Auxiliary line */
	int			view_aux_grid_;						/* Rotation Value */
	int			view_radio_button_;					/* RadioButton : Point Color */
	bool		view_disable_winsize_;				/* Disable Window Size */
	int			view_radio_winsize_;				/* RadioButton : Window Size */
	bool		view_intlv_enable_;					/* CheckBox : Interleaving */
	int			view_intlv_distance_;				/* Distance */
	int			view_intlv_factor_;					/* RadioButton : Sampling Factor */

	/* Snapshot */
	static const uint8_t	SNAP_FRMS_NUM = 5U;
	static constexpr std::array<uint8_t, SNAP_FRMS_NUM> SNAP_FRMS = {		/* List of Snapshot frame count */
		1U, 16U, 32U, 64U, 128U};
	std::vector<std::string> snap_frms_list_;
	uint16_t	snap_frms_idx_;					/* Combo : Selecting index */
	std::array<char, MAX_PATH_LEN>	snap_tgt_dir_path_c_;
	std::filesystem::path			snap_tgt_dir_path_;
	enum SnapFileFormat {
		SNAP_FILE_BIN = 0,	/* binary format */
		SNAP_FILE_CSV,		/* CSV format */
	};
	int			snap_file_fmt_;				/* save file format */
	uint8_t		snap_took_frms_;			/* took frame count */
	bool		snap_taking_;				/* taking snapshot */
	bool		snap_disable_take_;			/* disable to take snapshot */
	bool		snap_disable_exit_;			/* disable to exit snapshot */
	bool		snap_disable_save_;			/* disable to save snapshot */

	/* Camera Parameter */
	bool		cam_prm_disabled_;			/* disable set/get camera parameter */
	int			cam_prm_w_selected_;		/* RadioButton : Write Parameter */
	bool		cam_prm_w_part_;			/* Write target is part */
	std::array<char, 5> 			cam_prm_w_id_;
	std::array<char, MAX_PATH_LEN>	cam_prm_w_file_path_c_;
	std::filesystem::path			cam_prm_w_file_path_;
	int			cam_prm_r_selected_;		/* RadioButton : Read Parameter */
	bool		cam_prm_r_part_;			/* Read target is part */
	std::array<char, 5>				cam_prm_r_id_;
	std::array<char, MAX_PATH_LEN>	cam_prm_r_dir_path_c_;
	std::filesystem::path			cam_prm_r_dir_path_;

	/* Firmware */
	int								firmware_w_selected_;	/* RadioButton : Write Parameter */
	krm::CamPrmKind					firmware_w_kind_;		/* write target */
	std::array<char, MAX_PATH_LEN>	firmware_w_file_path_c_;
	std::filesystem::path			firmware_w_file_path_;
	int								firmware_r_selected_;	/* RadioButton : Read Parameter */
	krm::CamPrmKind					firmware_r_kind_;		/* read target */
	std::array<char, MAX_PATH_LEN>	firmware_r_dir_path_c_;
	std::filesystem::path			firmware_r_dir_path_;

	/*--- View Panel Variables ---*/
	static constexpr ImGuiWindowFlags VIEW_WIN_FLAG = (
		ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);


	/*--- Status Panel Variables ---*/
	static constexpr ImGuiWindowFlags STS_WIN_FLAG = (
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
		ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoMove);
	static constexpr float STS_ITEM_IDT = 150.F;
	static constexpr float STS_FPS_IDT = STS_ITEM_IDT + 50.F;
	static constexpr float STS_CUR_D_VAL_IDT = 80.F;
	static constexpr float STS_CUR_IR_IDT = 180.F;
	static constexpr float STS_CUR_IR_VAL_IDT = 230.F;
	static constexpr float STS_CUR_RAW_G_IDT = 60.F;
	static constexpr float STS_CUR_RAW_IDT = 32.F;
	static constexpr float STS_OTH_IDT = 200.F;
	static constexpr float STS_OTH_BTN_W = 80.F;
	float sts_item_idt_;
	float sts_fps_idt_;
	float sts_cur_d_val_idt_;
	float sts_cur_ir_idt_;
	float sts_cur_ir_val_idt_;
	float sts_cur_raw_g_idt_;
	float sts_cur_raw_idt_;
	float sts_oth_idt_;
	float sts_oth_btn_w_;

	enum CamPlayState : uint8_t {
		ST_CLOSED,
		ST_STOPPED,
		ST_STREAMING,
		ST_PAUSE_PLAY,
		ST_FAST_PLAY,
		ST_SLOW_PLAY,
		ST_TIMEOUT,
		ST_NUM
	};
	static constexpr std::array<std::string_view, ST_NUM> STATE_CAM_LIST = {
		"Closed",
		"Stopped",
		"Streaming",
		"Pause",
		"Fast",
		"Slow",
		"Stopping(Timeout)"
	};
	enum RecState : uint8_t {
		ST_REC_STOP,
		ST_REC_RECORD,
		ST_REC_FINISH,
		ST_REC_FULL,
		ST_REC_NUM
	};
	static constexpr std::array<std::string_view, ST_REC_NUM>	STATE_REC_LIST = {
		"Stopped",
		"Recording",
		"Finished",
		"Failed(Disk full)"
	};

	std::array<ImVec2, 2>	img_win_pos_;
	CamPlayState			state_cam_;
	RecState				state_rec_;
	std::array<char, 16>	state_fps_;

	bool			stat_disable_cfg_save_;
	bool			stat_disable_cfg_load_;
	std::string		hide_ctrl_panel_str_;
	bool			hide_ctrl_panel_;			/* true : hide, false : show */

	static Point2d	mouse_pos_raw_;
	static bool		mouse_enter_raw_;

	/* Configuration */
	ViewerConfig	config_;

	/* Draw images */
	std::shared_ptr<ColorTable>		color_table_;
	std::shared_ptr<GrayTable>		gray_table_;
	DrawImages						draw_imgs_;
	std::shared_ptr<DisplayRaw>		raw_win_;
	std::vector<Point3d>			draw_points_;
	std::vector<Point3d>			draw_image_;

	/* Changing viewing position of PCD View */
	bool		is_view_rot_changing_;
	bool		is_view_pos_changing_;
	bool		is_scale_changing_;

	/* Parameters for drawing Grid line*/
	enum GridKind {
		GRID_KIND_CAMERA,
		GRID_KIND_ORIGIN,
		GRID_KIND_FORCE_ORIGIN,
	};
	GridKind	grid_view_kind_;
	float		grid_line_w_;		/* size of grid line */
	ImU32		grid_color_;		/* color of grid line */
	float		pcd_point_size_;	/* size of point */

	/*--- Show display function ---*/
	bool initialize(void);
	void terminate(void);
	static void gl_error(int error, const char* description);

	bool init_device(void);
	void reloadDevice(void);

	PlFw* getCamera(void);
	uint16_t getProcId(PlThreadKind thd);
	bool openDevice(void);
	void closeDevice(void);
	void startCapture(void);
	void stopCapture(void);
	bool getDeviceProperty(uint16_t cmd, void* param);
	bool setDeviceProperty(uint16_t cmd, void* param = nullptr);

	void updateDeviceInfo(void);
	void updateModeInfo(void);
	void updateImgKind(void);
	void updateImgFmts(void);
	void updateShowImgKind(void);
	void updateColorRange(const Range& d_range);

	void updatePlayInfo(bool update_time = true);
	void updatePlayingTime(const PlayBack::PlayTime& time, bool update_total = false);

	bool updateCapture(void);

	void notifyPcdPrm(void);

	void startRecord(void);
	void stopRecord(void);
	void setRecState(RecState state);

	void takeSnapshot(void);
	void exitSnapshot(void);
	void saveSnapshot(void);

	void loadConfig(void);
	void saveConfig(void);
	void disableCfgBtn(void);
	void enableCfgBtn(void);

	inline bool isCapturing(void)
	{
		return ((state_cam_ == ST_STREAMING) ||
				(state_cam_ == ST_FAST_PLAY) || (state_cam_ == ST_SLOW_PLAY));
	}
	inline bool isCaptureStopped(void)
	{
		return ((state_cam_ == ST_CLOSED) || (state_cam_ == ST_STOPPED));
	}

	bool createWindow(void);
	void destroyWindow(void);
	void setFont(void);

	bool drawButton(const std::string& str, float width = 0, bool disable = false);
	bool drawCombo(const std::string& label, const std::vector<std::string>& list, uint16_t& index, bool disable = false);
	bool convToCharPath(const std::filesystem::path& path, std::array<char, MAX_PATH_LEN>& c_path);
	void convToFsPath(const std::array<char, MAX_PATH_LEN>& c_path, std::filesystem::path& path);

	void drawPopUpMessage(void);

	void updateLayout();

	/*--- Draw Control Panel ---*/
	void drawCtrlPanel(void);
	void drawTgtDevLbl(void);
	void drawDevCtrlLbl(void);

	void drawDevCtrlCam(void);
	void drawDevCtrlPlay(void);

	void drawPostFiltLbl(void);

	void drawPostProcLbl(void);

	void drawViewStgLbl(void);

	void drawSnapShotLbl(void);

	void drawCamParaLbl(void);

	/*--- Draw View Panel ---*/
	void drawViewPanel(void);

	/*--- Draw Status Panel ---*/
	void drawStatusPanel(void);
	void drawStatusWin(void);
	void drawCursorWin(void);
	void drawOtherWin(void);

	/* show message */
	void showMessage(MSG_KIND msg_kind, const std::string& message);
	/* other */
	uint32_t convStrVal(const std::string& str, int base = 10);
	float convDegRad(float deg);
	float convRadDeg(float rad);
	std::string makeVerString(const Version& ver);
	std::string makeModeString(const ModeInfo& mode_info);
	std::string makeDegString(uint16_t deg);
	void makeFpsString(uint16_t fps, std::array<char, 16>& fps_str, bool stuff_left=false);
	void makePlayTimeString(uint32_t frame, std::array<std::string, 3>& time_str);
	/* image */
	void drawImage(uint8_t texture_idx, uint16_t width, uint16_t height, const std::vector<RgbaColor>& image);
	/* point cloud */
	void changePcdView(const ImGuiIO& io, ImVec2 win_pos, ImVec2 win_size);
	void updatePointCloud(const ImVec2& pos, bool need_update = false);
	void drawPointCloud(ImDrawList* draw_list, bool need_update);
	void drawGrid(ImDrawList* draw_list);

	/* Mouse CB */
	static void updateRawCursor(GLFWwindow* window, double x, double y);
	static void enterRawCursor(GLFWwindow* window, int entered);

	/* update Camera */
	bool writeCamPrm(void);
	bool readCamPrm(void);
	bool writeFirmware(void);
	bool readFirmware(void);
};

} // namespace krm
