/*------------------------------------------------------------------*/
/// @file		CameraType.h
/// @brief		Camera class definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <string>
#include <vector>
#include "TpTofSdkDefine.h"

/* #undef DEVELOP */

namespace krm
{
/*-----------------------------------------------------------------*/
/// @brief	Camera type
/*-----------------------------------------------------------------*/
enum CameraType {
	C11_USB = 0,	/*!< C11(USB I/F) */
	PLAYBACK,			/*!< File PlayBack */
};

/*-----------------------------------------------------------------*/
/// @brief	Property commands
/*-----------------------------------------------------------------*/
enum PropCmd : uint16_t {
	CMD_DEV_INFO = 0,	/*!< Camera device information : [parameter type] DeviceInfo */
	CMD_FOV,			/*!< Field of View : [parameter type] CamFov */
	CMD_EXT_TRG_TYPE,	/*!< Extension Trigger Type : [parameter type] ExtTriggerType */
	CMD_EXT_TRG_OFFSET,	/*!< Extension Trigger Offset : [parameter type] uint8_t */
	CMD_MODE_LIST,		/*!< List of Motion modes : [parameter type] ModeList */
	CMD_MODE,			/*!< Motion mode : [parameter type] uint8_t */
	CMD_IMG_KINDS,		/*!< Kind of camera output images : [parameter type] ImgOutKind */
	CMD_IMG_FORMAT,		/*!< Image formats : [parameter type] ImageFormats */
	CMD_POSTFILT_INFO,	/*!< PostFilter information : [parameter type] PostFiltInfo */
	CMD_LENS_INFO,		/*!< Parameters for Lens conversion : [parameter type] LensInfo */
	CMD_LIGHT_TIMES,	/*!< Light times : [parameter type] LightTimesInfo */
	CMD_AE_STATE,		/*!< Auto Exposure State : [parameter type] bool */
	CMD_AE_INTERVAL,	/*!< Interval for Auto Exposure : [parameter type] AEIntervalInfo */
	CMD_RAW_SAT_TH,		/*!< RAW Saturation Threshold : [parameter type] SignalThresholdInfo */
	CMD_IR_DARK_TH,		/*!< IR Dark Threshold : [parameter type] SignalThresholdInfo */
	CMD_INT_SUPP_INFO,	/*!< Interference Suppression Info : [parameter type] IntSuppInfo */
	CMD_CAM_PRM,		/*!< Camera parameters (only DEVELOP) : [parameter type] CamPrmRequest */
	CMD_REG_DEVS,		/*!< Register controllable devices (only DEVELOP) : [parameter type] RegDevs */
	CMD_REG,			/*!< Device register (only DEVELOP) : [parameter type] RegInfo */
	CMD_REG_LIST,		/*!< List of device register (only DEVELOP) : [parameter type] RegList */
	CMD_REG_REFRESH,	/*!< Refresh to original registers (only DEVELOP) : [parameter type] none */
	CMD_OP_MODE,		/*!< Operation Mode (only DEVELOP) : [parameter type] OperationMode */
	CMD_CALB_OP_INFO,	/*!< Information of Calibration Operation Mode (only DEVELOP) : [parameter type] CalbOpInfo */
	CMD_USB_PC_ACC_KEY,	/*!< USB Peripheral Controller Access key (only DEVELOP) : [parameter type] UsbPCAccKey */
	CMD_USB_PC_UPDATE,	/*!< Turn USB Peripheral Controller into updating mode (only DEVELOP) : [parameter type] none */
	CMD_MAX				/*!< Number of common property commands */
};

/*-----------------------------------------------------------------*/
/// @brief	Operation Mode
/*-----------------------------------------------------------------*/
enum OperationMode {
	OP_NORMAL = 0,		/*!< Normal operation */
	OP_CALIB,			/*!< Calibration operation (only DEVELOP) */
};

/*-----------------------------------------------------------------*/
/// @brief	Extension Trigger Type
/*-----------------------------------------------------------------*/
enum ExtTriggerType : uint8_t {
	EXT_TRG_STANDALONE = 1U,	/*!< Standalone */
	EXT_TRG_SLAVE,				/*!< Slave */
	EXT_TRG_MASTER,				/*!< Master */
};

/*-----------------------------------------------------------------*/
/// @brief	Interference Suppression Mode Type
/*-----------------------------------------------------------------*/
enum IntSuppModeType : uint8_t {
	INT_SUPP_MODE_OFF = 0,		/*!< No Interference Suppression */
	INT_SUPP_MODE_MANUAL,		/*!< Manual Mode */
	INT_SUPP_MODE_AUTO,			/*!< Auto Mode */
};

/*-----------------------------------------------------------------*/
/// @brief	Kinds of output images
/*-----------------------------------------------------------------*/
enum ImgOutKind {
	OUT_IMG_DEPTH = 0,		/*!< Depth only */
	OUT_IMG_IR,				/*!< IR only */
	OUT_IMG_DEPTH_IR,		/*!< Depth + IR */
	OUT_IMG_DEPTH_IR_RAW,	/*!< Depth + IR + Sensor RAW */
	OUT_IMG_RAW				/*!< Sensor RAW only */
};

/*-----------------------------------------------------------------*/
/// @brief	Kind of camera parameter
/*-----------------------------------------------------------------*/
enum CamPrmKind : uint8_t {
	CAM_PRM_ALL,		/*!< All parameters */
	CAM_PRM_PART,		/*!< Partial parameter */
	CAM_PRM_ARTIX_CONF,	/*!< ARTIX configuration parameter */
	CAM_PRM_FIRM_2		/*!< Firmware image */
};

/*-----------------------------------------------------------------*/
/// @brief	Register controllable device type
/*-----------------------------------------------------------------*/
enum RegDevType {
	REG_IMG_SENSOR = 0,	/*!< Image sensor */
	REG_IMG_PROCESSOR,	/*!< Image processor */
};

/*-----------------------------------------------------------------*/
/// @brief	Connected Camera device
/*-----------------------------------------------------------------*/
struct ConnDevice {
	uint16_t	id;		/*!< Camera device ID */
	std::string	name;	/*!< Camera device name */
};

/*-----------------------------------------------------------------*/
/// @brief	Calibration Operation Information
/*-----------------------------------------------------------------*/
struct CalbOpInfo {
	ImgOutKind		kind;		/*!< Kinds of output images */
	uint16_t		image_w;	/*!< Sensor image width [pixel] */
	uint16_t		image_h;	/*!< Sensor image height [pixel] */
	uint8_t			sub_num;	/*!< number of sub frame : single=1, 2sub=2, 3sub=3 */
	uint8_t			num_raw;	/*!< Number of RAW images */
	uint16_t		fps;		/*!< Framerate [fps * 100] */
};

/*-----------------------------------------------------------------*/
/// @brief	Operation Information
/*-----------------------------------------------------------------*/
struct OpInfo {
	OperationMode	mode;		/*!< Operation Mode */
	CalbOpInfo		calib_prm;	/*!< Parameter for Calibration operation */
};

/*-----------------------------------------------------------------*/
/// @brief	Camera device information
/*-----------------------------------------------------------------*/
struct DeviceInfo {
	uint32_t	hw_kind;		/*!< H/W kind */
	uint32_t	serial_no;		/*!< Serial Number */
	Version		map_ver;		/*!< Camera device parameter version */
	Version		firm_ver;		/*!< Camera device firmware version */
	uint32_t	adjust_no;		/*!< Adjust Number */
	uint16_t	ld_wave;		/*!< LD wavelength [nm] */
	uint16_t	ld_enable;		/*!< Enable for each LDs(Number of LDs) */
	uint16_t	correct_calib;	/*!< Correction parameters calibration revision */
};


/*-----------------------------------------------------------------*/
/// @brief	Parameters for Lens conversion
/*-----------------------------------------------------------------*/
struct LensInfo {
	uint16_t					sens_w;		/*!< Sensor width [pixel] */
	uint16_t					sens_h;		/*!< Sensor height [pixel] */
	uint32_t					focal_len;	/*!< Focal length : FixedPoint[Integer:12bit/Decimal:32bit] */
	uint8_t						thin_w;		/*!< Thinning number(Horizontal) */
	uint8_t						thin_h;		/*!< Thinning number(Vertical) */
	Point2d						crop;		/*!< Crop point from Sensor */
	bool						cam_dist;	/*!< Dist correction is already done in Camera device */
	uint64_t					dist[9];	/*!< Parameter for distortion correction : FixedPoint[Sign:1bit/Integer:16bit/Decimal:47bit] */
	uint16_t					lens_calib;	/*!< Lens parameters calibration revision */
};

/*-----------------------------------------------------------------*/
/// @brief	Parameters for Post Filter
/*-----------------------------------------------------------------*/
struct PostFiltInfo {
	bool	cam_med_filt;	/*!< Median Filter is already done in Camera device */
	bool	cam_bil_filt;	/*!< Bilateral Filter is already done in Camera device */
	bool	cam_fly_p_filt;	/*!< Flying Pixel Filter is already done in Camera device */
};

/*-----------------------------------------------------------------*/
/// @brief	Camera Filed of View(FOV)
/*-----------------------------------------------------------------*/
struct CamFov {
	uint16_t	horz;	/*!< Horizontal FOV [degree * 100] */
	uint16_t	vert;	/*!< Vertical FOV [degree * 100] */
};

/*-----------------------------------------------------------------*/
/// @brief	Motion mode information
/*-----------------------------------------------------------------*/
struct ModeInfo {
	uint8_t					id;				/*!< Motion mode ID */
	std::string				description;	/*!< Motion mode description */
	std::vector<ImgOutKind>	img_out;		/*!< Selectable output images */
	Range					dist_range;		/*!< Range of Distance */
	uint16_t				fps;			/*!< Framerate [fps * 100] */
	uint8_t					thin_w;			/*!< Thinning number(Horizontal) */
	uint8_t					thin_h;			/*!< Thinning number(Vertical) */
	Point2d					crop;			/*!< Crop point from Sensor */
	bool					light_times;	/*!< Is enable to change Light times */
	uint16_t				range_calib;	/*!< Range calibration revision */
};

/*-----------------------------------------------------------------*/
/// @brief	List of Motion modes
/*-----------------------------------------------------------------*/
using ModeList = std::vector<ModeInfo>;

/*-----------------------------------------------------------------*/
/// @brief	Light times information
/*-----------------------------------------------------------------*/
struct LightTimesInfo {
	uint32_t	min;	/*!< Minimum of Light times */
	uint32_t	max;	/*!< Maximum of Light times */
	uint32_t	count;	/*!< Current Light times */
};

/*-----------------------------------------------------------------*/
/// @brief	Interval for Auto Exposure information
/*-----------------------------------------------------------------*/
struct AEIntervalInfo {
	uint8_t	min;		/*!< Minimum of Interval */
	uint8_t	max;		/*!< Maximum of Interval */
	uint8_t	interval;	/*!< Current Interval */
};

/*-----------------------------------------------------------------*/
/// @brief	Signal threshold Information (Saturation/Dark)
/*-----------------------------------------------------------------*/
struct SignalThresholdInfo {
	uint16_t	min;		/*!< Minimum of Threshold */
	uint16_t	max;		/*!< Maximum of Threshold */
	uint16_t	threshold;	/*!< Current Threshold */
};

/*-----------------------------------------------------------------*/
/// @brief	Interference Suppression Parameter Information
/*-----------------------------------------------------------------*/
struct IntSuppParamInfo {
	uint8_t	min;			/*!< Minimum of Parameter */
	uint8_t	max;			/*!< Maximum of Parameter */
	uint8_t	value;			/*!< Current Value */
};

/*-----------------------------------------------------------------*/
/// @brief	Interference Suppression Information
/*-----------------------------------------------------------------*/
struct IntSuppInfo {
	IntSuppModeType		mode;		/*!< Interference Suppression Mode */
	IntSuppParamInfo	prm_m;		/*!< Manual Mode Parameter */
	IntSuppParamInfo	prm_a1;		/*!< Automatic Mode Parameter 1 */
	IntSuppParamInfo	prm_a2;		/*!< Automatic Mode Parameter 2 */
	IntSuppParamInfo	prm_a3;		/*!< Automatic Mode Parameter 3 */
};

/*-----------------------------------------------------------------*/
/// @brief	Camera parameters
/*-----------------------------------------------------------------*/
struct CamPrmRequest {
	CamPrmKind				kind;		/*!< Kind of Camera parameter */
	uint16_t				id;			/*!< Parameter ID */
	std::vector<uint8_t>	data;		/*!< Parameter data */
};

/*-----------------------------------------------------------------*/
/// @brief	Register controllable device information
/*-----------------------------------------------------------------*/
struct RegDevInfo {
	RegDevType	target;		/*!< Target device type */
	uint8_t		addr_len;	/*!< length of register address [byte] */
	uint8_t		val_len;	/*!< length of register value [byte] */
	uint8_t		list_len;	/*!< Maximum number of consecutive addresses in CMD_REG_LIST */
};

/*-----------------------------------------------------------------*/
/// @brief	Register controllable devices
/*-----------------------------------------------------------------*/
using RegDevs = std::vector<RegDevInfo>;

/*-----------------------------------------------------------------*/
/// @brief	Register information
/*-----------------------------------------------------------------*/
struct RegInfo {
	RegDevType	target;		/*!< Target device type */
	uint16_t	addr;		/*!< Register address */
	uint16_t	value;		/*!< Register value */
};

/*-----------------------------------------------------------------*/
/// @brief	List of registers
/*-----------------------------------------------------------------*/
struct RegList {
	RegDevType				target;		/*!< Target device type */
	uint16_t				addr;		/*!< Start register address */
	std::vector<uint16_t>	values;		/*!< List of register values */
};

/*-----------------------------------------------------------------*/
/// @brief	USB Peripheral Controller Access key
/*-----------------------------------------------------------------*/
struct UsbPCAccKey {
	uint16_t				key;		/*!< Key value */
};

} // namespace krm
