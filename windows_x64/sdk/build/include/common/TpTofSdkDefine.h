/*------------------------------------------------------------------*/
/// @file		TpTofSdkDefine.h
/// @brief		SDK common definitions
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <cstdint>
#include <array>
#include <vector>
#include <climits>
#include <ctime>

namespace krm
{
/*-----------------------------------------------------------------*/
/// @brief	Result of function
/*-----------------------------------------------------------------*/
enum Result {
	SUCCESS = 0,		/*!< Success */
	CANCELED,			/*!< Canceled wait state */
	REACH_EOF,			/*!< Reached end of file */
	ERR_INVALID_PTR,	/*!< Invalid pointer */
	ERR_OVER_RANGE,		/*!< Over the range */
	ERR_BAD_ARG,		/*!< Invalid argument */
	ERR_BAD_STATE,		/*!< Bad state transmission */
	ERR_NOT_EXIST,		/*!< Not exist */
	ERR_TIMEOUT,		/*!< Timeout from waiting */
	ERR_EMPTY,			/*!< Empty (Buffer, File etc.) */
	ERR_FULL,			/*!< Full (Buffer, Storage etc.) */
	ERR_NOT_SUPPORT,	/*!< Not supported */
	ERR_SYSTEM			/*!< Other errors (System error) */
};

/*-----------------------------------------------------------------*/
/// @brief	Kind of Image
/*-----------------------------------------------------------------*/
enum ImageKind : uint8_t {
	IMG_DEPTH = 0,	/*!< Depth */
	IMG_IR,			/*!< IR */
	IMG_RAW1,		/*!< Sensor RAW1 */
	IMG_RAW2,		/*!< Sensor RAW2 */
	IMG_RAW3,		/*!< Sensor RAW3 */
	IMG_RAW4,		/*!< Sensor RAW4 */
	IMG_KINDS		/*!< Number of image kinds */
};

/*-----------------------------------------------------------------*/
/// @brief	Kind of Point Cloud
/*-----------------------------------------------------------------*/
enum PcdKind {
	PCD_XYZ = 0,	/*!< X,Y,Z */
	PCD_RGBXYZ,		/*!< X,Y,Z + RGB */
	PCD_IRXYZ,		/*!< X,Y,Z + IR */
};

/*-----------------------------------------------------------------*/
/// @brief	Version information
/*-----------------------------------------------------------------*/
struct Version {
	uint8_t		major;	/*!< Major version */
	uint8_t		minor;	/*!< Minor version */
	uint16_t	rev;	/*!< Revision */
};

/*-----------------------------------------------------------------*/
/// @brief	Pixel position on the image
/*-----------------------------------------------------------------*/
struct Point2d {
	uint16_t	x;		/*!< X-axis coordinate [pixel] */
	uint16_t	y;		/*!< Y-axis coordinate [pixel] */
};

/*-----------------------------------------------------------------*/
/// @brief	Coordinates position in 3D space
/*-----------------------------------------------------------------*/
struct Point3d {
	uint32_t	color;	/*!< valid : color data / invalid : UINT32_MAX */
	float		x;		/*!< X-axis coordinate [mm] */
	float		y;		/*!< Y-axis coordinate [mm] */
	float		z;		/*!< Z-axis coordinate [mm] */
};

/*-----------------------------------------------------------------*/
/// @brief	Format of Image data
/*-----------------------------------------------------------------*/
struct ImageFormat {
	uint16_t	width;			/*!< Image width [pixel] */
	uint16_t	height;			/*!< Image height [pixel] */
	Point2d		active_start;	/*!< Active area start point */
	uint16_t	active_w;		/*!< Active area width [pixel] */
	uint16_t	active_h;		/*!< Active area height [pixel] */
	uint32_t	pixels;			/*!< Number of pixels */
	uint8_t		bpp;			/*!< Byte per pixel */
	size_t		size;			/*!< Image data size [byte] */
	/*------------------------------------------------------------------*/
	/// @brief	Set image format
	/// @param	[in]	w			Image width [pixel]
	/// @param	[in]	h			Image height [pixel]
	/// @param	[in]	b			Byte per pixel
	/*------------------------------------------------------------------*/
	inline void set(uint16_t w = 0, uint16_t h = 0, uint8_t b = 0)
	{
		width = w;
		height = h;
		pixels = static_cast<uint32_t>(width) * static_cast<uint32_t>(height);
		bpp = b;
		size = static_cast<size_t>(pixels) * static_cast<size_t>(bpp);
		active_start = {0, 0};
		active_w = w;
		active_h = h;
	}
	/*------------------------------------------------------------------*/
	/// @brief	Image exists?
	/// @retval	true	exist
	/// @retval	false	not exist
	/*------------------------------------------------------------------*/
	inline bool isExist(void)
	{
		return (size > 0);
	}
};

/*-----------------------------------------------------------------*/
/// @brief	Image formats of each image kinds
/*-----------------------------------------------------------------*/
using ImageFormats = std::array<ImageFormat, IMG_KINDS>;

/*-----------------------------------------------------------------*/
/// @brief	Range of distance
/*-----------------------------------------------------------------*/
struct Range {
	uint16_t	min;	/*!< Closest distance [mm] */
	uint16_t	max;	/*!< Farthest distance [mm] */
};

/*-----------------------------------------------------------------*/
/// @brief	Frame error information
/*-----------------------------------------------------------------*/
struct FrameError {
	uint16_t		drop     : 1;	/*!< dropped frame */
	uint16_t		crc      : 1;	/*!< CRC error */
	uint16_t		reserved : 14;	/*!< reserved */
};

/*-----------------------------------------------------------------*/
/// @brief	Conversion state information
/*-----------------------------------------------------------------*/
struct ConvState {
	uint8_t		is_crct_dist  : 1;	/*!< Whether the image is distortion corrected */
	uint8_t		is_filt_med   : 1;	/*!< Whether the image is a median filter applied */
	uint8_t		is_filt_bil   : 1;	/*!< Whether the image is a bilateral filter applied */
	uint8_t		is_filt_fly_p : 1;	/*!< Whether the image is a flying pixel filter applied */
	uint8_t		reserved      : 4;	/*!< reserved */
};

/*-----------------------------------------------------------------*/
/// @brief	Frame information
/*-----------------------------------------------------------------*/
struct FrameInfo {
	uint32_t		number;			/*!< frame number */
	timespec		time;			/*!< time stamp */
	FrameError		frm_err;		/*!< frame error */
	uint16_t		temperature;	/*!< temperature [degree Celsius] / integer:10bit,decimal:6bit */
	uint32_t		light_cnt;		/*!< Current Light Times */
	ConvState		conv_stat;		/*!< Conversion state */
};

/*-----------------------------------------------------------------*/
/// @brief	Image data
/*-----------------------------------------------------------------*/
struct ImageData {
	FrameInfo				info;	/*!< Frame Information */
	std::vector<uint16_t>	data;	/*!< Image data */
	/*------------------------------------------------------------------*/
	/// @brief	Resize image data
	/// @param	[in]	format	image format
	/*------------------------------------------------------------------*/
	inline void resize(const ImageFormat& format)
	{
		data.resize(format.pixels);
		data.shrink_to_fit();
		std::fill(data.begin(), data.end(), 0);
	}
};

/*-----------------------------------------------------------------*/
/// @brief	Image data of each image kinds
/*-----------------------------------------------------------------*/
using Images = std::array<ImageData, IMG_KINDS>;

/*-----------------------------------------------------------------*/
/// @brief	Point Cloud data
/*-----------------------------------------------------------------*/
struct PcdData {
	FrameInfo				info;	/*!< Frame Information */
	PcdKind					kind;	/*!< Kind of Point Cloud */
	std::vector<Point3d>	data;	/*!< Point Cloud data */
	/*------------------------------------------------------------------*/
	/// @brief	Resize Point Cloud data
	/// @param	[in]	format	image format
	/*------------------------------------------------------------------*/
	inline void resize(const ImageFormat& format)
	{
		Point3d inval_data = {UINT32_MAX, 0, 0, 0};
		data.resize(format.pixels);
		data.shrink_to_fit();
		std::fill(data.begin(), data.end(), inval_data);
	}
};

/*-----------------------------------------------------------------*/
/// @brief	Frame data
/*-----------------------------------------------------------------*/
struct Frame {
	Images		images;		/*!< Image data of each image kinds */
	PcdData		pcd;		/*!< Point Cloud data */
};


/*-----------------------------------------------------------------*/
/// @brief	SDK Version
/*-----------------------------------------------------------------*/
const Version SDK_VERSION = {3, 0, 6};	/* Ver 3.0.6 */

/*-----------------------------------------------------------------*/
/// @brief	Invalid Depth value(Far)
/*-----------------------------------------------------------------*/
const uint16_t INVALID_DEPTH = 0xFFFFU;

/*-----------------------------------------------------------------*/
/// @brief	Invalid Depth value(Near)
/*-----------------------------------------------------------------*/
const uint16_t SATURATION_DEPTH = 0x0000U;


} // namespace krm
