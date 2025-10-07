/*------------------------------------------------------------------*/
/// @file		SaveSnapshot.h
/// @brief		Save snapshot data class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <string>
#include <filesystem>
#include <iostream>
#include <iomanip>

#include "PlFw.h"
#include "DrawThread.h"
#include "DrawImage.h"
#include "DrawDepth.h"
#include "DrawPcd.h"

namespace krm
{

/*-----------------------------------------------------------------*/
/// @brief	Saving Snapshot Parameter
/*-----------------------------------------------------------------*/
struct SnapshotParams {
	int						file_fmt;			/*!< save file format type */
	uint8_t					took_frms;			/*!< number of frames taken snapshot */
	bool					medf;				/*!< enable/disable median_filter for information */
	bool					bilf;
	bool					flypf;				/*!< enable/disable flying pixel for information */
	bool					dist;				/*!< enable/disable distortion correction for information */
	int						pcd_type;			/*!< point cloud conversion type */
	std::array<int16_t, 3U>	pcd_ofst;			/*!< offset value for point cloud conversion */
	std::array<float, 3U>	pcd_rot;			/*!< rotation for point cloud conversion */
};

class SaveSnapshot
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	SaveSnapshot(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~SaveSnapshot(void);

	/*------------------------------------------------------------------*/
	/// @brief	Save snapshot files
	///	@param	[in]	path		save target directory
	///	@param	[in]	params		save parameters
	///	@param	[in]	img_fmts	image formats
	///	@param	[in]	draws		snapshot data classes
	///	@param	[in]	pflw		Pipeline Framework object
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Invalid argument
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_FULL		Full (Buffer, Storage etc.)
	/// @retval	ERR_SYSTEM		Other errors (System error)
	/*------------------------------------------------------------------*/
	Result save(const std::filesystem::path& path, const SnapshotParams& params,
				const ImageFormats& img_fmts, DrawImages& draws, PlFw* plfw);

private:
	static const std::streamsize	CSV_AVE_SIZE = 6U;		// max digits(5) + comma(1)
	static const std::streamsize	CSV_STD_SIZE = 9U;		// max digits(5) + decimal point(1) + max decimal digits(2) + comma(1)
	static const std::streamsize	PLY_HEADER_SIZE = 179U;	// header size in ply file
	static const std::streamsize	PLY_PXL_XYZ_SIZE = 47U;	// sign(1) + max digits(39) + decimal point(1) + decimal digits(5) + space(1)
	static const std::streamsize	PLY_PXL_RGB_SIZE = 4U;	// max digits(3) + space(1)
	static const uint32_t		DIR_NAME_LEN = 32U;			// "Snapshot_YYYYMMDD_HHMMSS" + some padding
	static const uintmax_t		LIMIT_SIZE = 1000000000;	// 1000 * 1000 * 1000 = 1GB
	static constexpr std::array<std::string_view, IMG_KINDS> IMG_STR = {
		"Depth",	/* IMG_DEPTH */
		"IR",		/* IMG_IR */
		"Raw G1",	/* IMG_RAW1 */
		"Raw G2",	/* IMG_RAW2 */
		"Raw G3",	/* IMG_RAW3 */
		"Raw G4",	/* IMG_RAW4 */
	};
	static constexpr std::string_view	INFO_FILE  = "SnapshotInfo.json";
	static constexpr std::string_view	D_AVE_FILE = "DepthImg";
	static constexpr std::string_view	D_STD_FILE = "DepthStd";
	static constexpr std::string_view	I_AVE_FILE = "IrImg";
	static constexpr std::string_view	I_STD_FILE = "IrStd";
	static constexpr std::string_view	R_AVE_FILE = "RawImg";
	static constexpr std::string_view	R_STD_FILE = "RawStd";
	static constexpr std::string_view	PCD_FILE   = "PointCloud.ply";
	static constexpr std::string_view	BIN_EXT    = ".raw";
	static constexpr std::string_view	CSV_EXT    = ".csv";

	std::filesystem::path	dir_name_;
	std::streamsize			stream_size_;
	std::vector<char>		stream_buffer_;

	std::streamsize			depth_ave_size_;
	std::streamsize			depth_std_size_;
	std::streamsize			ir_ave_size_;
	std::streamsize			ir_std_size_;
	std::streamsize			raw_ave_size_;
	std::streamsize			raw_std_size_;
	std::streamsize			pcd_size_;

	Result init(const std::filesystem::path& path, const SnapshotParams& params, const ImageFormats& img_fmts);
	Result openFile(const std::filesystem::path& path, std::streamsize size, bool file_fmt, std::ofstream& img_file);
	Result saveBin(const ImageFormat& fmt, const std::shared_ptr<DrawImage>& draw, ImageKind kind, bool en_std);
	Result saveCsv(const ImageFormat& fmt, const std::shared_ptr<DrawImage>& draw, ImageKind kind, bool en_std);
	Result savePcd(const std::shared_ptr<DrawPcd>& draw);
	Result saveInfoJson(const SnapshotParams& params, const ImageFormats& img_fmts, PlFw* plfw);
	Result checkFilePath(const std::filesystem::path& path);
	Result checkDiskCapacity(const std::filesystem::path& path, const ImageFormats& img_fmt, bool file_fmt);
	Result createDirectory(const std::filesystem::path& path);
};

} // namespace krm
