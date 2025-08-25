/*------------------------------------------------------------------*/
/// @file		SaveSnapshot.cpp
/// @brief		Save snapshot data class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <boost/property_tree/json_parser.hpp>

#include "SaveSnapshot.h"
#include "CommonLog.h"
#include "CameraType.h"

namespace krm
{

SaveSnapshot::SaveSnapshot(void) :
	dir_name_(), stream_size_(0),
	depth_ave_size_(0), depth_std_size_(0), ir_ave_size_(0), ir_std_size_(0),
	raw_ave_size_(0), raw_std_size_(0), pcd_size_(0)
{
	stream_buffer_.clear();
}

SaveSnapshot::~SaveSnapshot(void)
{
	stream_buffer_.clear();
}

Result SaveSnapshot::save(const std::filesystem::path& path, const SnapshotParams& params,
				const ImageFormats& img_fmts, DrawImages& draws, PlFw* plfw)
{
	Result	ret = init(path, params, img_fmts);
	if (ret != SUCCESS) {
		LOG_ERR("Invalid argument : %d\n", ret);
		return ret;
	}

	// put record information
	ret = saveInfoJson(params, img_fmts, plfw);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to save json : %d\n", ret);
		return ret;
	}

	if (img_fmts[IMG_DEPTH].pixels > 0) {
		if (params.file_fmt == 0) {		// binary file
			ret = saveBin(img_fmts[IMG_DEPTH], draws[IMG_DEPTH], IMG_DEPTH, (params.took_frms > 1U));
		} else {						// csv file
			ret = saveCsv(img_fmts[IMG_DEPTH], draws[IMG_DEPTH], IMG_DEPTH, (params.took_frms > 1U));
		}
		if (ret != SUCCESS) {
			LOG_ERR("Failed to save Depth : ret=%d\n", ret);
			return ret;
		}
		ret = savePcd(std::dynamic_pointer_cast<DrawPcd>(draws[IMG_PCD]));
		if (ret != SUCCESS) {
			LOG_ERR("Failed to save Point Cloud : ret=%d\n", ret);
			return ret;
		}
	}

	if (img_fmts[IMG_IR].pixels > 0) {
		if (params.file_fmt == 0) {		// binary file
			ret = saveBin(img_fmts[IMG_IR], draws[IMG_IR], IMG_IR, (params.took_frms > 1U));
		} else {						// csv file
			ret = saveCsv(img_fmts[IMG_IR], draws[IMG_IR], IMG_IR, (params.took_frms > 1U));
		}
		if (ret != SUCCESS) {
			LOG_ERR("Failed to save IR : ret=%d\n", ret);
			return ret;
		}
	}

	for (uint8_t i = IMG_RAW1; i <= IMG_RAW4; i++) {
		if (img_fmts[i].pixels > 0) {
			if (params.file_fmt == 0) {		// binary file
				ret = saveBin(img_fmts[i], draws[i], static_cast<ImageKind>(i), (params.took_frms > 1U));
			} else {						// csv file
				ret = saveCsv(img_fmts[i], draws[i], static_cast<ImageKind>(i), (params.took_frms > 1U));
			}
			if (ret != SUCCESS) {
				LOG_ERR("Failed to save IR : ret=%d\n", ret);
				return ret;
			}
		}
	}

	return SUCCESS;
}

Result SaveSnapshot::init(const std::filesystem::path& path, const SnapshotParams& params, const ImageFormats& img_fmts)
{
	Result	ret = SUCCESS;

	// parameter check
	ret = checkFilePath(path);
	if (ret != SUCCESS) {
		LOG_ERR("checkFilePath error : ret=%d\n", ret);
		return ret;
	}

	// disk capacity check
	ret = checkDiskCapacity(path, img_fmts, (params.file_fmt == 0));
	if (ret != SUCCESS) {
		LOG_ERR("checkDispCapacity error : ret=%d\n", ret);
		return ret;
	}

	// create directory
	ret = createDirectory(path);
	if (ret != SUCCESS) {
		LOG_ERR("createDirectory error : ret=%d\n", ret);
	}

	return ret;
}

Result SaveSnapshot::openFile(const std::filesystem::path& path, std::streamsize size, bool file_fmt, std::ofstream& img_file)
{
	stream_buffer_.resize(size);
	img_file.rdbuf()->pubsetbuf(stream_buffer_.data(), size);
	if (file_fmt) {
		img_file.open(path.c_str(), (std::ios::out | std::ios::binary | std::ios::trunc));
	} else {
		img_file.open(path.c_str(), (std::ios::out | std::ios::trunc));
	}
	if (img_file.bad()) {
		LOG_ERR("Failed to open : %s\n", path.string<char>().c_str());
		return ERR_SYSTEM;
	}

	return SUCCESS;
}

Result SaveSnapshot::saveBin(const ImageFormat& fmt, const std::shared_ptr<DrawImage>& draw, ImageKind kind, bool en_std)
{
	std::filesystem::path file_path = dir_name_;
	std::ofstream img_file;
	Result ret;

	std::string file_ave_name;
	std::string file_std_name;
	std::streamsize ave_size;
	std::streamsize std_size;

	switch (kind) {
	case IMG_DEPTH:
		file_ave_name = static_cast<std::string>(D_AVE_FILE);
		file_std_name = static_cast<std::string>(D_STD_FILE);
		ave_size = depth_ave_size_;
		std_size = depth_std_size_;
		break;
	case IMG_IR:
		file_ave_name = static_cast<std::string>(I_AVE_FILE);
		file_std_name = static_cast<std::string>(I_STD_FILE);
		ave_size = ir_ave_size_;
		std_size = ir_std_size_;
		break;
	case IMG_RAW1:
	case IMG_RAW2:
	case IMG_RAW3:
	case IMG_RAW4:
		file_ave_name = static_cast<std::string>(R_AVE_FILE) + std::to_string(kind - IMG_RAW1 + 1U);
		file_std_name = static_cast<std::string>(R_STD_FILE) + std::to_string(kind - IMG_RAW1 + 1U);
		ave_size = raw_ave_size_;
		std_size = raw_std_size_;
		break;
	default:
		return ERR_BAD_ARG;
	}

	// Ave
	file_path.append(file_ave_name.data());
	file_path += BIN_EXT.data();

	ret = openFile(file_path, ave_size, true, img_file);	// file open
	if (ret != SUCCESS) {
		LOG_ERR("Failed to open(Ave)\n");
		return ret;
	}

	draw->lock();
	img_file.write(reinterpret_cast<const char*>(draw->getSnapshotImage()), static_cast<std::streamsize>(fmt.size));
	draw->unlock();

	if (img_file.bad()) {
		LOG_ERR("Failed to write(Ave)\n");
		ret = ERR_SYSTEM;
	}
	img_file.close();

	// Std
	if (en_std) {
		file_path = dir_name_;
		file_path.append(file_std_name.data());
		file_path += BIN_EXT.data();

		ret = openFile(file_path, std_size, true, img_file);	// file open
		if (ret != SUCCESS) {
			LOG_ERR("Failed to open(Std)\n");
			return ret;
		}

		draw->lock();
		img_file.write(reinterpret_cast<const char*>(draw->getSnapshotStd()), static_cast<std::streamsize>(fmt.size));
		draw->unlock();

		if (img_file.bad()) {
			LOG_ERR("Failed to write(Std)\n");
			ret = ERR_SYSTEM;
		}
		img_file.close();
	}

	return ret;
}

Result SaveSnapshot::saveCsv(const ImageFormat& fmt, const std::shared_ptr<DrawImage>& draw, ImageKind kind, bool en_std)
{
	uint16_t i, j;
	std::filesystem::path file_path = dir_name_;
	std::ofstream img_file;
	uint16_t* val;
	Result ret;

	std::string file_ave_name;
	std::string file_std_name;
	std::streamsize ave_size;
	std::streamsize std_size;

	switch (kind) {
	case IMG_DEPTH:
		file_ave_name = static_cast<std::string>(D_AVE_FILE);
		file_std_name = static_cast<std::string>(D_STD_FILE);
		ave_size = depth_ave_size_;
		std_size = depth_std_size_;
		break;
	case IMG_IR:
		file_ave_name = static_cast<std::string>(I_AVE_FILE);
		file_std_name = static_cast<std::string>(I_STD_FILE);
		ave_size = ir_ave_size_;
		std_size = ir_std_size_;
		break;
	case IMG_RAW1:
	case IMG_RAW2:
	case IMG_RAW3:
	case IMG_RAW4:
		file_ave_name = static_cast<std::string>(R_AVE_FILE) + std::to_string(kind - IMG_RAW1 + 1U);
		file_std_name = static_cast<std::string>(R_STD_FILE) + std::to_string(kind - IMG_RAW1 + 1U);
		ave_size = raw_ave_size_;
		std_size = raw_std_size_;
		break;
	default:
		return ERR_BAD_ARG;
	}

	// Ave
	file_path.append(file_ave_name.data());
	file_path += CSV_EXT.data();
	ret = openFile(file_path, ave_size, true, img_file);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to open(Ave)\n");
		return ret;
	}

	draw->lock();
	val = draw->getSnapshotImage();
	for (i = 0; i < fmt.height; i++) {
		for (j = 0; j < fmt.width; j++) {
			img_file << *(val++) << ",";
		}
		img_file << std::endl;
	}
	draw->unlock();

	if (img_file.bad()) {
		LOG_ERR("Failed to write(Ave)\n");
		ret = ERR_SYSTEM;
	}
	img_file.close();

	// Std
	if (en_std) {
		file_path = dir_name_;
		file_path.append(file_std_name.data());
		file_path += CSV_EXT.data();

		ret = openFile(file_path, std_size, true, img_file);
		if (ret != SUCCESS) {
			LOG_ERR("Failed to open(Std)\n");
			return ret;
		}

		draw->lock();
		val = draw->getSnapshotStd();
		for (i = 0; i < fmt.height; i++) {
			for (j = 0; j < fmt.width; j++) {
				img_file << *(val++) << ",";
			}
			img_file << std::endl;
		}
		draw->unlock();

		if (img_file.bad()) {
			LOG_ERR("Failed to write(Std)\n");
			ret = ERR_SYSTEM;
		}
		img_file.close();
	}

	return SUCCESS;
}

Result SaveSnapshot::savePcd(const std::shared_ptr<DrawPcd>& draw)
{
	std::filesystem::path file_path = dir_name_;
	std::ofstream img_file;
	const std::vector<Point3d>* pcd_data;
	RgbaColor rgba;
	Result ret;

	file_path.append(PCD_FILE.data());
	ret = openFile(file_path, pcd_size_, false, img_file);
	if (ret != SUCCESS) {
		LOG_ERR("Failed to open Point Cloud\n");
		return ERR_SYSTEM;
	}

	draw->lock();
	pcd_data = draw->getDrawImage();
	// write header
	img_file << "ply" << std::endl;
	img_file << "format ascii 1.0" << std::endl;
	img_file << "element vertex " << pcd_data->size() << std::endl;
	img_file << "property float x" << std::endl;
	img_file << "property float y" << std::endl;
	img_file << "property float z" << std::endl;
	img_file << "property uchar red" << std::endl;
	img_file << "property uchar green" << std::endl;
	img_file << "property uchar blue" << std::endl;
	img_file << "end_header" << std::endl;

	// write body
	for (auto pcd : *pcd_data) {
		std::memcpy(rgba.data(), &pcd.color, sizeof(pcd.color));
		img_file << std::fixed << std::setprecision(5) << pcd.x << " "
				 << std::fixed << std::setprecision(5) << pcd.y << " "
				 << std::fixed << std::setprecision(5) << -pcd.z << " "
				 << std::to_string(rgba[0]) << " " << std::to_string(rgba[1]) << " " << std::to_string(rgba[2]) << std::endl;
	}
	draw->unlock();

	if (img_file.bad()) {
		LOG_ERR("Failed to write Point Cloud\n");
		ret = ERR_SYSTEM;
	}

	img_file.close();

	return ret;
}


Result SaveSnapshot::saveInfoJson(const SnapshotParams& params, const ImageFormats& img_fmts, PlFw* plfw)
{
	Result ret = SUCCESS;
	DeviceInfo	device_info;
	LensInfo	lens_info;
	CamFov		fov;
	uint8_t		mode = 0;
	uint16_t	mode_idx = UINT16_MAX;
	ModeList	mode_list;
	boost::property_tree::ptree pt;
	boost::property_tree::ptree child;
	boost::property_tree::ptree grandchild;
	boost::property_tree::ptree children;
	boost::property_tree::ptree grandchildren;
	boost::property_tree::ptree pt_child_d;
	boost::property_tree::ptree offset_children;
	boost::property_tree::ptree offset_child1, offset_child2, offset_child3;
	boost::property_tree::ptree rotation_children;
	boost::property_tree::ptree rotation_child1, rotation_child2, rotation_child3;
	std::filesystem::path	path = dir_name_;

	if (plfw == nullptr) {
		LOG_ERR("PlFw object is null\n");
		return ERR_BAD_ARG;
	}

	ret = plfw->getCamProperty(CMD_DEV_INFO, &device_info);
	if (ret != SUCCESS) {
		LOG_ERR("CMD_DEV_INFO error : %d\n", ret);
		return ret;
	}

	ret = plfw->getCamProperty(CMD_LENS_INFO, &lens_info);
	if (ret != SUCCESS) {
		LOG_ERR("CMD_LENS_INFO error : %d\n", ret);
		return ret;
	}

	ret = plfw->getCamProperty(CMD_FOV, &fov);
	if (ret != SUCCESS) {
		LOG_ERR("CMD_FOV error : %d\n", ret);
		return ret;
	}

	ret = plfw->getCamProperty(CMD_MODE, &mode);
	if (ret != SUCCESS) {
		LOG_ERR("CMD_MODE error : %d\n", ret);
		return ret;
	}

	ret = plfw->getCamProperty(CMD_MODE_LIST, &mode_list);
	if (ret != SUCCESS) {
		LOG_ERR("CMD_MODE_LIST error : %d\n", ret);
		return ret;
	}

	for (uint16_t i = 0; i < static_cast<uint16_t>(mode_list.size()); i++) {
		if (mode_list[i].id == mode) {
			mode_idx = i;
			break;
		}
	}
	if (mode_idx == UINT16_MAX) {
		LOG_ERR("Invalid mode id : %u\n", mode);
		return ERR_SYSTEM;
	}

	// create json tree
	pt.clear();

	pt.put("Snapshot.frame_count",		params.took_frms);

	pt.put("Device.hw_kind",			device_info.hw_kind);
	pt.put("Device.serial_no",			device_info.serial_no);
	pt.put("Device.adjust_no",			device_info.adjust_no);
	pt.put("Device.firm_ver.major",		device_info.firm_ver.major);
	pt.put("Device.firm_ver.minor",		device_info.firm_ver.minor);
	pt.put("Device.firm_ver.revision",	device_info.firm_ver.rev);
	pt.put("Device.ld_wave",			device_info.ld_wave);
	pt.put("Device.ld_enable",			device_info.ld_enable);
	pt.put("Device.correct_calib",		device_info.correct_calib);

	pt.put("Lens.cam_dist",				lens_info.cam_dist ? 1U : 0);
	pt.put("Lens.lens_calib",			lens_info.lens_calib);

	pt.put("Fov.horz",					fov.horz);
	pt.put("Fov.vert",					fov.vert);

	children.clear();
	pt.put("Mode.id",					mode);
	pt.put("Mode.description",			mode_list[mode_idx].description);

	for (uint8_t i = IMG_DEPTH; i <= IMG_RAW4; i++) {
		child.clear();
		grandchild.clear();
		grandchildren.clear();

		child.put("kind",		IMG_STR[i]);
		child.put("width",		img_fmts[i].width);
		child.put("height",		img_fmts[i].height);
		grandchild.put("",		img_fmts[i].active_start.x);
		grandchildren.push_back(std::make_pair("", grandchild));
		grandchild.put("",		img_fmts[i].active_start.y);
		grandchildren.push_back(std::make_pair("", grandchild));
		child.add_child("active_start", grandchildren);
		child.put("active_w",	img_fmts[i].active_w);
		child.put("active_h",	img_fmts[i].active_h);
		child.put("bpp",		img_fmts[i].bpp);
		children.push_back(std::make_pair("", child));
	}
	pt.add_child("Mode.Images", children);

	pt.put("Mode.range.min",		mode_list[mode_idx].dist_range.min);
	pt.put("Mode.range.max",		mode_list[mode_idx].dist_range.max);
	pt.put("Mode.fps",				mode_list[mode_idx].fps);
	pt.put("Mode.range_calib",		mode_list[mode_idx].range_calib);

	pt.put("PostFilt.med_filt", params.medf);
	pt.put("PostFilt.bil_filt", params.bilf);
	pt.put("PostFilt.fly_p_filt",	params.flypf);

	pt.put("PostProcess.Distortion",	params.dist);

	pt_child_d.put("origin",			params.pcd_type);
	offset_child1.put("", params.pcd_ofst[0]);
	offset_child2.put("", params.pcd_ofst[1]);
	offset_child3.put("", params.pcd_ofst[2]);
	offset_children.push_back(std::make_pair("", offset_child1));
	offset_children.push_back(std::make_pair("", offset_child2));
	offset_children.push_back(std::make_pair("", offset_child3));
	pt_child_d.add_child("offset", offset_children);
	rotation_child1.put("", params.pcd_rot[0]);
	rotation_child2.put("", params.pcd_rot[1]);
	rotation_child3.put("", params.pcd_rot[2]);
	rotation_children.push_back(std::make_pair("", rotation_child1));
	rotation_children.push_back(std::make_pair("", rotation_child2));
	rotation_children.push_back(std::make_pair("", rotation_child3));
	pt_child_d.add_child("rotation", rotation_children);
	pt.add_child("PostProcess.PointCloud", pt_child_d);

	path.append(INFO_FILE);
	try {
		boost::property_tree::write_json(path.string<char>().c_str(), pt);
	} catch (...) {
		LOG_ERR("Failed to save %s\n", path.string<char>().c_str());
		return ERR_SYSTEM;
	}

	return SUCCESS;
}

Result SaveSnapshot::checkFilePath(const std::filesystem::path& path)
{
	std::filesystem::file_status status = std::filesystem::status(path);
	if (status.type() == std::filesystem::file_type::not_found) {
		LOG_ERR("%s is not exists\n", path.string<char>().c_str());
		return ERR_NOT_EXIST;
	}
	if (status.type() != std::filesystem::file_type::directory) {
		LOG_ERR("%s is not directory\n", path.string<char>().c_str());
		return ERR_NOT_EXIST;
	}

	return SUCCESS;
}

Result SaveSnapshot::checkDiskCapacity(const std::filesystem::path& path, const ImageFormats& img_fmts, bool file_fmt)
{
	uintmax_t					sum_size = 0;
	std::filesystem::space_info	diskInfo = std::filesystem::space(path);

	if (img_fmts[IMG_DEPTH].pixels > 0) {
		if (file_fmt) {					// binary file
			depth_ave_size_ = depth_std_size_ = static_cast<std::streamsize>(img_fmts[IMG_DEPTH].size);
		} else {						// csv file
			std::streamsize line_num = static_cast<std::streamsize>(img_fmts[IMG_DEPTH].height) * 2U;
			depth_ave_size_ = static_cast<std::streamsize>(img_fmts[IMG_DEPTH].pixels) * CSV_AVE_SIZE;
			depth_std_size_ = static_cast<std::streamsize>(img_fmts[IMG_DEPTH].pixels) * CSV_STD_SIZE;
			depth_ave_size_ += line_num;
			depth_std_size_ += line_num;
		}
		sum_size += static_cast<uintmax_t>(depth_ave_size_ + depth_std_size_);
		pcd_size_ = static_cast<std::streamsize>(((PLY_PXL_XYZ_SIZE + PLY_PXL_RGB_SIZE) * 3U * img_fmts[IMG_DEPTH].pixels) + img_fmts[IMG_DEPTH].pixels + PLY_HEADER_SIZE);
		sum_size += static_cast<uintmax_t>(pcd_size_);
	}

	if (img_fmts[IMG_IR].pixels > 0) {
		if (file_fmt) {					// binary file
			ir_ave_size_ = ir_std_size_ = static_cast<std::streamsize>(img_fmts[IMG_IR].size);
		} else {						// csv file
			std::streamsize line_num = static_cast<std::streamsize>(img_fmts[IMG_IR].height) * 2U;
			ir_ave_size_ = static_cast<uintmax_t>(img_fmts[IMG_IR].pixels) * CSV_AVE_SIZE;
			ir_std_size_ = static_cast<uintmax_t>(img_fmts[IMG_IR].pixels) * CSV_STD_SIZE;
			ir_ave_size_ += line_num;
			ir_std_size_ += line_num;
		}
		sum_size += static_cast<uintmax_t>(ir_ave_size_);
	}

	if (img_fmts[IMG_RAW1].pixels > 0) {
		if (file_fmt) {					// binary file
			raw_ave_size_ = raw_std_size_ = static_cast<std::streamsize>(img_fmts[IMG_RAW1].size);
		} else {
			std::streamsize line_num = static_cast<std::streamsize>(img_fmts[IMG_RAW1].height) * 2U;
			raw_ave_size_ = static_cast<uintmax_t>(img_fmts[IMG_RAW1].pixels) * CSV_AVE_SIZE;
			raw_std_size_ = static_cast<uintmax_t>(img_fmts[IMG_RAW1].pixels) * CSV_STD_SIZE;
			raw_ave_size_ += line_num;
			raw_std_size_ += line_num;
		}
		sum_size += static_cast<uintmax_t>(raw_ave_size_ * (IMG_RAW4 - IMG_RAW1 + 1U));
	}
	stream_size_ = static_cast<std::streamsize>(sum_size);

	if (diskInfo.available < sum_size) {
		LOG_ERR("insufficient disk space : available = %llu sum = %llu\n",
			static_cast<unsigned long long>(diskInfo.available), static_cast<unsigned long long>(sum_size));
		return ERR_FULL;
	}
	if ((diskInfo.available - sum_size) < LIMIT_SIZE) {
		LOG_ERR("disk space is under limit size : available = %llu sum = %llu\n",
			static_cast<unsigned long long>(diskInfo.available), static_cast<unsigned long long>(sum_size));
		return ERR_FULL;
	}

	return SUCCESS;
}

Result SaveSnapshot::createDirectory(const std::filesystem::path& path)
{
	char			save_dir[DIR_NAME_LEN] = {0};
	std::error_code	ret;
	std::time_t 	t = std::time(nullptr);
	std::tm			now;

#ifdef LNX_FUNC
	(void)localtime_r(&t, &now);
#else	// LNX_FUNC
#ifdef WIN_FUNC
	(void)localtime_s(&now, &t);
#endif	// WIN_FUNC
#endif	// LNX_FUNC

	// make directory name
	(void)strftime(save_dir, sizeof(save_dir), "Snapshot_%Y%m%d_%H%M%S", &now);

	dir_name_ = path;
	dir_name_.append(save_dir);

	// already exist or not
	if (std::filesystem::exists(dir_name_)) {
		LOG_ERR("%s is already exists\n", dir_name_.string<char>().c_str());
		return ERR_BAD_ARG;
	}

	// create a directory
	if (!std::filesystem::create_directory(dir_name_, ret)) {
		LOG_ERR("create_directory(%s) failed\n", dir_name_.string<char>().c_str());
		return ERR_SYSTEM;
	}

	return SUCCESS;
}

} // namespace krm
