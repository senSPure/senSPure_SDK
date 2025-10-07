/*------------------------------------------------------------------*/
/// @file		RegCsvList.h
/// @brief		Register list file(csv) class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <string>
#include <filesystem>

#include "PlFw.h"

namespace krm
{

class RegCsvList
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	RegCsvList(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~RegCsvList(void);

	/*------------------------------------------------------------------*/
	/// @brief	Read register
	///	@param	[in]	camera		Camera object
	///	@param	[in]	file_path	CVS file path
	/*------------------------------------------------------------------*/
	Result readRegList(PlFw* camera, const std::filesystem::path& file_path);

	/*------------------------------------------------------------------*/
	/// @brief	Write register
	///	@param	[in]	camera		Camera object
	///	@param	[in]	file_path	CVS file path
	/*------------------------------------------------------------------*/
	Result writeRegList(PlFw* camera, const std::filesystem::path& file_path);

private:
	/*------------------------------------------------------------------*/
	/// @brief	Read csv file
	///	@param	[in]	read_list	List for read
	///	@param	[in]	file_path	CVS file path
	///	@param	[in]	reg_devs	supported devices
	///	@param	[out]	regs		register list
	/*------------------------------------------------------------------*/
	Result readCsv(bool read_list, const std::filesystem::path& file_path,
		const RegDevs& reg_devs, std::vector<RegInfo>& regs);

	/*------------------------------------------------------------------*/
	/// @brief	Write csv file
	///	@param	[in]	file_path	CVS file path
	///	@param	[in]	regs	register list
	/*------------------------------------------------------------------*/
	Result writeCsv(const std::filesystem::path& file_path, const std::vector<RegInfo>& regs);

	/*------------------------------------------------------------------*/
	/// @brief	Pack consecutive register list
	///	@param	[in]	regs		register list
	///	@param	[in]	reg_devs	supported devices
	///	@param	[out]	reg_list	consecutive register list
	/*------------------------------------------------------------------*/
	void packRegList(const std::vector<RegInfo>& regs, const RegDevs& reg_devs,
		std::vector<RegList>& reg_list);

	/*------------------------------------------------------------------*/
	/// @brief	Unpack consecutive register list
	///	@param	[in]	reg_list	consecutive register list
	///	@param	[out]	regs		register list
	/*------------------------------------------------------------------*/
	void unpackRegList(const std::vector<RegList>& reg_list, std::vector<RegInfo>& regs);

	/*------------------------------------------------------------------*/
	/// @brief	Convert file name
	///	@param	[in]	org_path	original file path
	///	@param	[in]	dst_path	converted file path
	/*------------------------------------------------------------------*/
	void getResultName(const std::filesystem::path& org_path, std::filesystem::path& dst_path);

	void delChr(std::string& str, char delc);
	void splitStr(const std::string& istr, char delimiter, std::vector<std::string>& ostr);
};

} // namespace krm
