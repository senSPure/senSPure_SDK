/*------------------------------------------------------------------*/
/// @file		RegCsvList.cpp
/// @brief		Register list file(csv) class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <fstream>
#include <sstream>

#include "RegCsvList.h"
#include "CommonLog.h"

namespace krm
{

RegCsvList::RegCsvList(void)
{

}

RegCsvList::~RegCsvList(void)
{

}

Result RegCsvList::readRegList(PlFw* camera, const std::filesystem::path& file_path)
{
	Result					ret;
	RegDevs					devs;
	std::vector<RegInfo>	regs;
	std::vector<RegList>	reg_list;
	std::filesystem::path	out_path;

	ret = camera->getCamProperty(CMD_REG_DEVS, &devs);
	if (ret == SUCCESS) {
		ret = readCsv(true, file_path, devs, regs);
		if (ret == SUCCESS) {
			packRegList(regs, devs, reg_list);
			for (auto& list : reg_list) {
				ret = camera->getCamProperty(CMD_REG_LIST, &list);
				if (ret != SUCCESS) {
					LOG_ERR("Failed to Read REG_LIST : %d\n", ret);
					break;
				}
			}
			if (ret == SUCCESS) {
				unpackRegList(reg_list, regs);
				getResultName(file_path, out_path);
				ret = writeCsv(out_path, regs);
			}
		}
	}
	return ret;
}

Result RegCsvList::writeRegList(PlFw* camera, const std::filesystem::path& file_path)
{
	Result ret;
	RegDevs					devs;
	std::vector<RegInfo>	regs;
	std::vector<RegList>	reg_list;

	ret = camera->getCamProperty(CMD_REG_DEVS, &devs);
	if (ret == SUCCESS) {
		if (devs.size() == 0) {
			LOG_ERR("Control Register is not Supported\n");
			return ERR_NOT_SUPPORT;
		}
		ret = readCsv(false, file_path, devs, regs);
		if (ret == SUCCESS) {
			packRegList(regs, devs, reg_list);
			for (auto& list : reg_list) {
				ret = camera->setCamProperty(CMD_REG_LIST, &list);
				if (ret != SUCCESS) {
					LOG_ERR("Failed to Read REG_LIST : %d\n", ret);
					break;
				}
			}
		}
	}

	return ret;
}

Result RegCsvList::readCsv(bool read_list, const std::filesystem::path& file_path,
	const RegDevs& reg_devs, std::vector<RegInfo>& regs)
{
	std::ifstream				ifs(file_path);
	std::string					line = "";
	std::vector<std::string>	str;
	std::string					name = "";
	std::string					comment = "#";
	RegInfo						reg;
	int							tgt;
	bool						found;

	regs.clear();
	if (ifs.bad() || ifs.eof() || ifs.fail() || file_path.empty()) {
		LOG_ERR("Fail to open : file = %s\n", file_path.string<char>().c_str());
		return ERR_NOT_EXIST;
	}
	while (std::getline(ifs, line)) {
		if (line.compare(0, 1, comment) != 0) {
			/* delete comment */
			if (line.find('#') != std::string::npos) {
				std::istringstream	lst(line);
				std::getline(lst, line, '#');
			}
			/* delimit csv section */
			splitStr(line, ',', str);
			if (str.size() >= 3) {	// parameter is exist
				try {
					tgt = std::stoi(str[0].c_str());
				} catch (...) {
					LOG_ERR("invalid string\n");
					return ERR_NOT_SUPPORT;
				}
				found = false;
				for (auto dev : reg_devs) {
					if (tgt == static_cast<int>(dev.target) && (dev.list_len > 0)) {
						reg.target = static_cast<RegDevType>(tgt);
						reg.addr = static_cast<uint16_t>(std::strtoul(str[1].c_str(), NULL, 16));
						reg.value = read_list ? 0 : static_cast<uint16_t>(std::strtoul(str[2].c_str(), NULL, 16));
						regs.push_back(reg);
						found = true;
						break;
					}
				}
				if (!found) {
					LOG_ERR("Not supported register target : %d\n", tgt);
					return ERR_NOT_SUPPORT;
				}
			}
		}
		line.clear();
	}
	if (regs.empty()) {
		LOG_ERR("%s is empty\n", file_path.string<char>().c_str());
		return ERR_NOT_EXIST;
	}
	return SUCCESS;
}

Result RegCsvList::writeCsv(const std::filesystem::path& file_path, const std::vector<RegInfo>& regs)
{
	Result				ret = SUCCESS;
	std::ofstream		ofs(file_path, (std::ios_base::out | std::ios_base::trunc));
	const std::string	delimiter = ",\t";
	std::string			lf = "\n";

	for(auto reg : regs) {
		ofs << std::to_string(reg.target) << delimiter <<
			std::hex << reg.addr << delimiter <<
			std::hex << reg.value << lf;
		if (ofs.bad()) {
			LOG_ERR("Feild to write %s\n", file_path.string<char>().c_str());
			ret = ERR_SYSTEM;
			break;
		}
	}
	if (ret == SUCCESS) { ofs.flush(); }
	ofs.close();
	return ret;
}

void RegCsvList::packRegList(const std::vector<RegInfo>& regs, const RegDevs& reg_devs,
	std::vector<RegList>& reg_list)
{
	reg_list.clear();
	if (regs.size() > 0) {
		RegList list;
		uint32_t i;
		uint8_t limit = 0;
		list.values.clear();
		list.target = regs[0].target;
		list.addr = regs[0].addr;
		list.values.push_back(regs[0].value);
		for (auto dev : reg_devs) {
			if (dev.target == list.target) {
				limit = dev.list_len;
				break;
			}
		}
		for (i = 1U; i < static_cast<uint32_t>(regs.size()); i++) {
			if ((regs[i].target != list.target) ||
				(regs[i].addr != (list.addr + list.values.size())) ||
				(static_cast<uint8_t>(list.values.size()) >= limit)) {
				reg_list.push_back(list);
				list.values.clear();
				list.target = regs[i].target;
				list.addr = regs[i].addr;
				for (auto dev : reg_devs) {
					if (dev.target == list.target) {
						limit = dev.list_len;
						break;
					}
				}
			}
			list.values.push_back(regs[i].value);
		}
		if (list.values.size() > 0) {
			reg_list.push_back(list);
		}
	}
}

void RegCsvList::unpackRegList(const std::vector<RegList>& reg_list, std::vector<RegInfo>& regs)
{
	RegInfo reg;
	reg.target = reg_list[0].target;
	reg.addr = reg_list[0].addr;
	reg.value = reg_list[0].values[0];

	regs.clear();
	for (auto list : reg_list) {
		reg.target = list.target;
		reg.addr = list.addr;
		for (auto val : list.values) {
			reg.value = val;
			regs.push_back(reg);
			reg.addr++;
		}
	}
}

void RegCsvList::getResultName(const std::filesystem::path& org_path, std::filesystem::path& dst_path)
{
	std::filesystem::path filename = org_path.stem();
	dst_path = org_path.parent_path();
	dst_path.append(filename.string<char>().c_str());
	dst_path += "_Result.csv";
}

void RegCsvList::delChr(std::string& str, char delc)
{
	size_t pos = 0;
	while (1) {
		pos = str.find(delc);
		if (pos == std::string::npos) { break; }
		str.erase(pos, 1);
	}
}

void RegCsvList::splitStr(const std::string& istr, char delimiter, std::vector<std::string>& ostr)
{
	std::istringstream	ifs(istr);
	std::string			str = "";

	ostr.clear();
	while (std::getline(ifs, str, delimiter)) {
		delChr(str, ' ');	// delete space
		delChr(str, '\"');	// delete quotation marks
		delChr(str, '\t');	// delete tab
		delChr(str, '\n');	// delete LF
		delChr(str, '\r');	// delete CR
		ostr.push_back(str);
	}
}

} // namespace krm
