/*------------------------------------------------------------------*/
/// @file		FrameData.h
/// @brief		A frame data class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <vector>
#include <unordered_map>

#include "TpTofSdkDefine.h"
#include "PlayBackType.h"

namespace krm
{
/*------------------------------------------------------------------*/
/// @brief	Extension Frame information
/*------------------------------------------------------------------*/
struct FrameExtInfo {
	uint16_t			rcv_fps;		/*!< received frame rate [fps * 100] */
	PlayBack::PlayTime	play_time;		/*!< playing time(only PlayBack) */
};


class FrameData
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	FrameData(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	virtual ~FrameData(void);

	/*------------------------------------------------------------------*/
	/// @brief	Get Frame data
	/// @return Frame data
	/*------------------------------------------------------------------*/
	Frame* getFrame(void) { return &frame_; }

	/*------------------------------------------------------------------*/
	/// @brief	Get Extension Frame Information
	/// @return Extension Frame Information
	/*------------------------------------------------------------------*/
	FrameExtInfo* getFrameExt(void) { return &ext_info_; }

	/*------------------------------------------------------------------*/
	/// @brief	Get user buffer pointer
	/// @param	[in]	buf_id		User buffer ID
	/// @return	user buffer pointer  (nullptr : invalid User buffer ID)
	/*------------------------------------------------------------------*/
	void* getUserBuf(uint16_t buf_id);

	/* delete copy */
	FrameData(const FrameData&) = delete;
	FrameData& operator=(const FrameData&) = delete;

protected:
	Frame			frame_;		/* Frame date */
	FrameExtInfo	ext_info_;	/* Extension Frame information */
	std::unordered_map<uint16_t, std::vector<uint8_t>>	user_buf_;	/* User's buffers */

private:
	void clear(void);	/* clear data */
};

} // namespace krm
