/*------------------------------------------------------------------*/
/// @file		Record.h
/// @brief		Record API class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "RecordType.h"

namespace krm::Record
{

class Record
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	Record(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~Record(void);

	/*------------------------------------------------------------------*/
	/// @brief	Open Record class
	/// @param	[in]	param	Parameter
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Invalid argument
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_FULL		Full (Buffer, Storage etc.)
	/// @retval	ERR_SYSTEM		Other errors (System error)
	/*------------------------------------------------------------------*/
	Result openRec(const ConfigParam& config);
	/*------------------------------------------------------------------*/
	/// @brief	Open Record class
	/// @param	[in]	config	Configuration
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_ARG		Invalid argument
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_FULL		Full (Buffer, Storage etc.)
	/// @retval	ERR_SYSTEM		Other errors (System error)
	/*------------------------------------------------------------------*/
	Result openRec(const RecInfoParam& param);
	/*------------------------------------------------------------------*/
	/// @brief	Close Record class
	/// @retval	SUCCESS			Success
	/*------------------------------------------------------------------*/
	Result closeRec(void);
	/*------------------------------------------------------------------*/
	/// @brief	Record a frame
	/// @param	[in]	frame	a frame data to record
	/// @retval	SUCCESS			Success
	/// @retval	REACH_EOF		Reached end of file
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		Other errors (System error)
	/*------------------------------------------------------------------*/
	Result recFrame(const Frame& frame);

	/* delete copy */
	Record(const Record&) = delete;
	Record& operator=(const Record&) = delete;

private:
	class RecordPriv;

	std::unique_ptr<RecordPriv>	rec_;	/* record class */
};

} // namespace krm::Record
