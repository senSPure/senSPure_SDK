/*------------------------------------------------------------------*/
/// @file		MsgQue.h
/// @brief		Massage queue class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

#include "TpTofSdkDefine.h"

namespace krm
{
/*------------------------------------------------------------------*/
/// @brief	Massage structure
/*------------------------------------------------------------------*/
struct MsgData {
	uint32_t	cmd;	/*!< message command */
	void*		msg;	/*!< message data */
	Result		ret;	/*!< Result of message */
};

class MsgQue
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/*------------------------------------------------------------------*/
	MsgQue(void);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~MsgQue(void);

	/*------------------------------------------------------------------*/
	/// @brief	Send Event message
	/// @param	[out]	msg_data		message command & data
	/*------------------------------------------------------------------*/
	void send(MsgData& msg_data);
	/*------------------------------------------------------------------*/
	/// @brief	Receive Massage
	/// @param	[out]	msg_data		message command & data
	/// @retval	SUCCESS			Success
	/// @retval	CANCELED		Canceled wait state
	/*------------------------------------------------------------------*/
	Result recv(MsgData& msg_data);
	/*------------------------------------------------------------------*/
	/// @brief	Receive Massage with timeout
	/// @param	[out]	msg_data		message command & data
	/// @param	[in]	timeout			timeout [ms]
	/// @retval	SUCCESS			Success
	/// @retval	CANCELED		Canceled wait state
	/// @retval ERR_TIMEOUT		Timeout from waiting
	/*------------------------------------------------------------------*/
	Result recv(MsgData& msg_data, uint32_t timeout);
	/*------------------------------------------------------------------*/
	/// @brief	Poll Massage
	/// @param	[out]	msg_data		message command & data
	/// @retval	SUCCESS			Success
	/// @retval	ERR_EMPTY		Queue is empty
	/*------------------------------------------------------------------*/
	Result poll(MsgData& msg_data);

	/*------------------------------------------------------------------*/
	/// @brief	Cancel to receive
	/*------------------------------------------------------------------*/
	void cancel(void);
	/*------------------------------------------------------------------*/
	/// @brief	Get number of queueing messages
	/// @return number of queueing messages
	/*------------------------------------------------------------------*/
	size_t getQueNum(void);

	/* delete copy */
	MsgQue(const MsgQue&) = delete;
	MsgQue& operator=(const MsgQue&) = delete;

private:
	bool						canceled_;	/*!< Canceled state */
	std::queue<MsgData>			que_;		/*!< Message queue */
	std::mutex					mtx_;		/*!< Mutex */
	std::condition_variable		cond_;		/*!< Condition variable */

	Result popMsg(MsgData& msg);			/*!< Pop message queue */
};

} // namespace krm
