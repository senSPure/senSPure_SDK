/*------------------------------------------------------------------*/
/// @file		Camera.h
/// @brief		Camera API class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>

#include "CameraType.h"

namespace krm
{

class Camera
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	/// @param	[in]	type	Camera type
	/// @param	[in]	param	Camera parameter
	/// @param	[out]	res		Result
	/// @retval	SUCCESS			Success
	/// @retval	ERR_OVER_RANGE	Over the range
	/// @retval ERR_BAD_ARG		Invalid argument
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Camera(CameraType type, const void* param, Result& res);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~Camera(void);

	/*------------------------------------------------------------------*/
	/// @brief	Get connected devices
	/// @param	[out]	dev_list	Connected devices
	/// @retval	SUCCESS			Success
	/// @retval	ERR_NOT_EXIST	Not connected
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result getDeviceList(std::vector<ConnDevice>& dev_list);

	/*------------------------------------------------------------------*/
	/// @brief	Open Camera device
	/// @param	[in]	dev_id	Camera Device ID
	/// @retval	SUCCESS			Success
	/// @retval	ERR_NOT_EXIST	Not connected
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_NOT_SUPPORT	Not supported device
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result openDevice(uint16_t dev_id);
	/*------------------------------------------------------------------*/
	/// @brief	Close Camera device
	/// @retval	SUCCESS			Success
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result closeDevice(void);

	/*------------------------------------------------------------------*/
	/// @brief	Get Camera device property
	/// @param	[in]		prop_cmd	Property command
	/// @param	[in,out]	param		Property parameter
	/// @retval	SUCCESS			Success
	/// @retval	ERR_INVALID_PTR	Invalid pointer
	/// @retval	ERR_OVER_RANGE	Over the range
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_TIMEOUT		Timeout communication between camera device
	/// @retval	ERR_NOT_SUPPORT	Not supported
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result getProperty(uint16_t prop_cmd, void* param);
	/*------------------------------------------------------------------*/
	/// @brief	Set Camera device property
	/// @param	[in]		prop_cmd	Property command
	/// @param	[in]		param		Property parameter
	/// @retval	SUCCESS			Success
	/// @retval	ERR_INVALID_PTR	Invalid pointer
	/// @retval	ERR_OVER_RANGE	Over the range
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_TIMEOUT		Timeout communication between camera device
	/// @retval	ERR_NOT_SUPPORT	Not supported
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result setProperty(uint16_t prop_cmd, const void* param = nullptr);

	/*------------------------------------------------------------------*/
	/// @brief	Start capture images
	/// @retval	SUCCESS			Success
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result startCapture(void);
	/*------------------------------------------------------------------*/
	/// @brief	Stop capture images
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result stopCapture(void);

	/*------------------------------------------------------------------*/
	/// @brief	Capture images
	/// @param	[in,out]		frame	Captured images
	/// @param	[in]			block	Block until received
	/// @retval	SUCCESS			Success
	/// @retval	CANCELED		Canceled wait state
	/// @retval	REACH_EOF		Reached end of file
	/// @retval	ERR_NOT_EXIST	Not exist
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_TIMEOUT		Timeout from waiting
	/// @retval ERR_EMPTY		Buffer is empty
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result capture(Frame& frame, bool block = true);
	/*------------------------------------------------------------------*/
	/// @brief	Cancel to blocking in Camera::capture().
	/// @retval	SUCCESS			Success
	/// @retval	ERR_BAD_STATE	Bad state transmission
	/// @retval	ERR_SYSTEM		System error
	/*------------------------------------------------------------------*/
	Result cancel(void);

	/* delete copy */
	Camera(const Camera&) = delete;
	Camera& operator=(const Camera&) = delete;

private:
	class CameraPriv;
	std::unique_ptr<CameraPriv> priv_; /* private class */
};

} // namespace krm
