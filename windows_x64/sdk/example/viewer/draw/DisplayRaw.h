/*------------------------------------------------------------------*/
/// @file		DisplayRaw.h
/// @brief		Display RAW Image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#pragma once

#include <memory>
#include <string_view>
#include <functional>
#include <GLFW/glfw3.h>

#include "TpTofSdkDefine.h"
#include "DrawRaw.h"

namespace krm
{

static constexpr uint8_t RAW_MAX = (IMG_RAW4 - IMG_RAW1) + 1U;
using DrawRaws = std::array<std::shared_ptr<DrawRaw>, RAW_MAX>;

class DisplayRaw
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	///	@param	[in]	draws	Drawing classes
	/*------------------------------------------------------------------*/
	explicit DisplayRaw(DrawRaws& draws);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~DisplayRaw(void);

	/*------------------------------------------------------------------*/
	/// @brief	set Display scale
	///	@param	[in]	scale	display scale
	/*------------------------------------------------------------------*/
	void setScale(uint16_t scale);

	/*------------------------------------------------------------------*/
	/// @brief	set format & create window
	///	@param	[in]	img_fmts	Image formats
	///	@retval	true	Success
	///	@retval	false	Failed to create window
	/*------------------------------------------------------------------*/
	bool setFormat(const ImageFormats& img_fmts);
	/*------------------------------------------------------------------*/
	/// @brief	destroy window
	/*------------------------------------------------------------------*/
	void destroyWin(void);

	/*------------------------------------------------------------------*/
	/// @brief	hide/hide window
	///	@retval	true	window is hiding
	///	@retval	false	window is showing
	/*------------------------------------------------------------------*/
	bool toggle(void);

	/*------------------------------------------------------------------*/
	/// @brief	refresh image
	/*------------------------------------------------------------------*/
	void refresh(void);
	/*------------------------------------------------------------------*/
	/// @brief	clear image
	/*------------------------------------------------------------------*/
	void clear(void);
	/*------------------------------------------------------------------*/
	/// @brief	draw image
	/*------------------------------------------------------------------*/
	void draw(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set mouse callback in GLFW window
	///	@param	[in]	enter_cb	callback for enter window
	///	@param	[in]	cursor_cb	callback for cursor position
	/*-----------------------------------------------------------------*/
	void setMouseCb(GLFWcursorenterfun enter_cb, GLFWcursorposfun cursor_cb);

private:
	std::string_view	WIN_NAME = "RAW Images";

	DrawRaws			draws_;			/* Draw RAW images */
	bool				exist_;			/* displayed window */

	std::array<ImageFormat, RAW_MAX>	fmts_;		/* original RAW format */
	uint8_t								raw_num_;	/* Number of RAW images */
	std::vector<RgbaColor>				image_;		/* image data(scaled) */
	uint16_t							width_;		/* original image width */
	uint16_t							height_;	/* original image height */
	uint16_t							scale_;		/* display scale */
	uint16_t							disp_w_;	/* display width */
	uint16_t							disp_h_;	/* display height */

	GLFWwindow*		win_;			/* drawing window */
	bool			hide_;			/* hide window */

	/*------------------------------------------------------------------*/
	/// @brief	crate window
	///	@param	[in]	w	window width
	///	@param	[in]	h	window height
	///	@retval	true	Success
	///	@retval	false	Failed to create window
	/*------------------------------------------------------------------*/
	bool createWin(uint16_t w, uint16_t h);
	/*------------------------------------------------------------------*/
	/// @brief	update hide status
	/*------------------------------------------------------------------*/
	void updateHideStatus(void);
	/*------------------------------------------------------------------*/
	/// @brief	show window
	/*------------------------------------------------------------------*/
	void show(void);
	/*------------------------------------------------------------------*/
	/// @brief	hide window
	/*------------------------------------------------------------------*/
	void hide(void);
	/*------------------------------------------------------------------*/
	/// @brief	close event
	///	@param	[in]	window	The window that the user attempted to close.
	/*------------------------------------------------------------------*/
	static void closeEvent(GLFWwindow* window);
};

} // namespace krm
