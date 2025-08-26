/*------------------------------------------------------------------*/
/// @file		DisplayRaw.cpp
/// @brief		Display RAW Image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <algorithm>
#include "DisplayRaw.h"

namespace tof_camera_example
{

DisplayRaw::DisplayRaw(DrawRaws& draws) :
	draws_(draws), exist_(false), raw_num_(0),
	width_(0), height_(0), scale_(1U), disp_w_(0), disp_h_(0),
	win_(NULL), hide_(true)
{
	image_.clear();
	for (auto& fmt : fmts_) {
		fmt.set();
	}
}

DisplayRaw::~DisplayRaw(void)
{
	this->destroyWin();
}

void DisplayRaw::setScale(uint16_t scale)
{
	if (scale > 0) { scale_ = scale; }
}

bool DisplayRaw::setFormat(const krm::ImageFormats& img_fmts)
{
	uint16_t	w = 0, h = 0;

	updateHideStatus();
	raw_num_ = 0;
	for (auto& fmt : fmts_) {
		fmt.set();
	}
	for (uint8_t i = krm::IMG_RAW1; i <= krm::IMG_RAW4; i++) {
		if (img_fmts[i].pixels > 0) {
			fmts_[raw_num_++] = img_fmts[i];
		}
	}
	w = fmts_[0].width;
	if (raw_num_ > 1U) { w += fmts_[1].width; }
	h = fmts_[0].height;
	if ((raw_num_ / 2U) > 1U) { h += fmts_[2].height; }

	if ((w == width_) && (h == height_)) {
		return true;
	}
	destroyWin();
	if ((w > 0) && (h > 0)) {
		return createWin(w, h);
	} else {
		width_ = 0;
		height_ = 0;
		disp_w_ = 0;
		disp_h_ = 0;
	}
	return true;
}

void DisplayRaw::destroyWin(void)
{
	if (win_ != NULL) {
		glfwDestroyWindow(win_);
		win_ = NULL;
		hide_ = true;
		width_ = 0;
		height_ = 0;
		disp_w_ = 0;
		disp_h_ = 0;
		image_.clear();
	}
}

bool DisplayRaw::toggle(void)
{
	updateHideStatus();
	if (hide_) {
		show();
	} else {
		hide();
	}
	return hide_;
}

void DisplayRaw::refresh(void)
{
	uint8_t		i, i_x, i_y;
	uint16_t	y, w, h;
	uint32_t	half_w, half_h;
	const RgbaColor*	src;
	RgbaColor*	dst;

	if (hide_) { return; }

	half_w = disp_w_ / 2U;
	half_h = (disp_h_ / 2U) * disp_w_;
	for (auto drw : draws_) { drw->lock(); }
	for (i = 0; i < raw_num_; i++) {
		i_x = (i % 2U);
		i_y = (i / 2U);
		w = fmts_[i].width * scale_;
		h = fmts_[i].height * scale_;
		src = draws_[i]->getDrawImage().data();
		if (!draws_[i]->getDrawImage().empty()) {
			dst = image_.data() + ((static_cast<uint32_t>(i_y) * half_h) + (static_cast<uint32_t>(i_x) * half_w));
			for (y = 0; y < h; y++) {
				std::copy(src, src + w, dst);
				src += w;
				dst += disp_w_;
			}
		}
	}
	for (auto drw : draws_) { drw->unlock(); }
}

void DisplayRaw::clear(void)
{
	std::fill(image_.begin(), image_.end(), BLACK_COLOR);
}

void DisplayRaw::draw(void)
{
	if (win_ == NULL) { return; }
	if (hide_) { return; }
	glfwMakeContextCurrent(win_);
	glViewport(0, 0, disp_w_, disp_h_);
	glClear(GL_COLOR_BUFFER_BIT);
	glPixelZoom(1, -1);
	glRasterPos2i(-1, 1);
	glDrawPixels(disp_w_, disp_h_, GL_RGBA, GL_UNSIGNED_BYTE, image_.data());
	glfwSwapBuffers(win_);
}

void DisplayRaw::setMouseCb(GLFWcursorenterfun enter_cb, GLFWcursorposfun cursor_cb)
{
	if (win_ != NULL) {
		(void)glfwSetCursorEnterCallback(win_, enter_cb);
		(void)glfwSetCursorPosCallback(win_, cursor_cb);
	}
}

bool DisplayRaw::createWin(uint16_t w, uint16_t h)
{
	uint16_t disp_w = w * scale_, disp_h = h * scale_;
	glfwWindowHint(GLFW_VISIBLE, hide_ ? GL_FALSE : GL_TRUE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_MAXIMIZED, GL_FALSE);
	glfwWindowHint(GLFW_RED_BITS, 8);
	glfwWindowHint(GLFW_GREEN_BITS, 8);
	glfwWindowHint(GLFW_BLUE_BITS, 8);
	glfwWindowHint(GLFW_ALPHA_BITS, 8);

	win_ = glfwCreateWindow(disp_w, disp_h, WIN_NAME.data(), NULL, NULL);
	if (win_ == NULL) {
		return false;
	}

	glfwSetWindowCloseCallback(win_, static_cast<GLFWwindowclosefun>(&this->closeEvent));
	glfwSetWindowAspectRatio(win_, disp_w, disp_h);
	glfwSetWindowShouldClose(win_, GLFW_FALSE);
	glfwSetInputMode(win_, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
	width_ = w;
	height_ = h;
	disp_w_ = disp_w;
	disp_h_ = disp_h;
	image_.resize(disp_w * disp_h);
	clear();
	if (!hide_) {
		refresh();
	}
	return true;
}

void DisplayRaw::updateHideStatus(void)
{
	if (win_ != NULL) {
		hide_ = (glfwGetWindowAttrib(win_, GLFW_VISIBLE) == GLFW_FALSE);
	}
}

void DisplayRaw::show(void)
{
	if (win_ != NULL) {
		glfwShowWindow(win_);
		hide_ = false;
		refresh();
		draw();
	}
}

void DisplayRaw::hide(void)
{
	if (win_ != NULL) {
		glfwHideWindow(win_);
		hide_ = true;
	}
}

void DisplayRaw::closeEvent(GLFWwindow* window)
{
	glfwHideWindow(window);
}

} // namespace tof_camera_example
