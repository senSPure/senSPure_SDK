/*------------------------------------------------------------------*/
/// @file		DrawPcd.cpp
/// @brief		Draw 3D-image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <algorithm>

#include "DrawPcd.h"

namespace tof_camera_example
{

DrawPcd::DrawPcd(const std::shared_ptr<ColorTable>& color_table,
					const std::shared_ptr<GrayTable>& gray_table) :
	update_pnt_(false), update_line_(false), update_view_(false),
	color_table_(color_table), gray_table_(gray_table), pcd_ir_(false),
	win_x_(0), win_y_(0), win_w_(0), win_h_(0), cz_(0.0F),
	view_pos_kind_(VIEW_TOP),
	eye_({0.0F, 0.0F, 0.0F}), look_at_({0.0F, 0.0F, 0.0F}), up_({0.0F, 0.0F, 0.0F}),
	drw_scale_(1.0F), clip_w_(10000.0F), clip_h_(0.0F),
	clip_far_(static_cast<float>(UINT16_MAX)),
	grid_xz_max_(0), grid_fov_max_(0.0F),
	is_over_90_degree_(false), grid_tan_half_fov_h_(0.0F), aux_grid_(1000U),
	interleave_enable_(true), interleave_distance_(0.0F), interleave_factor_(2U)
{
	pcd_org_.kind = krm::PCD_XYZ;
	pcd_org_.resize(img_fmt_);
	pcd_draw_.resize(img_fmt_.pixels);
	pcd_screen_.resize(img_fmt_.pixels);
	proj_mat_.fill(0.0F);
	trans_mat_.fill(0.0F);
	trans_scaled_mat_.fill(0.0F);
	grid_screen_.resize(0);
}

DrawPcd::~DrawPcd(void)
{
	color_table_.reset();
	gray_table_.reset();
	pcd_org_.data.clear();
	pcd_draw_.clear();
	pcd_screen_.clear();
	grid_screen_.clear();
}

void DrawPcd::setWindow(float win_x, float win_y, float win_w, float win_h)
{
	win_x_ = win_x;
	win_y_ = win_y;
	win_w_ = win_w;
	win_h_ = win_h;
	clip_h_ = (clip_w_ * win_h_) / win_w_;
	createOrtho(clip_w_, clip_h_, clip_far_, proj_mat_);
	updateGridSetting();
}

void DrawPcd::setFormat(const krm::ImageFormat& dpt_fmt)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	img_fmt_ = dpt_fmt;
	pcd_org_.resize(dpt_fmt);
	pcd_draw_.resize(dpt_fmt.pixels);
	pcd_screen_.resize(dpt_fmt.pixels);
	pcd_filtered_.resize(dpt_fmt.pixels);
	clear();
}

void DrawPcd::copyDrawImage(std::vector<krm::Point3d>& image)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	image.assign(pcd_draw_.begin(), pcd_draw_.end());
}

void DrawPcd::convImage(const std::vector<uint16_t>& /*src_img*/)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	RgbaColor color;
	krm::Point3d point;

	pcd_draw_.clear();
	if (pcd_ir_) {
		gray_table_->lock();
		for (auto& pcd : pcd_org_.data) {
			point = pcd;
			color = gray_table_->getColor(static_cast<uint16_t>(point.color));
			std::memcpy(&point.color, color.data(), sizeof(point.color));
			pcd_draw_.push_back(point);
		}
		gray_table_->unlock();
	} else {
		color_table_->lock();
		for (auto& pcd : pcd_org_.data) {
			point = pcd;
			color = color_table_->getColor(static_cast<uint16_t>(point.z));
			std::memcpy(&point.color, color.data(), sizeof(point.color));
			pcd_draw_.push_back(point);
		}
		color_table_->unlock();
	}
	update_pnt_ = true;
}

void DrawPcd::interleaveImage(const std::vector<krm::Point3d>& src)
{
	std::vector<krm::Point3d>& dst = pcd_filtered_;
	bool enable;
	float distance;
	uint8_t factor;
	uint16_t width;
	uint16_t x;
	uint16_t y;

	getInterleave(enable, distance, factor);
	dst.clear();
	if (enable) {
		width = img_fmt_.width;
		x = y = 0;
		for (auto& pcd : src) {
			if ((pcd.color != UINT32_MAX) && ((pcd.z > distance) || (((x % factor) == 0) && ((y % factor) == 0)))) {
				dst.push_back(pcd);
			}
			if (++x >= width) {
				x = 0;
				y++;
			}
		}
	} else {
		for (auto& pcd : src) {
			if (pcd.color != UINT32_MAX) {
				dst.push_back(pcd);
			}
		}
	}
}

void DrawPcd::setImage(const krm::PcdData& pcd)
{
	interleaveImage(pcd.data);
	pcd_org_.data = pcd_filtered_;
	convImage(org_img_);
}

void DrawPcd::clear(void)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	pcd_org_.data.clear();
	pcd_draw_.clear();
	pcd_screen_.clear();
	pcd_filtered_.clear();
}

void DrawPcd::setKind(bool pcd_ir)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	pcd_ir_ = pcd_ir;
	convImage(org_img_);
}

void DrawPcd::setViewPos(void)
{
	if (view_pos_kind_ == VIEW_TOP) {
		eye_ = { 0.0F, cz_ + 10.0F, cz_ - EPS };
		look_at_ = { 0.0F, 0.0F, cz_ };
		up_ = { 0.0F, EPS, 1.0F };
	} else if (view_pos_kind_ == VIEW_LEFT) {
		eye_ = { -cz_ - 10.0F, EPS, cz_ - EPS };
		look_at_ = { 0.0F, 0.0F, cz_ };
		up_ = { 0.0F, 1.0F, EPS };
	} else if (view_pos_kind_ == VIEW_RIGHT) {
		eye_ = { cz_ + 10.0F, EPS, cz_ - EPS };
		look_at_ = { 0.0F, 0.0F, cz_ };
		up_ = { 0.0F, 1.0F, EPS };
	} else {
		eye_ = { 0.0F, EPS, -10.0F };
		look_at_ = { 0.0F, 0.0F, cz_ };
		up_ = { 0.0F, 1.0F, EPS };
	}
	drw_scale_ = 1.0F;
	updateViewSize();

	update_view_ = true;
}

void DrawPcd::setViewPos(ViewKind kind)
{
	view_pos_kind_ = kind;
	setViewPos();
}

void DrawPcd::setViewCameraRot(float delta_x, float delta_y)
{
	float dx_angle = static_cast<float>((2.0 * M_PI) / static_cast<double>(win_w_));
	float dy_angle = static_cast<float>(M_PI / static_cast<double>(win_h_));
	float x_angle = delta_x * dx_angle;
	float y_angle = delta_y * dy_angle;
	std::array<float, MATRIX3x3_SIZE> rot_x, rot_y;
	std::array<float, MATRIX4x4_SIZE> view;
	DrawPcdPoint view_x_vec;

	lookAt(view);
	view_x_vec.x = view[0];
	view_x_vec.y = view[1];
	view_x_vec.z = view[2];

	createRotMat(x_angle, up_, rot_x);
	createRotMat(y_angle, view_x_vec, rot_y);

	up_.x = (rot_y[0] * up_.x) + (rot_y[1] * up_.y) + (rot_y[2] * up_.z);
	up_.y = (rot_y[3] * up_.x) + (rot_y[4] * up_.y) + (rot_y[5] * up_.z);
	up_.z = (rot_y[6] * up_.x) + (rot_y[7] * up_.y) + (rot_y[8] * up_.z);
	eye_.x = (rot_x[0] * (eye_.x - look_at_.x)) + (rot_x[1] * (eye_.y - look_at_.y)) + (rot_x[2] * (eye_.z - look_at_.z)) + look_at_.x;
	eye_.y = (rot_x[3] * (eye_.x - look_at_.x)) + (rot_x[4] * (eye_.y - look_at_.y)) + (rot_x[5] * (eye_.z - look_at_.z)) + look_at_.y;
	eye_.z = (rot_x[6] * (eye_.x - look_at_.x)) + (rot_x[7] * (eye_.y - look_at_.y)) + (rot_x[8] * (eye_.z - look_at_.z)) + look_at_.z;
	eye_.x = (rot_y[0] * (eye_.x - look_at_.x)) + (rot_y[1] * (eye_.y - look_at_.y)) + (rot_y[2] * (eye_.z - look_at_.z)) + look_at_.x;
	eye_.y = (rot_y[3] * (eye_.x - look_at_.x)) + (rot_y[4] * (eye_.y - look_at_.y)) + (rot_y[5] * (eye_.z - look_at_.z)) + look_at_.y;
	eye_.z = (rot_y[6] * (eye_.x - look_at_.x)) + (rot_y[7] * (eye_.y - look_at_.y)) + (rot_y[8] * (eye_.z - look_at_.z)) + look_at_.z;

	updateMatrix();
}

void DrawPcd::setViewCameraPos(float delta_x, float delta_y)
{
	float trans_rate = clip_far_ / (drw_scale_ * POS_TRANS_RATE);
	float dx = (-delta_x) * trans_rate;
	float dy = delta_y * trans_rate;

	float length;
	DrawPcdPoint trans, delta;
	std::array<float, MATRIX3x3_SIZE> matrix;

	trans.x = look_at_.x - eye_.x;
	trans.y = look_at_.y - eye_.y;
	trans.z = look_at_.z - eye_.z;
	length = std::sqrt((trans.x * trans.x) + (trans.y * trans.y) + (trans.z * trans.z));
	matrix[2] = trans.x / length;
	matrix[5] = trans.y / length;
	matrix[8] = trans.z / length;

	trans.x = (up_.y * matrix[8]) - (up_.z * matrix[5]);
	trans.y = (up_.z * matrix[2]) - (up_.x * matrix[8]);
	trans.z = (up_.x * matrix[5]) - (up_.y * matrix[2]);
	length = std::sqrt((trans.x * trans.x) + (trans.y * trans.y) + (trans.z * trans.z));

	matrix[0] = trans.x / length;
	matrix[3] = trans.y / length;
	matrix[6] = trans.z / length;

	matrix[1] = (matrix[5] * matrix[6]) - (matrix[8] * matrix[3]);
	matrix[4] = (matrix[8] * matrix[0]) - (matrix[2] * matrix[6]);
	matrix[7] = (matrix[2] * matrix[3]) - (matrix[5] * matrix[0]);

	delta.x = (matrix[0] * dx) + (matrix[1] * dy);
	delta.y = (matrix[3] * dx) + (matrix[4] * dy);
	delta.z = (matrix[6] * dx) + (matrix[7] * dy);

	eye_.x += delta.x;
	eye_.y += delta.y;
	eye_.z += delta.z;
	look_at_.x += delta.x;
	look_at_.y += delta.y;
	look_at_.z += delta.z;

	updateMatrix();
}

void DrawPcd::scalingView(bool is_zoom)
{
	float diff_scale = SCALE_RATE / drw_scale_;

	diff_scale += is_zoom ? -5.0F : 5.0F;
	if (diff_scale <= 5.0F) {
		diff_scale = 5.0F;
	}
	drw_scale_ = SCALE_RATE / diff_scale;

	updateMatrix(true);
}

void DrawPcd::convScreenPcd(const std::vector<krm::Point3d>* world_pos, bool need_update, std::vector<krm::Point3d>& out_pnt)
{
	if (need_update || update_pnt_ || update_view_) {
		krm::Point3d draw_pnt;
		pcd_screen_.clear();
		for (auto point : *world_pos) {
			draw_pnt.color = point.color;

			draw_pnt.x = (trans_scaled_mat_[0] * point.x) + (trans_scaled_mat_[1] * point.y)
						+ (trans_scaled_mat_[2] * point.z) + trans_scaled_mat_[3];
			draw_pnt.y = (trans_scaled_mat_[4] * point.x) + (trans_scaled_mat_[5] * point.y)
						+ (trans_scaled_mat_[6] * point.z) + trans_scaled_mat_[7];
			draw_pnt.z = (trans_scaled_mat_[8] * point.x) + (trans_scaled_mat_[9] * point.y)
						+ (trans_scaled_mat_[10] * point.z) + trans_scaled_mat_[11];

			draw_pnt.x = win_x_ + (((draw_pnt.x + 1.0F) * 0.5F) * win_w_);
			draw_pnt.y = win_y_ + win_h_ - (((draw_pnt.y + 1.0F) * 0.5F) * win_h_);

			pcd_screen_.push_back(draw_pnt);
		}
		std::sort(pcd_screen_.begin(), pcd_screen_.end(), [](const krm::Point3d& a, const krm::Point3d& b) {
			return (a.z > b.z);
		});
		update_pnt_ = false;
		update_view_ = false;
	}
	out_pnt = pcd_screen_;
}

void DrawPcd::convScreenGridCamera(bool need_update, std::vector<std::pair<krm::Point3d, krm::Point3d>>& out_pnt)
{
	if (need_update || update_line_ || update_view_) {
		krm::Point3d right_end_3d, left_end_3d;
		krm::Point3d origin, right_end, left_end;
		std::pair<krm::Point3d, krm::Point3d> pair;

		convScreenGridWorld(true, out_pnt);

		// circumference
		origin.x = win_x_ + (((trans_scaled_mat_[3] + 1.0F) * 0.5F) * win_w_);
		origin.y = win_y_ + win_h_ - (((trans_scaled_mat_[7] + 1.0F) * 0.5F) * win_h_);
		if (is_over_90_degree_) {
			right_end_3d.x = static_cast<float>(grid_xz_max_);
			left_end_3d.x = static_cast<float>(-grid_xz_max_);
			right_end_3d.z = left_end_3d.z = grid_fov_max_;
		} else {
			right_end_3d.x = grid_fov_max_;
			left_end_3d.x = -grid_fov_max_;
			right_end_3d.z = left_end_3d.z = static_cast<float>(grid_xz_max_);
		}
		right_end_3d.y = left_end_3d.y = 0.0F;
		right_end = convScreenPnt(right_end_3d);
		left_end = convScreenPnt(left_end_3d);

		pair = std::make_pair(origin, right_end);
		grid_screen_.push_back(pair);
		pair = std::make_pair(origin, left_end);
		grid_screen_.push_back(pair);

		update_line_ = false;
	}
	out_pnt = grid_screen_;
}

void DrawPcd::convScreenGridWorld(bool need_update, std::vector<std::pair<krm::Point3d, krm::Point3d>>& out_pnt)
{
	if (need_update || update_line_ || update_view_) {
		krm::Point3d start_3d, end_3d;
		krm::Point3d start, end;
		std::pair<krm::Point3d, krm::Point3d> pair;
		uint16_t i;
		grid_screen_.clear();

		// vertical line
		start_3d.x = end_3d.x = start_3d.y = end_3d.y = 0.0F;
		start_3d.z = 0.0F;
		end_3d.z = static_cast<float>(grid_xz_max_);
		start = convScreenPnt(start_3d);
		end = convScreenPnt(end_3d);
		pair = std::make_pair(start, end);
		grid_screen_.push_back(pair);

		for (i = aux_grid_; i <= grid_xz_max_; i += aux_grid_) {
			start_3d.x = end_3d.x = static_cast<float>(i);
			start = convScreenPnt(start_3d);
			end = convScreenPnt(end_3d);
			pair = std::make_pair(start, end);
			grid_screen_.push_back(pair);

			start_3d.x = end_3d.x = -static_cast<float>(i);
			start = convScreenPnt(start_3d);
			end = convScreenPnt(end_3d);
			pair = std::make_pair(start, end);
			grid_screen_.push_back(pair);
		}

		// horizontal line
		start_3d.x = -static_cast<float>(grid_xz_max_);
		end_3d.x = static_cast<float>(grid_xz_max_);

		for (i = 0; i <= grid_xz_max_; i += aux_grid_) {
			start_3d.z = end_3d.z = static_cast<float>(i);
			start = convScreenPnt(start_3d);
			end = convScreenPnt(end_3d);
			pair = std::make_pair(start, end);
			grid_screen_.push_back(pair);
		}
		update_line_ = false;
	}
	out_pnt = grid_screen_;
}

void DrawPcd::setFov(uint16_t horz)
{
	/* update grid setting */
	grid_tan_half_fov_h_ = std::tan(((static_cast<double>(horz) / 100.0) * 0.5) * (M_PI / 180.0));
	is_over_90_degree_ = (horz > DEGREE_90);
	if (is_over_90_degree_) {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) / grid_tan_half_fov_h_);
	} else {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) * grid_tan_half_fov_h_);
	}
	update_line_ = true;

	/* update view range setting */
	updateViewSize();
}

void DrawPcd::setRange(uint16_t max_range)
{
	/* update grid setting */
	if ((grid_xz_max_ < max_range) || (grid_xz_max_ - aux_grid_) >= max_range) {
		updateGridSetting(aux_grid_, max_range);
	}

	/* update view range setting */
	if (static_cast<uint16_t>(clip_far_) != max_range) {
		clip_far_ = static_cast<float>(max_range);
		updateViewSize();
	}
}

void DrawPcd::updateGridSetting(uint16_t aux_grid)
{
	uint16_t max_range = static_cast<uint16_t>(clip_far_);

	aux_grid_ = aux_grid;
	grid_xz_max_ = (((max_range - 1U) / aux_grid_) * aux_grid_) + aux_grid_;
	if (is_over_90_degree_) {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) / grid_tan_half_fov_h_);
	} else {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) * grid_tan_half_fov_h_);
	}
	grid_screen_.clear();
	if (grid_xz_max_ < max_range) {
		grid_screen_.resize((3U * (grid_xz_max_ / aux_grid_)) + 1U);
	}
	update_line_ = true;
}

void DrawPcd::updateGridSetting(uint16_t aux_grid, uint16_t max_range)
{
	aux_grid_ = aux_grid;
	grid_xz_max_ = (((max_range - 1U) / aux_grid_) * aux_grid_) + aux_grid_;
	if (is_over_90_degree_) {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) / grid_tan_half_fov_h_);
	} else {
		grid_fov_max_ = static_cast<float>(static_cast<double>(grid_xz_max_) * grid_tan_half_fov_h_);
	}
	grid_screen_.clear();
	if (grid_xz_max_ < max_range) {
		grid_screen_.resize((3U * (grid_xz_max_ / aux_grid_)) + 1U);
	}
	update_line_ = true;
}

void DrawPcd::lookAt(std::array<float, MATRIX4x4_SIZE>& matrix)
{
	float length;
	DrawPcdPoint trans;

	trans.x = look_at_.x - eye_.x;
	trans.y = look_at_.y - eye_.y;
	trans.z = look_at_.z - eye_.z;
	length = std::sqrt((trans.x * trans.x) + (trans.y * trans.y) + (trans.z * trans.z));
	matrix[8] = trans.x / length;
	matrix[9] = trans.y / length;
	matrix[10] = trans.z / length;

	trans.x = (up_.y * matrix[10]) - (up_.z * matrix[9]);
	trans.y = (up_.z * matrix[8]) - (up_.x * matrix[10]);
	trans.z = (up_.x * matrix[9]) - (up_.y * matrix[8]);
	length = std::sqrt((trans.x * trans.x) + (trans.y * trans.y) + (trans.z * trans.z));
	matrix[0] = trans.x / length;
	matrix[1] = trans.y / length;
	matrix[2] = trans.z / length;

	matrix[4] = (matrix[9] * matrix[2]) - (matrix[10] * matrix[1]);
	matrix[5] = (matrix[10] * matrix[0]) - (matrix[8] * matrix[2]);
	matrix[6] = (matrix[8] * matrix[1]) - (matrix[9] * matrix[0]);

	matrix[3] = -((eye_.x * matrix[0]) + (eye_.y * matrix[1]) + (eye_.z * matrix[2]));
	matrix[7] = -((eye_.x * matrix[4]) + (eye_.y * matrix[5]) + (eye_.z * matrix[6]));
	matrix[11] = -((eye_.x * matrix[8]) + (eye_.y * matrix[9]) + (eye_.z * matrix[10]));

	matrix[12] = matrix[13] = matrix[14] = 0.0F;
	matrix[15] = 1.0F;
}

void DrawPcd::createOrtho(float width, float height, float far, std::array<float, MATRIX4x4_SIZE>& matrix)
{
	float dz = CLIP_NEAR - far;

	matrix[0] = 2.0F / width;
	matrix[5] = 2.0F / height;
	matrix[10] = -1.0F / dz;
	matrix[11] = CLIP_NEAR / dz;
	matrix[1] = matrix[2] = matrix[3]
		= matrix[4] = matrix[6] = matrix[7]
		= matrix[8] = matrix[9] = matrix[12]
		= matrix[13] = matrix[14] = 0.0F;
	matrix[15] = 1.0F;
}

void DrawPcd::multiMat(const std::array<float, MATRIX4x4_SIZE>& mat1,
						const std::array<float, MATRIX4x4_SIZE>& mat2, std::array<float, MATRIX4x4_SIZE>& out_mat)
{
	uint8_t i, j, offset;
	for (i = 0; i < 4U; i++) {
		offset = i * 4U;
		for (j = 0; j < 4U; j++) {
			out_mat[offset + j] =
				(mat1[offset] * mat2[j]) + (mat1[offset + 1U] * mat2[j + 4U])
				+ (mat1[offset + 2U] * mat2[j + 8U]) + (mat1[offset + 3U] * mat2[j + 12U]);
		}
	}
}

void DrawPcd::createRotMat(float angle, const DrawPcdPoint& axis, std::array<float, MATRIX3x3_SIZE>& rot_mat)
{
	float cos_a = std::cos(angle);
	float sin_a = std::sin(angle);

	float axis_len = std::sqrt((axis.x * axis.x) + (axis.y * axis.y) + (axis.z * axis.z));
	DrawPcdPoint axis_norm = {axis.x / axis_len, axis.y / axis_len, axis.z / axis_len};
	DrawPcdPoint temp = {(1.0F - cos_a) * axis_norm.x, (1.0F - cos_a) * axis_norm.y, (1.0F - cos_a) * axis_norm.z};

	rot_mat[0] = cos_a + (temp.x * axis_norm.x);
	rot_mat[1] = (temp.y * axis_norm.x) - (sin_a * axis_norm.z);
	rot_mat[2] = (temp.z * axis_norm.x) + (sin_a * axis_norm.y);
	rot_mat[3] = (temp.x * axis_norm.y) + (sin_a * axis_norm.z);
	rot_mat[4] = cos_a + (temp.y * axis_norm.y);
	rot_mat[5] = (temp.z * axis_norm.y) - (sin_a * axis_norm.x);
	rot_mat[6] = (temp.x * axis_norm.z) - (sin_a * axis_norm.y);
	rot_mat[7] = (temp.y * axis_norm.z) + (sin_a * axis_norm.x);
	rot_mat[8] = cos_a + (temp.z * axis_norm.z);
}

void DrawPcd::updateViewSize(void)
{
	switch (view_pos_kind_) {
	case VIEW_LEFT:
	case VIEW_RIGHT:
		clip_w_ = clip_far_ * (1.0F + VIEW_MARGIN);
		clip_h_ = (clip_w_ * win_h_) / win_w_;
		break;
	case VIEW_FRONT:
	case VIEW_TOP:
	default:
		clip_w_ = grid_xz_max_ * 2.0F * (1.0F + VIEW_MARGIN);
		clip_h_ = (clip_w_ * win_h_) / win_w_;
		break;
	}
	createOrtho(clip_w_, clip_h_, clip_far_, proj_mat_);
	updateMatrix();
}

void DrawPcd::updateMatrix(bool is_only_scale)
{
	std::array<float, MATRIX4x4_SIZE> scale_mat;

	if (!is_only_scale) {
		std::array<float, MATRIX4x4_SIZE> view_mat;
		lookAt(view_mat);
		multiMat(proj_mat_, view_mat, trans_mat_);
	}
	scale_mat.fill(0.0F);
	scale_mat[0] = scale_mat[5] = scale_mat[10] = drw_scale_;
	scale_mat[15] = 1.0F;
	multiMat(scale_mat, trans_mat_, trans_scaled_mat_);
}

krm::Point3d DrawPcd::convScreenPnt(krm::Point3d world_pos)
{
	krm::Point3d out_pnt = { world_pos.color, 0.0F, 0.0F, 0.0F };

	out_pnt.x = (trans_scaled_mat_[0] * world_pos.x) + (trans_scaled_mat_[1] * world_pos.y)
				+ (trans_scaled_mat_[2] * world_pos.z) + trans_scaled_mat_[3];
	out_pnt.y = (trans_scaled_mat_[4] * world_pos.x) + (trans_scaled_mat_[5] * world_pos.y)
				+ (trans_scaled_mat_[6] * world_pos.z) + trans_scaled_mat_[7];

	out_pnt.x = win_x_ + (((out_pnt.x + 1.0F) * 0.5F) * win_w_);
	out_pnt.y = win_y_ + win_h_ - (((out_pnt.y + 1.0F) * 0.5F) * win_h_);

	return out_pnt;
}

void DrawPcd::setInterleave(bool enable, float distance, uint8_t factor)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	interleave_enable_ = enable;
	interleave_distance_ = distance;
	interleave_factor_ = factor;
}

void DrawPcd::getInterleave(bool& enable, float& distance, uint8_t& factor)
{
	std::lock_guard<std::recursive_mutex> lock(mtx_);
	enable = interleave_enable_;
	distance = interleave_distance_;
	factor = interleave_factor_;
}

} // namespace tof_camera_example
