/*------------------------------------------------------------------*/
/// @file		DrawPcd.h
/// @brief		Draw 3D-image class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/


#pragma once

#include <memory>

#include "DrawImage.h"
#include "ColorTable.h"
#include "GrayTable.h"

namespace krm
{
/*-----------------------------------------------------------------*/
/// @brief	conversion of RGBA-uint32_t structure
/*-----------------------------------------------------------------*/
struct RGBA32 {
	uint8_t		r;		/*!< Red */
	uint8_t		g;		/*!< Green */
	uint8_t		b;		/*!< Blue */
	uint8_t		a;		/*!< Alpha */
};

/*-----------------------------------------------------------------*/
/// @brief	Kind of View
/*-----------------------------------------------------------------*/
enum ViewKind {
	VIEW_TOP = 0,	/*!< Top View */
	VIEW_FRONT,		/*!< Front View */
	VIEW_LEFT,		/*!< Left View */
	VIEW_RIGHT,		/*!< Right View */
};

/*-----------------------------------------------------------------*/
/// @brief	Kind of Point cloud Image for drawing
/*-----------------------------------------------------------------*/
static const uint8_t IMG_PCD = IMG_KINDS;

class DrawPcd : public DrawImage
{
public:
	/*------------------------------------------------------------------*/
	/// @brief	constructor
	///	@param	[in]	color_table		Color table
	///	@param	[in]	gray_table		Grayscale table
	/*------------------------------------------------------------------*/
	explicit DrawPcd(const std::shared_ptr<ColorTable>& color_table,
					const std::shared_ptr<GrayTable>& gray_table);
	/*------------------------------------------------------------------*/
	/// @brief	destructor
	/*------------------------------------------------------------------*/
	~DrawPcd(void);

	/*------------------------------------------------------------------*/
	/// @brief	Set window position, size
	///	@param	[in]	win_x	Window x position
	///	@param	[in]	win_y	Window y position
	///	@param	[in]	win_w	Width of window
	///	@param	[in]	win_h	Height of window
	/*------------------------------------------------------------------*/
	void setWindow(float win_x, float win_y, float win_w, float win_h);

	/*------------------------------------------------------------------*/
	/// @brief	Set format
	///	@param	[in]	dpt_fmt		Depth Image format
	/*------------------------------------------------------------------*/
	void setFormat(const ImageFormat& dpt_fmt);

	/*------------------------------------------------------------------*/
	/// @brief	Get Draw point cloud data
	///	@return	drawing point cloud data pointer
	/*------------------------------------------------------------------*/
	const std::vector<Point3d>* getDrawImage(void) { return &pcd_draw_; }

	/*------------------------------------------------------------------*/
	/// @brief	Copy draw point cloud with lock
	///	@param	[out]	image	Point cloud image
	/*------------------------------------------------------------------*/
	void copyDrawImage(std::vector<Point3d>& image);

	/*------------------------------------------------------------------*/
	/// @brief	Set point cloud data
	///	@param	[in]	pcd		point cloud data
	/*------------------------------------------------------------------*/
	void setImage(const PcdData& pcd);
	/*------------------------------------------------------------------*/
	/// @brief	Calculate snapshot
	///	@param	[in]	pcd		point cloud data
	/// @return is finished taking snapshot
	/*------------------------------------------------------------------*/
	bool setSnapShot(const PcdData& pcd);

	/*------------------------------------------------------------------*/
	/// @brief	Clear drawing image
	/*------------------------------------------------------------------*/
	void clear(void) override;

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::startSnapshot
	/*------------------------------------------------------------------*/
	void startSnapshot(uint8_t cycle) override;

	/*------------------------------------------------------------------*/
	/// @brief	Set point cloud kind
	///	@param	[in]	pcd_ir		draw grayscale(IR) color
	/*------------------------------------------------------------------*/
	void setKind(bool pcd_ir);

	/*------------------------------------------------------------------*/
	/// @brief	Set position of viewing
	///	@param	[in]	kind		view kind
	/*------------------------------------------------------------------*/
	void setViewPos(void);
	void setViewPos(ViewKind kind);
	/*------------------------------------------------------------------*/
	/// @brief	Set center position of viewing rotation
	///	@param	[in]	cz		center z
	/*------------------------------------------------------------------*/
	inline void setCenterPos(float cz) { cz_ = cz; }

	/*------------------------------------------------------------------*/
	/// @brief	Set view camera rotation
	///	@param	[in]	delta_x		x of delta
	///	@param	[in]	delta_y		y of delta
	/*------------------------------------------------------------------*/
	void setViewCameraRot(float delta_x, float delta_y);
	/*------------------------------------------------------------------*/
	/// @brief	Set view camera position
	///	@param	[in]	delta_x		x of delta
	///	@param	[in]	delta_y		y of delta
	/*------------------------------------------------------------------*/
	void setViewCameraPos(float delta_x, float delta_y);
	/*------------------------------------------------------------------*/
	/// @brief	Scaling view
	///	@param	[in]	is_zoom		whether is zooming
	/*------------------------------------------------------------------*/
	void scalingView(bool is_zoom);

	/*------------------------------------------------------------------*/
	/// @brief	convert point to draw image position
	///	@param	[in]	world_pos	input points
	///	@param	[in]	need_update	is required of conversion calculation
	///	@param	[out]	out_pnt		output points
	/*------------------------------------------------------------------*/
	void convScreenPcd(const std::vector<Point3d> *world_pos, bool need_update, std::vector<Point3d>& out_pnt);
	/*------------------------------------------------------------------*/
	/// @brief	convert grid line to draw image position (camera view)
	///	@param	[in]	need_update	is required of conversion calculation
	///	@param	[out]	out_pnt		output Grid points
	/*------------------------------------------------------------------*/
	void convScreenGridCamera(bool need_update, std::vector<std::pair<Point3d, Point3d>>& out_pnt);
	/*------------------------------------------------------------------*/
	/// @brief	convert grid line to draw image position (world view)
	///	@param	[in]	need_update	is required of conversion calculation
	///	@param	[out]	out_pnt		output Grid point
	/*------------------------------------------------------------------*/
	void convScreenGridWorld(bool need_update, std::vector<std::pair<Point3d, Point3d>>& out_pnt);

	/*------------------------------------------------------------------*/
	/// @brief	Set field of view setting
	///	@param	[in]	horz	horizontal of FOV
	/*------------------------------------------------------------------*/
	void setFov(uint16_t horz);
	/*------------------------------------------------------------------*/
	/// @brief	Set range
	///	@param	[in]	max_range	max of range
	/*------------------------------------------------------------------*/
	void setRange(uint16_t max_range);
	/*------------------------------------------------------------------*/
	/// @brief	update grid line settings
	///	@param	[in]	aux_grid	Spacing of auxiliary lines
	///	@param	[in]	max_range	max of range
	/*------------------------------------------------------------------*/
	void updateGridSetting(void) { update_line_ = true; }
	void updateGridSetting(uint16_t aux_grid);
	void updateGridSetting(uint16_t aux_grid, uint16_t max_range);

	/*------------------------------------------------------------------*/
	/// @brief	set the interleaving parameters
	///	@param	[in]	enable		function enable flag
	///	@param	[in]	distance	upper distance to apply interleaving
	///	@param	[in]	factor		sampling factor (2 or 4)
	/*------------------------------------------------------------------*/
	void setInterleave(bool enable, float distance, uint8_t factor);

private:
	static const uint8_t		MATRIX4x4_SIZE = 16U;		/* 4x4 matrix size */
	static const uint8_t		MATRIX3x3_SIZE = 9U;		/* 3x3 matrix size */
	static const uint16_t		DEGREE_90 = 9000U;			/* 90 degree x 100 */
	static constexpr float		POS_TRANS_RATE = 600.0F;	/* position translation rate */
	static constexpr float		SCALE_RATE = 150.0F;		/* scaling rate */
	static constexpr float		EPS = 1.0E-20F;				/* epsilon value */
	static constexpr float		CLIP_NEAR = 10.0F;			/* view clip near */
	static constexpr float		VIEW_MARGIN = 0.09F;		/* horizontal margin for view */
	/*------------------------------------------------------------------*/
	/// @brief	structure of 3D point
	/*------------------------------------------------------------------*/
	struct DrawPcdPoint {
		float	x;		/* x */
		float	y;		/* y */
		float	z;		/* z */
	};

	bool						update_pnt_;	/* flag of point cloud date is update*/
	bool						update_line_;	/* flag of grid line is update*/
	bool						update_view_;	/* flag of viewing position is update*/

	std::shared_ptr<ColorTable>	color_table_;	/* color table for drawing */
	std::shared_ptr<GrayTable>	gray_table_;	/* gray table for drawing */
	PcdData						pcd_org_;		/* point cloud data */
	std::vector<Point3d>		pcd_draw_;		/* point cloud data */
	std::vector<Point3d>		pcd_screen_;	/* point cloud data */
	std::vector<Point3d>		pcd_filtered_;	/* point cloud data */
	bool						pcd_ir_;		/* drawing color is Grayscale(IR) */

	float						win_x_;			/* window x position */
	float						win_y_;			/* window y position */
	float						win_w_;			/* width of window */
	float						win_h_;			/* height of window */
	float						cz_;			/* center z position of viewing rotation */
	ViewKind					view_pos_kind_;	/* kind of view position */
	DrawPcdPoint				eye_;			/* viewpoint */
	DrawPcdPoint				look_at_;		/* target point */
	DrawPcdPoint				up_;			/* upward direction of viewpoint */
	float						drw_scale_;		/* drawing scale */

	float						clip_w_;		/* width of view clip size */
	float						clip_h_;		/* height of view clip size */
	float						clip_far_;		/* far range of view clip size */

	std::array<float, MATRIX4x4_SIZE>	proj_mat_;			/* projection matrix */
	std::array<float, MATRIX4x4_SIZE>	trans_mat_;			/* translation matrix */
	std::array<float, MATRIX4x4_SIZE>	trans_scaled_mat_;	/* translation matrix */

	uint16_t	grid_xz_max_;			/* max range of x-axis and z-axis for grid line */
	float		grid_fov_max_;			/* max range of x-axis/z-axis for FOV grid line */
	bool		is_over_90_degree_;		/* whether FOV is over 90 degree */
	double		grid_tan_half_fov_h_;	/* tangent value of FOV horizon */
	uint16_t	aux_grid_;				/* Spacing of auxiliary lines */

	bool		interleave_enable_;		/* enable interleaving */
	float		interleave_distance_;	/* upper distance to apply interleaving */
	uint8_t		interleave_factor_;		/* sampling factor of interleaving */

	std::vector<std::pair<Point3d, Point3d>>	grid_screen_;	/* grid data */

	/*------------------------------------------------------------------*/
	/// @copydoc	DrawImage::convImage
	/*------------------------------------------------------------------*/
	void convImage(const std::vector<uint16_t>& src_img) override;
	/*------------------------------------------------------------------*/
	/// @brief	Interleving Image
	///	@param	[in]	src			original image
	/*------------------------------------------------------------------*/
	void interleaveImage(const std::vector<Point3d>& src);
	/*------------------------------------------------------------------*/
	/// @brief	Validate Point Cloud Image
	///	@param	[in]	src			original image
	/*------------------------------------------------------------------*/
	void validateImage(const std::vector<Point3d>& src);

	/*------------------------------------------------------------------*/
	/// @brief	Create viewing transform matrix
	///	@param	[out]	matrix		viewing transform matrix
	/*------------------------------------------------------------------*/
	void lookAt(std::array<float, MATRIX4x4_SIZE>& matrix);
	/*------------------------------------------------------------------*/
	/// @brief	Create Orthogonal transform matrix
	///	@param	[in]	width		width of view volume
	///	@param	[in]	height		height of view volume
	///	@param	[in]	far			far point of view volume
	///	@param	[out]	matrix		Orthogonal transform matrix
	/*------------------------------------------------------------------*/
	void createOrtho(float width, float height, float far, std::array<float, MATRIX4x4_SIZE>& matrix);
	/*------------------------------------------------------------------*/
	/// @brief	Multiply matrices
	///	@param	[in]	mat1		input matrix(left)
	///	@param	[in]	mat2		input matrix(right)
	///	@param	[out]	out_mat		multiplied matrix
	/*------------------------------------------------------------------*/
	void multiMat(const std::array<float, MATRIX4x4_SIZE>& mat1,
					const std::array<float, MATRIX4x4_SIZE>& mat2, std::array<float, MATRIX4x4_SIZE>& out_mat);
	/*------------------------------------------------------------------*/
	/// @brief	Create rotation matrix for viewing operation
	///	@param	[in]	angle		rotation angle (radian)
	///	@param	[in]	axis		pivot axis
	///	@param	[out]	rot_mat		rotation matrix
	/*------------------------------------------------------------------*/
	void createRotMat(float angle, const DrawPcdPoint& axis, std::array<float, MATRIX3x3_SIZE>& rot_mat);
	/*------------------------------------------------------------------*/
	/// @brief	Update view range size
	/*------------------------------------------------------------------*/
	void updateViewSize(void);
	/*------------------------------------------------------------------*/
	/// @brief	Update matrices
	///	@param	[in]	is_only_scale	whether is changing only scale
	/*------------------------------------------------------------------*/
	void updateMatrix(bool is_only_scale = false);

	/*------------------------------------------------------------------*/
	/// @brief	convert to draw image position
	///	@param	[in]	world_pos	input point
	///	@return	drawing 2D point
	/*------------------------------------------------------------------*/
	Point3d convScreenPnt(Point3d world_pos);

	/*------------------------------------------------------------------*/
	/// @brief	get the interleaving parameters
	///	@param	[out]	enable		function enable flag
	///	@param	[out]	distance	upper distance to apply interleaving
	///	@param	[out]	factor		sampling factor (2 or 4)
	/*------------------------------------------------------------------*/
	void getInterleave(bool& enable, float& distance, uint8_t& factor);
};

} // namespace krm
