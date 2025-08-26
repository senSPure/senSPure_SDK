/*------------------------------------------------------------------*/
/// @file		lens_conv_component.cpp
/// @brief		LensConv component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "lens_conv_component.hpp"

namespace tof_camera_node
{

LensConvNode::LensConvNode(
	const rclcpp::NodeOptions& options
): Node("LensConvNode", options),
	lens_conv_(),
	is_dist_corrected_(false), enable_distortion_(true),
	enable_pcd_(true), pcd_world_(false), pcd_color_(PCD_COLOR_NONE),
	image_size_(0)
{
	RCLCPP_INFO(this->get_logger(), "run LensConvNode");
	tmp_dp1_.data.clear();
	tmp_ir_.data.clear();
	tmp_pcd_.data.clear();

	dpt_fmt_.set();

	// output Point Cloud is Left-Hand
	fields_.resize(4);
	fields_[0].name = "color";
	fields_[0].offset = 0;
	fields_[0].datatype = sensor_msgs::msg::PointField::UINT32;
	fields_[0].count = 1;
	fields_[1].name = "x";
	fields_[1].offset = 4;
	fields_[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields_[1].count = 1;
	fields_[2].name = "y";
	fields_[2].offset = 8;
	fields_[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields_[2].count = 1;
	fields_[3].name = "z";
	fields_[3].offset = 12;
	fields_[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields_[3].count = 1;

	createTopics();
	loadParam();
}

LensConvNode::~LensConvNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit LensConvNode");
}

void LensConvNode::loadParam(void)
{
	krm::PosOrgRotation pos;

	declare_parameter("distortion", true);
	declare_parameter("pcd_origin", false);
	declare_parameter("pcd_offset_x", 0);
	declare_parameter("pcd_offset_y", 0);
	declare_parameter("pcd_offset_z", 0);
	declare_parameter("pcd_rotation_x", 0.0);
	declare_parameter("pcd_rotation_y", 0.0);
	declare_parameter("pcd_rotation_z", 0.0);

	get_parameter("distortion",     enable_distortion_);
	get_parameter("pcd_origin",     pcd_world_);
	get_parameter("pcd_offset_x",   pos.offset.x);
	get_parameter("pcd_offset_y",   pos.offset.y);
	get_parameter("pcd_offset_z",   pos.offset.z);
	get_parameter("pcd_rotation_x", pos.rotation.pitch);
	get_parameter("pcd_rotation_y", pos.rotation.yaw);
	get_parameter("pcd_rotation_z", pos.rotation.roll);

	(void)setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_CONV_DIST, enable_distortion_);
	(void)setLensConv(tof_camera_interface::srv::SetLensConv::Request::LENS_PCD_KIND , pcd_world_);
	setPcdPos(pos);
}

void LensConvNode::createTopics(void)
{
	rclcpp::QoS qos_image(rclcpp::KeepLast(5));
	rclcpp::QoS qos_event(rclcpp::KeepLast(5));

	auto cb_psbl     = std::bind(&LensConvNode::cbSrvPsblLensConv, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_lens     = std::bind(&LensConvNode::cbSrvSetLensConv , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_pcd_pos  = std::bind(&LensConvNode::cbSrvSetPcdPos   , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_pcd_colr = std::bind(&LensConvNode::cbSrvSetPcdColor , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_chg_prop = std::bind(&LensConvNode::recvChgProp      , this, std::placeholders::_1);
	auto cb_chg_fmt  = std::bind(&LensConvNode::recvChgFmt       , this, std::placeholders::_1);
	auto cb_frame    = std::bind(&LensConvNode::recvFrameData    , this, std::placeholders::_1);

	qos_image = qos_image.reliable().durability_volatile();
	qos_event = qos_event.reliable().durability_volatile();

	srv_psbl_lens_conv_ = create_service<tof_camera_interface::srv::PsblLensConv>("krm/psbl_lens_conv", cb_psbl);
	srv_set_lens_conv_  = create_service<tof_camera_interface::srv::SetLensConv >("krm/set_lens_conv" , cb_lens);
	srv_set_pcd_pos_    = create_service<tof_camera_interface::srv::SetPcdPos   >("krm/set_pcd_pos"   , cb_pcd_pos);
	srv_set_pcd_color_  = create_service<tof_camera_interface::srv::SetPcdColor >("krm/set_pcd_color" , cb_pcd_colr);

	sub_chg_prop_   = create_subscription<tof_camera_interface::msg::EventChgProp>("krm/event_chg_prop", qos_event, cb_chg_prop);
	sub_chg_fmt_    = create_subscription<tof_camera_interface::msg::EventChgFmt >("krm/event_chg_fmt" , qos_event, cb_chg_fmt);
	sub_frame_data_ = create_subscription<tof_camera_interface::msg::FrameData   >("krm/post_filt_out" , qos_image, cb_frame);

	pub_frame_data_ = create_publisher<tof_camera_interface::msg::FrameData>("krm/lens_out" , qos_image);
}

bool LensConvNode::setLensConv(uint8_t lens_conv_type, bool enable)
{
	switch (lens_conv_type) {
	case tof_camera_interface::srv::SetLensConv::Request::LENS_CONV_DIST:
		if (!is_dist_corrected_) {
			enable_distortion_ = enable;
			RCLCPP_INFO(this->get_logger(), "Distortion = %d", enable_distortion_);
		} else {
			RCLCPP_WARN(this->get_logger(), "Unable to enable dist correction");
			return false;
		}
		break;
	case tof_camera_interface::srv::SetLensConv::Request::LENS_PCD_KIND:
		pcd_world_ = enable;
		RCLCPP_INFO(this->get_logger(), "Point cloud kind = %d", pcd_world_);
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "Invalid LensConv Type : %d", lens_conv_type);
		return false;
	}
	return true;
}

void LensConvNode::setPcdPos(krm::PosOrgRotation& pos)
{
	lens_conv_.setPosOrgRotation(pos);

	RCLCPP_INFO(this->get_logger(), "offset.x = %d",       pos.offset.x);
	RCLCPP_INFO(this->get_logger(), "offset.y = %d",       pos.offset.y);
	RCLCPP_INFO(this->get_logger(), "offset.z = %d",       pos.offset.z);
	RCLCPP_INFO(this->get_logger(), "rotation.pitch = %f", pos.rotation.pitch);
	RCLCPP_INFO(this->get_logger(), "rotation.yaw   = %f", pos.rotation.yaw);
	RCLCPP_INFO(this->get_logger(), "rotation.roll  = %f", pos.rotation.roll);
}

void LensConvNode::cbSrvPsblLensConv(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::PsblLensConv::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::PsblLensConv::Response> response)
{
	response->result = true;
	switch (request->conv_type) {
	case tof_camera_interface::srv::PsblLensConv::Request::LENS_CONV_DIST:
		response->possible = !is_dist_corrected_;
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "Invalid Lens Conversion Type : %d", request->conv_type);
		break;
	}
}

void LensConvNode::cbSrvSetLensConv(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetLensConv::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetLensConv::Response> response)
{
	response->result = setLensConv(request->conv_type, request->enable);
}

void LensConvNode::cbSrvSetPcdPos(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPcdPos::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPcdPos::Response> response)
{
	krm::PosOrgRotation	pos;

	pos.offset.x       = request->pos.offset_x;
	pos.offset.y       = request->pos.offset_y;
	pos.offset.z       = request->pos.offset_z;
	pos.rotation.pitch = request->pos.rotation_pitch;
	pos.rotation.yaw   = request->pos.rotation_yaw;
	pos.rotation.roll  = request->pos.rotation_roll;

	setPcdPos(pos);
	response->result = true;
}

void LensConvNode::cbSrvSetPcdColor(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPcdColor::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPcdColor::Response> response)
{
	response->result = true;
	pcd_color_ = static_cast<PcdColorKind>(request->color);
}

void LensConvNode::recvChgProp(
	const tof_camera_interface::msg::EventChgProp::SharedPtr property)
{
	krm::Result ret;
	krm::LensInfo lens_info;
	krm::CamFov cam_fov;

	RCLCPP_INFO(this->get_logger(), "LensConv : Receive ChgProp");

	lens_info.sens_w = property->lens_info.sens_w;
	lens_info.sens_h = property->lens_info.sens_h;
	lens_info.focal_len = property->lens_info.focal_len;
	lens_info.thin_w = property->lens_info.thin_w;
	lens_info.thin_h = property->lens_info.thin_h;
	lens_info.crop.x = property->lens_info.crop.x;
	lens_info.crop.y = property->lens_info.crop.y;
	lens_info.cam_dist = property->lens_info.cam_dist;
	std::memcpy(lens_info.dist, property->lens_info.dist.data(), sizeof(lens_info.dist));
	lens_info.lens_calib = property->lens_info.lens_calib;

	cam_fov.horz = property->fov.horz;
	cam_fov.vert = property->fov.vert;

	ret = lens_conv_.setLensPrm(lens_info, cam_fov);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to setLensPrm : %d", ret);
	} else {
		is_dist_corrected_ = property->lens_info.cam_dist;
		if (is_dist_corrected_) { enable_distortion_ = false; }
	}
}

void LensConvNode::recvChgFmt(
	const tof_camera_interface::msg::EventChgFmt::SharedPtr formats)
{
	krm::Result ret = krm::SUCCESS;
	krm::ImageFormat dpt_fmt, ir_fmt;
	tof_camera_interface::msg::ImageFormat msg_fmt;

	RCLCPP_INFO(this->get_logger(), "LensConv : Receive ChgFmt");

	msg_fmt = formats->formats.data[krm::IMG_DEPTH];
	dpt_fmt.set(msg_fmt.width, msg_fmt.height, msg_fmt.bpp);
	msg_fmt = formats->formats.data[krm::IMG_IR];
	ir_fmt.set(msg_fmt.width, msg_fmt.height, msg_fmt.bpp);
	if (dpt_fmt.pixels > 0) {
		ret = lens_conv_.setFormat(dpt_fmt);
		image_size_ = dpt_fmt.size;
	} else if (ir_fmt.pixels > 0) {
		ret = lens_conv_.setFormat(ir_fmt);
		image_size_ = ir_fmt.size;
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to setFormat : %d", ret);
	} else {
		dpt_fmt_ = dpt_fmt;
		tmp_dp1_.resize(dpt_fmt);
		tmp_ir_.resize(ir_fmt);
		tmp_pcd_.resize(dpt_fmt);
	}
}

void LensConvNode::recvFrameData(
	tof_camera_interface::msg::FrameData::UniquePtr frame)
{
	krm::Result		ret = krm::SUCCESS;
	uint16_t*		depth;
	uint16_t*		ir;
	krm::Point3d*	pcd;
	uint32_t 		pixels = static_cast<uint32_t>(tmp_pcd_.data.size());
	bool			is_depth = (frame->depth.image.data.size() > 0);
	bool			is_ir = (frame->ir.image.data.size() > 0);
	krm::ConvState	con_sta_dpt, con_sta_ir;

	if (frame->stopped) {
		pub_frame_data_->publish(std::move(frame));
		return;
	}

	if (is_ir) {
		ir = reinterpret_cast<uint16_t*>(frame->ir.image.data.data());
		std::memcpy(&con_sta_ir, &frame->ir.info.conv_stat, sizeof(uint8_t));
		tmp_ir_.info.conv_stat = con_sta_ir;

		if ((enable_pcd_ && is_depth && (pcd_color_ == PCD_COLOR_IR)) || (enable_distortion_)) {
			if (!is_dist_corrected_) {
				ret = lens_conv_.correctDist(ir, false, tmp_ir_.data.data());
				con_sta_ir.is_crct_dist = 1U;
				if (ret != krm::SUCCESS) {
					RCLCPP_ERROR(this->get_logger(), "Failed to correctDist : %d", ret);
					return;
				}
			} else {
				std::memcpy(tmp_ir_.data.data(), ir, image_size_);
			}
		}
	}

	if (is_depth) {
		uint8_t* ros_conv_stat = &frame->depth.info.conv_stat;

		depth = reinterpret_cast<uint16_t*>(frame->depth.image.data.data());
		std::memcpy(&con_sta_dpt, ros_conv_stat, sizeof(uint8_t));
		tmp_dp1_.info.conv_stat = con_sta_dpt;

		if (enable_pcd_) {
			frame->pcd.header = frame->depth.image.header;
			frame->pcd.fields = fields_;
			frame->pcd.is_bigendian = 0;	// little
			frame->pcd.point_step = 16;		// float:4byte * 4(x,y,z,color)
			frame->pcd.is_dense = false;
			frame->pcd.width = dpt_fmt_.width;
			frame->pcd.height = dpt_fmt_.height;
			frame->pcd.row_step = frame->pcd.width * frame->pcd.point_step;
			frame->pcd.data.resize(frame->pcd.row_step * frame->pcd.height);
			pcd = reinterpret_cast<krm::Point3d*>(frame->pcd.data.data());
			if (!is_dist_corrected_) {
				ret = lens_conv_.correctDist(depth, true, tmp_dp1_.data.data());
				if (ret != krm::SUCCESS) {
					RCLCPP_ERROR(this->get_logger(), "Failed to correctDist : %d", ret);
					return;
				}
			} else {
				std::memcpy(tmp_dp1_.data.data(), depth, image_size_);
			}
			if (enable_distortion_) {
				std::memcpy(depth, tmp_dp1_.data.data(), image_size_);
				con_sta_dpt.is_crct_dist = 1U;
				std::memcpy(ros_conv_stat, &con_sta_dpt, sizeof(uint8_t));
			}
			if (pcd_world_) {
				ret = lens_conv_.convPcdWorld(tmp_dp1_.data.data(), pcd);
			} else {
				ret = lens_conv_.convPcdCamera(tmp_dp1_.data.data(), pcd);
			}
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to convPcd(%d) : %d", pcd_world_, ret);
				return;
			}
			switch (pcd_color_) {
			case PCD_COLOR_IR:
				if (dpt_fmt_.pixels == tmp_ir_.data.size()) {
					frame->pcd_kind = tof_camera_interface::msg::FrameData::PCD_IRXYZ;
					for (uint32_t i = 0; i < pixels; i++, pcd++) {
						if (pcd->color != UINT32_MAX) {
							pcd->color = tmp_ir_.data[i];
						}
					}
				}
			case PCD_COLOR_NONE:
			default:
				break;
			}
		} else if (enable_distortion_) {
			ret = lens_conv_.correctDist(depth, true, tmp_dp1_.data.data());
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to correctDist : %d", ret);
				return;
			}
			std::memcpy(depth, tmp_dp1_.data.data(), image_size_);
			con_sta_dpt.is_crct_dist = 1U;
			std::memcpy(ros_conv_stat, &con_sta_dpt, sizeof(uint8_t));
		}
	}

	if (is_ir && enable_distortion_) {
		std::memcpy(frame->ir.image.data.data(), tmp_ir_.data.data(), image_size_);
		std::memcpy(&frame->ir.info.conv_stat, &con_sta_ir, sizeof(uint8_t));
	}

	pub_frame_data_->publish(std::move(frame));
}

}	// namespace tof_camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_node::LensConvNode)
