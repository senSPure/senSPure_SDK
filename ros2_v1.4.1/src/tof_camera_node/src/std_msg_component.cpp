/*------------------------------------------------------------------*/
/// @file		std_msg_component.cpp
/// @brief		StdMsg component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "std_msg_component.hpp"

namespace tof_camera_node
{

StdMsgNode::StdMsgNode(
	const rclcpp::NodeOptions& options
): Node("StdMsgNode", options),
	cam_info_(), output_pcd_({}), counter_(0)
{
	std::vector<sensor_msgs::msg::PointField>	fields;

	RCLCPP_INFO(this->get_logger(), "run StdMsgNode");

	// output Point Cloud is Right-Hand(Z-up)
	fields.resize(3);
	fields[0].name = "x";
	fields[0].offset = 8;	// right-X = left-Z * -1
	fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields[0].count = 1;
	fields[1].name = "y";
	fields[1].offset = 0;	// right-Y = left-X
	fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields[1].count = 1;
	fields[2].name = "z";
	fields[2].offset = 4;	// right-Z = left-Y
	fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	fields[2].count = 1;

	output_pcd_.fields = fields;

	createTopics();
}

StdMsgNode::~StdMsgNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit StdMsgNode");
}

void StdMsgNode::createTopics(void)
{
	rclcpp::QoS qos_status(rclcpp::KeepLast(1));
	rclcpp::QoS qos_image(rclcpp::KeepLast(5));
	qos_status = qos_status.reliable().durability_volatile();
	qos_image = qos_image.reliable().durability_volatile();

	sub_event_chg_prop_ = create_subscription<tof_camera_interface::msg::EventChgProp>("krm/event_chg_prop", qos_status,
		std::bind(&StdMsgNode::recvEventChgProp, this, std::placeholders::_1));
	sub_event_chg_fmt_ = create_subscription<tof_camera_interface::msg::EventChgFmt>("krm/event_chg_fmt", qos_status,
		std::bind(&StdMsgNode::recvEventChgFmt, this, std::placeholders::_1));
	sub_frame_data_ = create_subscription<tof_camera_interface::msg::FrameData>("krm/lens_out", qos_image,
		std::bind(&StdMsgNode::recvFrameData, this, std::placeholders::_1));

	pub_cam_info_ = create_publisher<sensor_msgs::msg::CameraInfo>("krm/std/cam_info", qos_image);
	pub_depth_ = create_publisher<sensor_msgs::msg::Image>("krm/std/depth", qos_image);
	pub_ir_ = create_publisher<sensor_msgs::msg::Image>("krm/std/ir", qos_image);
	pub_raw1_ = create_publisher<sensor_msgs::msg::Image>("krm/std/raw1", qos_image);
	pub_raw2_ = create_publisher<sensor_msgs::msg::Image>("krm/std/raw2", qos_image);
	pub_raw3_ = create_publisher<sensor_msgs::msg::Image>("krm/std/raw3", qos_image);
	pub_raw4_ = create_publisher<sensor_msgs::msg::Image>("krm/std/raw4", qos_image);
	pub_pcd_ = create_publisher<sensor_msgs::msg::PointCloud2>("krm/std/pcd", qos_image);
}

double StdMsgNode::convFixed64Float64(uint64_t fixed_point)
{
	uint8_t i = 0U;
	double divisor = 0x1ULL << 47U;
	float floating_point = 0;
	uint16_t integer = static_cast<uint16_t>((fixed_point & 0x7FFF800000000000) >> 47U);	// Integer:16bit
	uint64_t decimal = (fixed_point & 0x00007FFFFFFFFFFFULL);								// Decimal:48bit
	for (i = 0U; i < 16U; i++) {
		floating_point += ((integer & 0x01) << i);
		integer >>= 1U;
	}
	for (i = 0U; i < 47U; i++) {
		floating_point += static_cast<float>((static_cast<float>(decimal & 0x00000001)) / divisor);
		decimal >>= 1U;
		divisor /= 2;
	}
	if ((fixed_point >> 63U) > 0) {
		floating_point *= -1;
	}

	return static_cast<double>(floating_point);
}

void StdMsgNode::recvEventChgProp(const tof_camera_interface::msg::EventChgProp::SharedPtr property)
{
	tof_camera_interface::msg::LensInfo lens_info = property->lens_info;

	cam_info_.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
	cam_info_.d.resize(5);
	std::fill(cam_info_.d.begin(), cam_info_.d.end(), 0);
	std::fill(cam_info_.k.begin(), cam_info_.k.end(), 0);
	std::fill(cam_info_.r.begin(), cam_info_.r.end(), 0);
	std::fill(cam_info_.p.begin(), cam_info_.p.end(), 0);
	cam_info_.k[0] = convFixed64Float64(lens_info.dist[0]);	// fx
	cam_info_.k[4] = convFixed64Float64(lens_info.dist[1]);	// fy
	cam_info_.k[2] = convFixed64Float64(lens_info.dist[2]);	// cx
	cam_info_.k[5] = convFixed64Float64(lens_info.dist[3]);	// cy
	cam_info_.k[8] = 1;
	cam_info_.d[0] = convFixed64Float64(lens_info.dist[4]);	// k1
	cam_info_.d[1] = convFixed64Float64(lens_info.dist[5]);	// k2
	cam_info_.d[2] = convFixed64Float64(lens_info.dist[6]);	// p1
	cam_info_.d[3] = convFixed64Float64(lens_info.dist[7]);	// p2
	cam_info_.d[4] = convFixed64Float64(lens_info.dist[8]);	// k3
	cam_info_.r[0] = 1;
	cam_info_.r[4] = 1;
	cam_info_.r[8] = 1;
	cam_info_.p[0] = 1;	// fx'
	cam_info_.p[5] = 1;	// fy'
	cam_info_.p[2] = 1;	// cx'
	cam_info_.p[6] = 1;	// cy'
	cam_info_.p[3] = 0;	// tx
	cam_info_.p[7] = 0;	// ty
	cam_info_.p[10] = 1;
	cam_info_.binning_x = 1;
	cam_info_.binning_y = 1;
	cam_info_.roi.x_offset = 0;
	cam_info_.roi.y_offset = 0;
	cam_info_.roi.height = cam_info_.height;
	cam_info_.roi.width = cam_info_.width;
	cam_info_.roi.do_rectify = false;
}

void StdMsgNode::recvEventChgFmt(const tof_camera_interface::msg::EventChgFmt::SharedPtr image_formats)
{
	tof_camera_interface::msg::ImageFormat* img_fmt = &image_formats->formats.data[krm::IMG_DEPTH];

	cam_info_.height = img_fmt->height;
	cam_info_.width = img_fmt->width;

	output_pcd_.is_bigendian = 0;	// little
	output_pcd_.point_step = 12;	// float:4byte * 3(x,y,z)
	output_pcd_.is_dense = false;
	output_pcd_.width = img_fmt->pixels;
	output_pcd_.height = 1;
	output_pcd_.row_step = output_pcd_.width * output_pcd_.point_step;
}

void StdMsgNode::recvFrameData(
	tof_camera_interface::msg::FrameData::UniquePtr frame)
{
	struct PointPtr {
		uint32_t	c;
		float		x;
		float		y;
		float		z;
	};

	PointPtr*	src_ptr;
	float*		dst_ptr;
	uint32_t	size = 0;

	tof_camera_interface::msg::FrameData::UniquePtr cur_frame = std::move(frame);
	std::unique_ptr<sensor_msgs::msg::CameraInfo> cam_info = std::make_unique<sensor_msgs::msg::CameraInfo>();
	std::unique_ptr<sensor_msgs::msg::Image> depth = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::Image> ir = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::Image> raw1 = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::Image> raw2 = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::Image> raw3 = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::Image> raw4 = std::make_unique<sensor_msgs::msg::Image>();
	std::unique_ptr<sensor_msgs::msg::PointCloud2> pcd = std::make_unique<sensor_msgs::msg::PointCloud2>();
	std::unique_ptr<sensor_msgs::msg::PointCloud2> out_pcd = std::make_unique<sensor_msgs::msg::PointCloud2>();

	if (cur_frame->stopped) {
		return;
	}

	*cam_info = cam_info_;
	*depth = cur_frame->depth.image;
	*ir = cur_frame->ir.image;
	*raw1 = cur_frame->raw1.image;
	*raw2 = cur_frame->raw2.image;
	*raw3 = cur_frame->raw3.image;
	*raw4 = cur_frame->raw4.image;

	*pcd = cur_frame->pcd;
	*out_pcd = output_pcd_;

	out_pcd->data.resize(out_pcd->row_step);
	src_ptr = reinterpret_cast<PointPtr*>(pcd->data.data());
	dst_ptr = reinterpret_cast<float*>(out_pcd->data.data());
	// convert [mm] -> [m] for rviz2
	for (uint32_t i = 0; i < out_pcd->width; i++) {
		if (src_ptr->c != UINT32_MAX) {
			*(dst_ptr++) = src_ptr->x / 1000.f;
			*(dst_ptr++) = src_ptr->y / 1000.f;
			*(dst_ptr++) = src_ptr->z / -1000.f;
			size++;
		}
		src_ptr++;
	}
	out_pcd->header = depth->header;
	out_pcd->width = size;
	out_pcd->row_step = out_pcd->width * out_pcd->point_step;
	out_pcd->data.resize(out_pcd->row_step);

	// Light count log for AE function
	// Enable if necessary
	// The display interval can be changed using the condition of the if statement.
	// ex. if ((counter_ % 30U) == 0) : Displays the current number of flashes once every "30" frames
#if 0
	if ((counter_ % 30U) == 0) {
		RCLCPP_INFO(this->get_logger(), "current light times = %u", cur_frame->depth.info.light_cnt);
		counter_ = 0;
	}
	counter_++;
#endif

	cam_info_.header = depth->header;
	pub_cam_info_->publish(std::move(cam_info));
	if ((depth->width != 0) && (depth->height != 0)) {
		pub_depth_->publish(std::move(depth));
	}
	if ((ir->width != 0) && (ir->height != 0)) {
		pub_ir_->publish(std::move(ir));
	}
	if ((raw1->width != 0) && (raw1->height != 0)) {
		pub_raw1_->publish(std::move(raw1));
	}
	if ((raw2->width != 0) && (raw2->height != 0)) {
		pub_raw2_->publish(std::move(raw2));
	}
	if ((raw3->width != 0) && (raw3->height != 0)) {
		pub_raw3_->publish(std::move(raw3));
	}
	if ((raw4->width != 0) && (raw4->height != 0)) {
		pub_raw4_->publish(std::move(raw4));
	}
	if (out_pcd->width != 0) {
		pub_pcd_->publish(std::move(out_pcd));
	}
}

}	// namespace tof_camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_node::StdMsgNode)
