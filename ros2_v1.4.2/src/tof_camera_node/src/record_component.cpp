/*------------------------------------------------------------------*/
/// @file		record_component.cpp
/// @brief		Record component class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "record_component.hpp"

namespace tof_camera_node
{

RecordNode::RecordNode(
	const rclcpp::NodeOptions& options
): Node("RecordNode", options),
	record_(), device_info_({}), lens_info_({}), cam_fov_({}),
	post_filt_({}), mode_(0), frame_({}),
	recording_(false)
{
	RCLCPP_INFO(this->get_logger(), "run RecordNode");

	createTopics();

	for (auto& fmt : image_formats_) {
		fmt.set();
	}
}

RecordNode::~RecordNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit RecordNode");
}

void RecordNode::createTopics(void)
{
	rclcpp::QoS qos_status(rclcpp::KeepLast(1));
	rclcpp::QoS qos_image(rclcpp::KeepLast(5));
	rclcpp::QoS qos_event(rclcpp::KeepLast(5));

	auto cb_ctrl     = std::bind(&RecordNode::cbSrvRecordCtrl , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_chg_prop = std::bind(&RecordNode::recvChgProp     , this, std::placeholders::_1);
	auto cb_chg_fmt  = std::bind(&RecordNode::recvChgFmt      , this, std::placeholders::_1);
	auto cb_frame    = std::bind(&RecordNode::recvFrameData   , this, std::placeholders::_1);

	qos_status = qos_status.reliable().durability_volatile();
	qos_image = qos_image.reliable().durability_volatile();
	qos_event = qos_event.reliable().durability_volatile();

	srv_record_ctrl_ = create_service<tof_camera_interface::srv::RecordCtrl>("krm/record_ctrl", cb_ctrl);

	sub_chg_prop_   = create_subscription<tof_camera_interface::msg::EventChgProp>("krm/event_chg_prop", qos_event, cb_chg_prop);
	sub_chg_fmt_    = create_subscription<tof_camera_interface::msg::EventChgFmt >("krm/event_chg_fmt" , qos_event, cb_chg_fmt);
	sub_frame_data_ = create_subscription<tof_camera_interface::msg::FrameData   >("krm/lens_out"      , qos_image, cb_frame);

	pub_notify_     = create_publisher<tof_camera_interface::msg::Notify>("krm/notify"         , qos_status);
}

void RecordNode::cbSrvRecordCtrl(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Control Command(%u)", request->cmd);
	response->result = false;

	switch (request->cmd) {
	case tof_camera_interface::srv::RecordCtrl::Request::CMD_REC_START:
		response->result = startRecord(request);
		break;
	case tof_camera_interface::srv::RecordCtrl::Request::CMD_REC_STOP:
		response->result = stopRecord();
		break;
	default:
		break;
	}
}

bool RecordNode::startRecord(const std::shared_ptr<tof_camera_interface::srv::RecordCtrl::Request> param)
{
	krm::Result ret;
	krm::Record::RecInfoParam rec_info;

	RCLCPP_INFO(this->get_logger(), "Record: %s %d %d", param->directory.c_str(), param->save_frames, param->packing_frames);

	rec_info.path = param->directory;
	rec_info.save_frames = param->save_frames;
	rec_info.packing_frames = param->packing_frames;
	rec_info.device_info = device_info_;
	rec_info.lens_info = lens_info_;
	rec_info.fov = cam_fov_;
	rec_info.mode_info = mode_info_;
	rec_info.image_formats = image_formats_;

	rec_info.is_crct_dist	= param->is_crct_dist;
	rec_info.is_filt_med	= param->is_filt_med;
	rec_info.is_filt_bil	= param->is_filt_bil;
	rec_info.is_filt_fly_p	= param->is_filt_fly_p;

	ret = record_.openRec(rec_info);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "failed to open Record : %d", ret);
		return false;
	}
	for (uint8_t i = 0; i < static_cast<uint8_t>(krm::IMG_KINDS); i++) {
		frame_.images[i].resize(image_formats_[i]);
	}

	recording_ = true;
	return true;
}

bool RecordNode::stopRecord(void)
{
	if (recording_) {
		krm::Result ret;

		recording_ = false;
		ret = record_.closeRec();
		if (ret != krm::SUCCESS) {
			RCLCPP_ERROR(this->get_logger(), "failed to close Record : %d", ret);
			return false;
		}
	} else {
		RCLCPP_WARN(this->get_logger(), "already stopped");
	}
	return true;
}

void RecordNode::recvChgProp(
	const tof_camera_interface::msg::EventChgProp::SharedPtr property)
{
	RCLCPP_INFO(this->get_logger(), "Record : Receive ChgProp");

	convertDeviceInfo(property->dev_info);
	convertModeInfo(property->mode_info);
	convertLensInfo(property->lens_info);
	convertCamFov(property->fov);
	convertPostFiltInfo(property->post_filt_info);
}

void RecordNode::recvChgFmt(
	const tof_camera_interface::msg::EventChgFmt::SharedPtr formats)
{
	krm::ImageFormat* dst;
	tof_camera_interface::msg::ImageFormat* src;

	RCLCPP_INFO(this->get_logger(), "Record : Receive ChgFmt");
	if (formats->formats.data.size() != image_formats_.size()) {
		RCLCPP_ERROR(this->get_logger(), "Invalid formats size : %ld", formats->formats.data.size());
	}

	for (uint8_t i = 0; i < image_formats_.size(); i++) {
		dst = &image_formats_[i];
		src = &formats->formats.data[i];
		dst->width          = src->width;
		dst->height         = src->height;
		dst->active_start.x = src->active_start.x;
		dst->active_start.y = src->active_start.y;
		dst->active_w       = src->active_w;
		dst->active_h       = src->active_h;
		dst->pixels         = src->pixels;
		dst->bpp            = src->bpp;
		dst->size           = static_cast<size_t>(src->size);
	}
}

void RecordNode::recvFrameData(
	tof_camera_interface::msg::FrameData::UniquePtr frame)
{
	krm::Result ret = krm::SUCCESS;
	tof_camera_interface::msg::FrameImage* msg_data[krm::IMG_KINDS] = {&frame->depth, &frame->ir, &frame->raw1, &frame->raw2, &frame->raw3, &frame->raw4};
	tof_camera_interface::msg::FrameImage* src;
	krm::ImageData* dst;

	tof_camera_interface::msg::Notify notify;

	if (frame->stopped) {
		return;
	}

	for (uint8_t i = 0; i < frame_.images.size(); i++) {
		src = msg_data[i];
		dst = &frame_.images[i];
		dst->data.resize(src->image.height * src->image.width);
		dst->info.number       = src->info.number;
		dst->info.time.tv_sec  = src->image.header.stamp.sec;
		dst->info.time.tv_nsec = src->image.header.stamp.nanosec;
		dst->info.frm_err      = *reinterpret_cast<krm::FrameError*>(&src->info.frm_err);
		dst->info.temperature  = src->info.temperature;
		dst->info.light_cnt    = src->info.light_cnt;
		dst->info.conv_stat    = *reinterpret_cast<krm::ConvState*>(&src->info.conv_stat);
		std::memcpy(dst->data.data(), src->image.data.data(), src->image.data.size());
	}

	if (recording_) {
		ret = record_.recFrame(frame_);
		if (ret != krm::SUCCESS) {
			if (ret == krm::REACH_EOF) {
				RCLCPP_INFO(this->get_logger(), "End of Recording");
				notify.notify = tof_camera_interface::msg::Notify::REC_REACHED_EOF;
			} else {
				RCLCPP_ERROR(this->get_logger(), "failed to record frame : %d", ret);
				notify.notify = tof_camera_interface::msg::Notify::REC_ERR_SYSTEM;
			}
			pub_notify_->publish(notify);
			recording_ = false;
			(void)record_.closeRec();
		}
	}
}

void RecordNode::convertDeviceInfo(tof_camera_interface::msg::DeviceInfo& dev_info)
{
	device_info_.hw_kind        = dev_info.hw_kind;
	device_info_.serial_no      = dev_info.serial_no;
	device_info_.map_ver.major  = dev_info.map_ver.major;
	device_info_.map_ver.minor  = dev_info.map_ver.minor;
	device_info_.map_ver.rev    = dev_info.map_ver.rev;
	device_info_.firm_ver.major = dev_info.firm_ver.major;
	device_info_.firm_ver.minor = dev_info.firm_ver.minor;
	device_info_.firm_ver.rev   = dev_info.firm_ver.rev;
	device_info_.adjust_no      = dev_info.adjust_no;
	device_info_.ld_wave        = dev_info.ld_wave;
	device_info_.ld_enable      = dev_info.ld_enable;
	device_info_.correct_calib  = dev_info.correct_calib;
}

void RecordNode::convertModeInfo(tof_camera_interface::msg::ModeInfo& mode_info)
{
	size_t size = mode_info.img_out.size();

	mode_info_.id             = mode_info.id;
	mode_info_.description    = mode_info.description;
	mode_info_.img_out.clear();
	mode_info_.img_out.resize(size);
	for (uint8_t i = 0; i < size; i++) {
		mode_info_.img_out[i] = static_cast<krm::ImgOutKind>(mode_info.img_out[i].img_out_kind);
	}
	mode_info_.dist_range.min = mode_info.dist_range.min;
	mode_info_.dist_range.max = mode_info.dist_range.max;
	mode_info_.fps            = mode_info.fps;
	mode_info_.thin_w         = mode_info.thin_w;
	mode_info_.thin_h         = mode_info.thin_h;
	mode_info_.crop.x         = mode_info.crop.x;
	mode_info_.crop.y         = mode_info.crop.y;
	mode_info_.light_times    = mode_info.light_times;
	mode_info_.range_calib    = mode_info.range_calib;
}

void RecordNode::convertLensInfo(tof_camera_interface::msg::LensInfo& lens_info)
{
	uint8_t dist_size = static_cast<uint8_t>(sizeof(lens_info.dist) / sizeof(lens_info.dist[0]));

	lens_info_.sens_w     = lens_info.sens_w;
	lens_info_.sens_h     = lens_info.sens_h;
	lens_info_.focal_len  = lens_info.focal_len;
	lens_info_.thin_w     = lens_info.thin_w;
	lens_info_.thin_h     = lens_info.thin_h;
	lens_info_.crop.x     = lens_info.crop.x;
	lens_info_.crop.y     = lens_info.crop.y;
	lens_info_.cam_dist   = lens_info.cam_dist;
	for (uint8_t i = 0; i < dist_size; i++) {
		lens_info_.dist[i] = lens_info.dist[i];
	}
	lens_info_.lens_calib = lens_info.lens_calib;
}

void RecordNode::convertCamFov(tof_camera_interface::msg::CamFov& fov)
{
	cam_fov_.horz = fov.horz;
	cam_fov_.vert = fov.vert;
}

void RecordNode::convertPostFiltInfo(tof_camera_interface::msg::PostFiltInfo& info)
{
	post_filt_.cam_med_filt		= info.cam_med_filt;
	post_filt_.cam_bil_filt		= info.cam_bil_filt;
	post_filt_.cam_fly_p_filt	= info.cam_fly_p_filt;
}

}	// namespace tof_camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_node::RecordNode)
