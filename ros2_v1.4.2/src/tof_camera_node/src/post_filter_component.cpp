/*------------------------------------------------------------------*/
/// @file		post_filter_component.cpp
/// @brief		PostFilter component class
/*------------------------------------------------------------------*/

#include <cstring>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"

#include "post_filter_component.hpp"

namespace tof_camera_node
{

PostFilterNode::PostFilterNode(
	const rclcpp::NodeOptions& options
): Node("PostFilterNode", options),
	post_filter_(),
	is_filt_med_(false), is_filt_bil_(false), is_filt_fly_p_(false),
	enable_medf_(true), enable_bilf_(true), enable_flypf_(true), image_size_(0)
{
	RCLCPP_INFO(this->get_logger(), "run PostFilterNode");
	dp_tmp1_.data.clear();
	ir_tmp1_.data.clear();

	createTopics();
	loadParam();
}

PostFilterNode::~PostFilterNode(void)
{
	RCLCPP_INFO(this->get_logger(), "exit PostFilterNode");
}

void PostFilterNode::loadParam(void)
{
	krm::PostFilterPrm param;

	declare_parameter("median", true);
	declare_parameter("bil", true);
	declare_parameter("flyp", true);
	declare_parameter("median_ksize", 3);
	declare_parameter("bil_ksize", 3);
	declare_parameter("bil_sigma_depth", 500.0);
	declare_parameter("bil_sigma_ir", 100.0);
	declare_parameter("bil_sigma_space", 1.0);
	declare_parameter("flyp_ksize", 3);
	declare_parameter("flyp_log", true);
	declare_parameter("flyp_thr", 110);
	declare_parameter("flyp_fast_proc", true);

	get_parameter("median",			enable_medf_);
	get_parameter("bil",			enable_bilf_);
	get_parameter("flyp",			enable_flypf_);
	get_parameter("median_ksize",	param.median_ksize);
	get_parameter("bil_ksize",		param.bil_ksize);
	get_parameter("bil_sigma_depth",param.bil_sigma_depth);
	get_parameter("bil_sigma_ir",	param.bil_sigma_ir);
	get_parameter("bil_sigma_space",param.bil_sigma_space);
	get_parameter("flyp_ksize",		param.flyp_ksize);
	get_parameter("flyp_log",		param.flyp_log);
	get_parameter("flyp_thr",		param.flyp_thr);
	get_parameter("flyp_fast_proc",	param.flyp_fast_proc);

	(void)setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF, enable_medf_);
	(void)setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF, enable_bilf_);
	(void)setPostFilt(tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF, enable_flypf_);
	(void)setPostFiltPrm(param);
}

void PostFilterNode::createTopics(void)
{
	rclcpp::QoS qos_image(rclcpp::KeepLast(5));
	rclcpp::QoS qos_event(rclcpp::KeepLast(5));

	auto cb_psbl     = std::bind(&PostFilterNode::cbSrvPsblPostFilt   , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_post     = std::bind(&PostFilterNode::cbSrvSetPostFilt    , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_prm      = std::bind(&PostFilterNode::cbSrvSetPostFiltPrm , this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
	auto cb_chg_prop = std::bind(&PostFilterNode::recvChgProp         , this, std::placeholders::_1);
	auto cb_chg_fmt  = std::bind(&PostFilterNode::recvChgFmt          , this, std::placeholders::_1);
	auto cb_frame    = std::bind(&PostFilterNode::recvFrameData       , this, std::placeholders::_1);

	qos_image = qos_image.reliable().durability_volatile();
	qos_event = qos_event.reliable().durability_volatile();

	srv_psbl_post_filt_    = create_service<tof_camera_interface::srv::PsblPostFilt  >("krm/psbl_post_filt", cb_psbl);
	srv_set_post_filt_     = create_service<tof_camera_interface::srv::SetPostFilt   >("krm/set_post_filt",  cb_post);
	srv_set_post_filt_prm_ = create_service<tof_camera_interface::srv::SetPostFiltPrm>("krm/set_post_filt_prm", cb_prm);

	sub_chg_prop_   = create_subscription<tof_camera_interface::msg::EventChgProp>("krm/event_chg_prop", qos_event, cb_chg_prop);
	sub_chg_fmt_    = create_subscription<tof_camera_interface::msg::EventChgFmt >("krm/event_chg_fmt" , qos_event, cb_chg_fmt);
	sub_frame_data_ = create_subscription<tof_camera_interface::msg::FrameData   >("krm/tof_out"       , qos_image, cb_frame);

	pub_frame_data_ = create_publisher<tof_camera_interface::msg::FrameData >("krm/post_filt_out" , qos_image);
}

bool PostFilterNode::setPostFilt(uint8_t filt_type, bool enable)
{
	switch (filt_type) {
	case tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_MEDF:
		if (!is_filt_med_) {
			enable_medf_ = enable;
			RCLCPP_INFO(this->get_logger(), "Median = %d", enable_medf_);
		} else {
			RCLCPP_WARN(this->get_logger(), "Unable to enable median filter");
			return false;
		}
		break;
	case tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_BILF:
		if (!is_filt_bil_) {
			enable_bilf_ = enable;
			RCLCPP_INFO(this->get_logger(), "Bilateral = %d", enable_bilf_);
		}
		else {
			RCLCPP_WARN(this->get_logger(), "Unable to enable bilateral filter");
			return false;
		}
		break;
	case tof_camera_interface::srv::SetPostFilt::Request::POST_FILT_FLYF:
		if (!is_filt_fly_p_) {
			enable_flypf_ = enable;
			RCLCPP_INFO(this->get_logger(), "FlyingPixel = %d", enable_flypf_);
		} else {
			RCLCPP_WARN(this->get_logger(), "Unable to enable flying pixel filter");
			return false;
		}
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "Invalid Post Filter Type : %d", filt_type);
		return false;
	}
	RCLCPP_INFO(this->get_logger(), "median = %d",		enable_medf_);
	RCLCPP_INFO(this->get_logger(), "bilateral = %d",	enable_bilf_);
	RCLCPP_INFO(this->get_logger(), "flyp = %d",		enable_flypf_);
	return true;
}

bool PostFilterNode::setPostFiltPrm(krm::PostFilterPrm& prm)
{
	krm::Result ret = post_filter_.setPostFilterPrm(prm);
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to setPostFilterPrm : %d", ret);
		return false;
	}

	RCLCPP_INFO(this->get_logger(), "median_ksize = %d",	prm.median_ksize);
	RCLCPP_INFO(this->get_logger(), "bil_ksize = %d",		prm.bil_ksize);
	RCLCPP_INFO(this->get_logger(), "bil_sigma_depth = %f",	prm.bil_sigma_depth);
	RCLCPP_INFO(this->get_logger(), "bil_sigma_ir = %f",	prm.bil_sigma_ir);
	RCLCPP_INFO(this->get_logger(), "bil_sigma_space = %f", prm.bil_sigma_space);
	RCLCPP_INFO(this->get_logger(), "flyp_ksize = %d",		prm.flyp_ksize);
	RCLCPP_INFO(this->get_logger(), "flyp_log = %d",		prm.flyp_log);
	RCLCPP_INFO(this->get_logger(), "flyp_thr = %d",		prm.flyp_thr);
	RCLCPP_INFO(this->get_logger(), "flyp_fast_proc = %d",	prm.flyp_fast_proc);

	return true;
}

void PostFilterNode::cbSrvPsblPostFilt(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::PsblPostFilt::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::PsblPostFilt::Response> response)
{
	response->result = false;
	switch (request->filt_type) {
	case tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_MEDF:
		response->result = true;
		response->possible = !is_filt_med_;
		break;
	case tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_BILF:
		response->result = true;
		response->possible = !is_filt_bil_;
		break;
	case tof_camera_interface::srv::PsblPostFilt::Request::POST_FILT_FLYF:
		response->result = true;
		response->possible = !is_filt_fly_p_;
		break;
	default:
		RCLCPP_ERROR(this->get_logger(), "Invalid Post Filter Type : %d", request->filt_type);
		break;
	}
}

void PostFilterNode::cbSrvSetPostFilt(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPostFilt::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPostFilt::Response> response)
{
	response->result = setPostFilt(request->filt_type, request->enable);
}

void PostFilterNode::cbSrvSetPostFiltPrm(
	const std::shared_ptr<rmw_request_id_t> /*request_header*/,
	const std::shared_ptr<tof_camera_interface::srv::SetPostFiltPrm::Request> request,
	const std::shared_ptr<tof_camera_interface::srv::SetPostFiltPrm::Response> response)
{
	krm::PostFilterPrm	prm;

	prm.median_ksize	= request->param.median_ksize;
	prm.bil_ksize		= request->param.bil_ksize;
	prm.bil_sigma_depth	= request->param.bil_sigma_depth;
	prm.bil_sigma_ir	= request->param.bil_sigma_ir;
	prm.bil_sigma_space = request->param.bil_sigma_space;
	prm.flyp_ksize		= request->param.flyp_ksize;
	prm.flyp_log		= request->param.flyp_log;
	prm.flyp_thr		= request->param.flyp_thr;
	prm.flyp_fast_proc	= request->param.flyp_fast_proc;

	response->result = setPostFiltPrm(prm);
}

void PostFilterNode::recvChgProp(
	const tof_camera_interface::msg::EventChgProp::SharedPtr property)
{
	RCLCPP_INFO(this->get_logger(), "PostFilt : Receive ChgProp");

	is_filt_med_ = property->post_filt_info.cam_med_filt;
	if (is_filt_med_) { enable_medf_ = false; }
	is_filt_bil_ = property->post_filt_info.cam_bil_filt;
	if (is_filt_bil_) { enable_bilf_ = false; }
	is_filt_fly_p_ = property->post_filt_info.cam_fly_p_filt;
	if (is_filt_fly_p_) { enable_flypf_ = false; }
}

void PostFilterNode::recvChgFmt(
	const tof_camera_interface::msg::EventChgFmt::SharedPtr formats)
{
	krm::Result ret = krm::SUCCESS;
	krm::ImageFormat dpt_fmt, ir_fmt;
	tof_camera_interface::msg::ImageFormat msg_fmt;

	RCLCPP_INFO(this->get_logger(), "PostFilt : Receive ChgFmt");

	msg_fmt = formats->formats.data[krm::IMG_DEPTH];
	dpt_fmt.set(msg_fmt.width, msg_fmt.height, msg_fmt.bpp);
	msg_fmt = formats->formats.data[krm::IMG_IR];
	ir_fmt.set(msg_fmt.width, msg_fmt.height, msg_fmt.bpp);
	if (dpt_fmt.pixels > 0) {
		RCLCPP_INFO(this->get_logger(), "width = %d",  dpt_fmt.width);
		RCLCPP_INFO(this->get_logger(), "height = %d", dpt_fmt.height);
		RCLCPP_INFO(this->get_logger(), "bpp = %d",    dpt_fmt.bpp);
		ret = post_filter_.setFormat(dpt_fmt);
		image_size_ = dpt_fmt.size;
	} else if (ir_fmt.pixels > 0) {
		RCLCPP_INFO(this->get_logger(), "width = %d",  ir_fmt.width);
		RCLCPP_INFO(this->get_logger(), "height = %d", ir_fmt.height);
		RCLCPP_INFO(this->get_logger(), "bpp = %d",    ir_fmt.bpp);
		ret = post_filter_.setFormat(ir_fmt);
		image_size_ = ir_fmt.size;
	}
	if (ret != krm::SUCCESS) {
		RCLCPP_ERROR(this->get_logger(), "Failed to set Format : %d", ret);
	} else {
		dp_tmp1_.resize(dpt_fmt);
		ir_tmp1_.resize(ir_fmt);
	}
}

void PostFilterNode::recvFrameData(
	tof_camera_interface::msg::FrameData::UniquePtr frame)
{
	krm::Result	ret;
	bool is_depth = (frame->depth.image.data.size() > 0);
	bool is_ir = (frame->ir.image.data.size() > 0);
	bool can_filt_med = enable_medf_ && !is_filt_med_;
	bool can_filt_bil = enable_bilf_ && !is_filt_bil_;
	bool can_filt_fly = enable_flypf_ && !is_filt_fly_p_;

	if (frame->stopped) {
		pub_frame_data_->publish(std::move(frame));
		return;
	}

	/* depth image processing */
	if (is_depth && (can_filt_med || can_filt_bil || can_filt_fly)) {
		auto& ros_time = frame->depth.image.header.stamp;
		auto& ros_info = frame->depth.info;
		auto& tmp1_info = dp_tmp1_.info;

		tmp1_info.number		= ros_info.number;
		tmp1_info.time.tv_sec	= ros_time.sec;
		tmp1_info.time.tv_nsec	= ros_time.nanosec;
		tmp1_info.frm_err		= *reinterpret_cast<krm::FrameError*>(&ros_info.frm_err);
		tmp1_info.temperature	= ros_info.temperature;
		tmp1_info.light_cnt		= ros_info.light_cnt;
		tmp1_info.conv_stat		= *reinterpret_cast<krm::ConvState*>(&ros_info.conv_stat);
		std::memcpy(dp_tmp1_.data.data(), frame->depth.image.data.data(), image_size_);
		if (can_filt_med) {
			ret = post_filter_.filterMedian(dp_tmp1_, dp_tmp1_);
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to filterMedian : %d", ret);
			}
		} 
		if (can_filt_bil) {
			ret = post_filter_.filterBilateral(dp_tmp1_, dp_tmp1_, true);
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to filterBilateral : %d", ret);
			}
		}
		if (can_filt_fly) {
			ret = post_filter_.filterFlyingPixel(dp_tmp1_, dp_tmp1_);
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to filterFlyingPixel : %d", ret);
			}
		}
		/* write the result image back into the frame */
		std::memcpy(frame->depth.image.data.data(), dp_tmp1_.data.data(), image_size_);
		ros_info.conv_stat = *reinterpret_cast<const uint8_t*>(&tmp1_info.conv_stat);
	}

	/* ir image processing */
	if (is_ir && (can_filt_med || can_filt_bil)) {
		auto& ros_time = frame->ir.image.header.stamp;
		auto& ros_info = frame->ir.info;
		auto& tmp1_info = ir_tmp1_.info;

		tmp1_info.number		= ros_info.number;
		tmp1_info.time.tv_sec	= ros_time.sec;
		tmp1_info.time.tv_nsec	= ros_time.nanosec;
		tmp1_info.frm_err		= *reinterpret_cast<krm::FrameError*>(&ros_info.frm_err);
		tmp1_info.temperature	= ros_info.temperature;
		tmp1_info.light_cnt		= ros_info.light_cnt;
		tmp1_info.conv_stat		= *reinterpret_cast<krm::ConvState*>(&ros_info.conv_stat);
		std::memcpy(ir_tmp1_.data.data(), frame->ir.image.data.data(), image_size_);
		if (can_filt_med) {
			ret = post_filter_.filterMedian(ir_tmp1_, ir_tmp1_);
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to filterMedian : %d", ret);
			}
		}
		if (can_filt_bil) {
			ret = post_filter_.filterBilateral(ir_tmp1_, ir_tmp1_, false);
			if (ret != krm::SUCCESS) {
				RCLCPP_ERROR(this->get_logger(), "Failed to filterBilateral : %d", ret);
			}
		}
		/* write the result image back into the frame */
		std::memcpy(frame->ir.image.data.data(), ir_tmp1_.data.data(), image_size_);
		ros_info.conv_stat = *reinterpret_cast<const uint8_t*>(&tmp1_info.conv_stat);
	}

	/* publish the image out */
	pub_frame_data_->publish(std::move(frame));
}

}	// namespace tof_camera_node

RCLCPP_COMPONENTS_REGISTER_NODE(tof_camera_node::PostFilterNode)
