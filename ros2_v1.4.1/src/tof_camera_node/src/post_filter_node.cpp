/*------------------------------------------------------------------*/
/// @file		post_filter_node.cpp
/// @brief		PostFilter Node
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "post_filter_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_node::PostFilterNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
