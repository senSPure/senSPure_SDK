/*------------------------------------------------------------------*/
/// @file		lens_conv_node.cpp
/// @brief		LensConv Node
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "lens_conv_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_node::LensConvNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
