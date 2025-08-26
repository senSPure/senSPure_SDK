/*------------------------------------------------------------------*/
/// @file		sample_viewer_node.cpp
/// @brief		Sample Viewer Node
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "sample_viewer_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_example::SampleViewerNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
