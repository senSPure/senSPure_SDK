/*------------------------------------------------------------------*/
/// @file		record_node.cpp
/// @brief		Record node class
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "record_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_node::RecordNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
