/*------------------------------------------------------------------*/
/// @file		std_msg_node.cpp
/// @brief		StdMsg Node
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "std_msg_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_node::StdMsgNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
