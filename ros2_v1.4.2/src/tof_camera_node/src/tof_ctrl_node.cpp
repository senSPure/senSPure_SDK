/*------------------------------------------------------------------*/
/// @file		tof_ctrl_node.cpp
/// @brief		TOF Control Node
/// @copyright	Copyright (C) 2023 TOPPAN Holdings Inc.
/*------------------------------------------------------------------*/

#include <rclcpp/rclcpp.hpp>
#include "tof_ctrl_component.hpp"

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<tof_camera_node::TofCtrlNode>(rclcpp::NodeOptions()));
	rclcpp::shutdown();
	return 0;
}
