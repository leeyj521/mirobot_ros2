#include <cstdio>
#include <string>
// #include <ros/ros.h>
// #include <serial/serial.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/JointState.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node_impl.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_bridge_node.hpp"


namespace mirobot_urdf
{
uint32_t baud = 115200;
drivers::serial_driver::FlowControl f;
drivers::serial_driver::Parity p;
drivers::serial_driver::StopBits s;
drivers::serial_driver::SerialPortConfig conf(baud, f, p, s);
drivers::common::IoContext io;
drivers::serial_driver::SerialPort _serial(io, "/dev/ttyUSB0", conf);				// serial object

using namespace std::placeholders;

class AngleCallbackNode : public rclcpp::Node
{
public:
    AngleCallbackNode() : Node("Mirobot_write_node")
	{
		auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(20));
		subscriber_ptr_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states",
			qos_profile,
			std::bind(&AngleCallbackNode::angle_write_callback, this, _1)
		);
	}

private:
	std_msgs::msg::String result;
    void angle_write_callback(const sensor_msgs::msg::JointState& msg)
	{
		if(!_serial.is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Serial port is not open!");
			return;
		}

		std::string Gcode = "";
		char angle0[10];
		char angle1[10];
		char angle2[10];
		char angle3[10];
		char angle4[10];
		char angle5[10];

		sprintf(angle0, "%.2f", msg.position[0]*57.296);
		sprintf(angle1, "%.2f", msg.position[1]*57.296);
		sprintf(angle2, "%.2f", msg.position[2]*57.296);
		sprintf(angle3, "%.2f", msg.position[3]*57.296);
		sprintf(angle4, "%.2f", msg.position[4]*57.296);
		sprintf(angle5, "%.2f", msg.position[5]*57.296);
		Gcode = (std::string)"M50 G0 X" + angle0 + " Y" + angle1 + " Z" + angle2 + " A" + angle3 + "B" + angle4 + "C" + angle5 + " F3000" + "\r\n";

		RCLCPP_INFO(this->get_logger(), "%s", Gcode.c_str());

		std::vector<uint8_t> vec(Gcode.begin(), Gcode.end());
		try{
			_serial.send(vec);
			result.data = _serial.receive(vec);
			RCLCPP_INFO(this->get_logger(), "Received response: %s", result.data.c_str());
		}
		catch(const std::exception& ex)
		{
			RCLCPP_ERROR(this->get_logger(), "Exception during serial communication: %s", ex.what());

		}
	}

	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_ptr_;
};
}

int main(int argc, char** argv)
{	
	rclcpp::init(argc, argv);//初始化，节点名称 "Mirobot_write_node"
	auto node= std::make_shared<mirobot_urdf::AngleCallbackNode>();

	rclcpp::Rate loop_rate(20);//指定了频率为20Hz

	mirobot_urdf::_serial.open();
	std::string str = "M50\r\n";
	std::vector<uint8_t> vec(str.begin(), str.end());

	mirobot_urdf::_serial.send(vec);

	if (mirobot_urdf::_serial.send_break())
	{
		RCLCPP_ERROR_STREAM(rclcpp::get_logger("rclcpp"), "Unable to open port");
		return -1;
	}

	RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Port has been open successfully");
	
	if (mirobot_urdf::_serial.is_open())
	{
		rclcpp::Duration(int32_t(1), uint32_t(0));
		rclcpp::Duration(int32_t(0), uint32_t(0));
		RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Attach and wait for commands");				
	}

	while (rclcpp::ok())
	{
		rclcpp::spin(node);
		loop_rate.sleep();
	}
	
	return 0;
}

