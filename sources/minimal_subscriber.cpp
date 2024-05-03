/*
Name: minimal_subscriber.cpp

Description: ROS subcriber to the topic "cpu_load_topic"

Company: Spleenlab

Author: Edwin Carreño
Last update: 03.05.2024
*/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

//++++++++++++++++++++++++++++++			CUSTOM FUNCTIONS		+++++++++++++++++++++++++++
/**
 * @brief Write a string into a file.
 *
 * This function ask for a path to the file, the filename and the data to writo into.
 *
 * @param path Path to the output file.
 * @param filename Name of the output file.
 * @param data String to be written into the output file in append mode.
 * @return void return.
 */
void write_to_file(std::string path, std::string filename, std::string data)
{
	std::ofstream out_file(path + filename,
						  std::ios::app);

	if(!out_file)
	{
		std::cerr << "Cannot create the file: " << filename << std::endl;
		exit(-1);
	}
	
	out_file << data;
	out_file.close();
}

//++++++++++++++++++++++++++++++++++++++++++++++      STRUCTS AND CLASSES
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cpu_load_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      // Paths for logging CPU load results
	    //std::string path_output_file = "/home/egcarren/WorkspacesSoftware/Spleenlab/base/sources/";
      std::string path_output_file = "/home/ROS_WS/";
	    std::string output_filename = "cpu_load.log";

      // Write stats to the output file
		  write_to_file(path_output_file, output_filename, output_data);

      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}