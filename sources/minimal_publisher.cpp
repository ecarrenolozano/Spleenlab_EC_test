/*
Name: minimal_publisher.cpp

Description: ROS node that publishes the CPU load every 5 seconds into the topic "cpu_load_topic"

Company: Spleenlab

Author: Edwin Carreño
Last update: 03.05.2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <ctime>

// ROS LIBRARIES
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

//++++++++++++++++++++++++++++++			STRUCTS AND CLASSES		+++++++++++++++++++++++++++
/*
* This struct stores the idle and busy times of a processor in one instance of time.
*/
struct cpu_times_intance
{
	long idle_time{0};
	long busy_time{0};
};

/**
 * @brief Read the first line of file.
 *
 * This function reads the first line of a file each time it is invoked.
 *
 * @param filename Name of the input file including the path, i.e. /proc/stat, being "stat" the filename.
 * @return line, string with the content of the first line read.
 */
std::string read_file(std::string filename)
{
	std::ifstream stat_file(filename);
	std::string line;
	std::getline(stat_file, line);
	stat_file.close();

	return line;
}

/**
 * @brief Extract the idle and busy times registered by the CPU at a time instance. 
 *
 * This function extracts the idle and total times based on statistics perform by the processor. * 
 * Registered times are based on values contained in "/proc/stat" file in Linux.
 *
 * @return times, struct with the values idle and total time.
 */

// Function to read CPU load from /proc/stat file
struct cpu_times_intance get_cpu_load()
{
	struct cpu_times_intance times;
	
	// Extract the first line of "/proc/stat"
	std::string line = read_file("/proc/stat");

	// Extracting CPU load information from the line variable
	long user,			// field 1 
		 nice,			// field 2
		 system,		// field 3 
		 idle,			// field 4
		 iowait,		// field 5
		 irq,			// field 6
		 softirq,		// field 7
		 steal,			// field 8
		 guest,			// field 9
		 guest_nice;	// field 10
	
	sscanf(line.c_str(), "cpu %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld",
		   &user, &nice, &system, &idle, &iowait, &irq, &softirq, &steal, &guest, &guest_nice);	
	
	// Calculating total time spent by CPU
	long total_time_step = user + nice + system + idle + iowait + irq + softirq + steal;
	
	// Calculating total idle time
	long total_idle_step = idle + iowait;
	
	// Calculating CPU load percentage
	times.idle_time = total_idle_step;
	times.busy_time = total_time_step;
	
	return times;	
}

/**
 * @brief Calculate the relative CPU load based on two time steps, an initial time step and a current time step. 
 *
 * This function allows to calculate the relative CPU load based on two time steps, an initial and a current time step.
 * 
 *
 * @return cpu_load, double variable that represents the ratio between CPU when is busy and the total CPU load.
 */
double calculate_cpu_load(cpu_times_intance current_cpu_load, cpu_times_intance initial_cpu_load)
{
	long cpu_total_time = current_cpu_load.busy_time - initial_cpu_load.busy_time;
	long cpu_idle_time  = current_cpu_load.idle_time - initial_cpu_load.idle_time;
	long cpu_busy_time = cpu_total_time - cpu_idle_time;
	double cpu_load = 100.0 * cpu_busy_time/cpu_total_time;

	return cpu_load;
}

std::string cpu_load_timestamp(double cpu_load)
{
	auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

	std::ostringstream oss;		
	oss << "CPU Load: " 
		<< std::fixed
		<< std::setprecision(2)
		<< cpu_load
		<< "%, " 
		<< std::ctime(&timenow);
		
	return oss.str();
}


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher() : Node("minimal_publisher"), count_(0), initial_cpu_load{0,0}, current_cpu_load{0,0}
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("cpu_load_topic", 10); // Topic name ("cpu_load_topic") and buffer size (10)
        timer_ = this->create_wall_timer(
        5000ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
        // Get current CPU load and store it
        current_cpu_load = get_cpu_load();
		
		// Calculate relative CPU load
		double CPU_LOAD = calculate_cpu_load(current_cpu_load, initial_cpu_load);

		// Add timestamp to the CPU load 
		std::string output_data = cpu_load_timestamp(CPU_LOAD); 

		// Print in console using cout
		//std::cout << output_data;
		
		// Update initial cpu load for subsequent interval
		initial_cpu_load = current_cpu_load;

        //Publish the message in the topic            
        auto message = std_msgs::msg::String();
        //message.data = output_data + std::to_string(count_++);
        message.data = output_data;        

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    cpu_times_intance initial_cpu_load;
	cpu_times_intance current_cpu_load;
};

int main(int argc, char * argv[])
{
	std::cout << "Starting log CPU load messages in topic..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}