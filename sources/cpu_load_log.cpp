#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>
#include <ctime>

//++++++++++++++++++++++++++++++			STRUCTS AND CLASSES		+++++++++++++++++++++++++++
/*
* This struct stores the idle and busy times of a processor in one instance of time.
*/
struct cpu_times_intance
{
	long idle_time{0};
	long busy_time{0};
};


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

//+++++++++++++++++++++++++++			MAIN FUNCTION			+++++++++++++++++++
int main()
{
	// Paths for logging CPU load results
	std::string path_output_file = "/home/egcarren/WorkspacesSoftware/Spleenlab/base/sources/";
	std::string output_filename = "cpu_load.log";
	
	// Initialization of structs
	struct cpu_times_intance initial_cpu_load;
	struct cpu_times_intance current_cpu_load;	
	
	// Defines the interval duration for calculating the CPU load in milliseconds.
	int duration_interval_ms{5000}; 
	
	initial_cpu_load = get_cpu_load(); // Store time values for the first interval
	
	// Wait for 5 seconds
	std::this_thread::sleep_for(std::chrono::milliseconds(duration_interval_ms));
	
	while(true)
	{
		// Store current time values
		current_cpu_load = get_cpu_load();
		
		// Calculate relative CPU load
		double CPU_LOAD = calculate_cpu_load(current_cpu_load, initial_cpu_load);

		// Add timestamp to the CPU load 
		std::string output_data = cpu_load_timestamp(CPU_LOAD); 

		// Print in console using cout
		std::cout << output_data;
		
		// Write stats to the output file
		write_to_file(path_output_file, output_filename, output_data);
		
		// Update initial cpu load for subsequent interval
		initial_cpu_load = current_cpu_load;
		
		// Wait for 5 seconds
		std::this_thread::sleep_for(std::chrono::milliseconds(duration_interval_ms));		
	}
	
	return 0;
}


