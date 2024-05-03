#include <iostream>
#include <thread>
#include <vector>
#include <cmath>

// Function to perform CPU-intensive task
void cpu_intensive_task()
{
	// This loop will consume CPU cycles
		for(int i = 0; i < 1000000000; i++)
		{
			// Do some computation
			double result = std::sqrt(i) * std::log(i+1);
			
			// Simulate some work by printing result
			std::cout << "Result: " << result << std::endl;
		}
}

int main()
{
	//Get the number of available hardware threads
	unsigned int num_threads = std::thread::hardware_concurrency();
	
	std::vector<std::thread> threads;
	
	// Create threads to perform CPU-intensive tasks
	for(unsigned int i = 0; i < num_threads; i++)
	{
		threads.push_back(std::thread(cpu_intensive_task));
	}
	
	// Join all threads
	for(auto& thread : threads)
	{
		thread.join();
	}
	
	return 0;
}
