/*#include <iostream>
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
*/

#include <iostream>

// Recursive function to calculate factorial
long long factorial(int n) {
    if (n == 0 || n == 1) {
        return 1;
    } else {
        return n * factorial(n - 1);
    }
}

int main() {
    // Define a large number for factorial calculation
    int num = 20;

    // Calculate factorial
    long long result = factorial(num);

    // Print the result
    std::cout << "Factorial of " << num << " is: " << result << std::endl;

    return 0;
}