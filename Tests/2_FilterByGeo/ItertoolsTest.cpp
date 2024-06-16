
#include <cppitertools/combinations.hpp>
#include <iostream>
#include <chrono>
#include <omp.h>

#define START_TIMER(NAME) auto NAME = std::chrono::high_resolution_clock::now()

#define STOP_TIMER(NAME)                                                                                               \
	auto NAME##_TIME_LENGTH = std::chrono::high_resolution_clock::now() - NAME;                                        \
	std::cout << "Timer [" << #NAME << "] use "                                                                        \
			  << std::chrono::duration_cast<std::chrono::microseconds>(NAME##_TIME_LENGTH).count() << " microseconds." \
			  << std::endl

#define __DEBUG_SLEEP_MS__(DURATION) std::this_thread::sleep_for(std::chrono::milliseconds(DURATION))

int main()
{
	// std::vector<int> v = { 1, 2, 3, 4, 5 };
	// for ( auto&& i : iter::combinations(v, 3) )
	// {
	// 	for ( auto&& j : i )
	// 	{
	// 		std::cout << j << " ";
	// 	}
	// 	std::cout << '\n';
	// }

	/* --------------------------- OMP for assign test -------------------------- */
	constexpr int TEST_LENGTH = 2e2;
	std::vector<int> IndexList;
	IndexList.assign(TEST_LENGTH, -1);

	START_TIMER(timer_omp);
#pragma omp parallel for
	for ( int i = 0; i < TEST_LENGTH; i++ )
	{
		for ( int j = 0; j < 30000; j++ )
		{
			IndexList[i] = i;
		}
	}
	STOP_TIMER(timer_omp);

	IndexList.assign(TEST_LENGTH, -1);
	START_TIMER(timer_seq);
	for ( int i = 0; i < TEST_LENGTH; i++ )
	{
		for ( int j = 0; j < 30000; j++ )
		{
			IndexList[i] = i;
		}
	}
	STOP_TIMER(timer_seq);

	return 0;
}
