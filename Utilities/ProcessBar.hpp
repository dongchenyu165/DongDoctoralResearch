#ifndef B91BBE7B_C650_4082_A3BD_F400B779BB6C
#define B91BBE7B_C650_4082_A3BD_F400B779BB6C

#include <iostream>


namespace Utilities
{

/**
 * @brief Updates and displays a progress bar in the console. Like:
 * [=====================>] 100%
 * 
 * @param InPercentage The current progress as a float between 0.0 and 1.0.
 * @param InBarWidth The width of the progress bar in characters (default is 20).
 */
inline void UpdateProgress(float InPercentage, const int InBarWidth = 20)
{
	int Position       = InBarWidth * InPercentage;

	std::cout.flush();
	std::cout << "\r[";
	for ( int i = 0; i < InBarWidth; ++i )
	{
		if ( i < Position )
		{
			std::cout << "=";
		}
		else if ( i == Position )
		{
			std::cout << ">";
		}
		else
		{
			std::cout << " ";
		}
	}
	std::cout << "] " << int(InPercentage * 100.0) << "%" << std::flush;
}

}

#endif /* B91BBE7B_C650_4082_A3BD_F400B779BB6C */
