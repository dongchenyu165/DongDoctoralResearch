#ifndef C06167EC_693E_4C48_8C3B_1A9E724F7505
#define C06167EC_693E_4C48_8C3B_1A9E724F7505

#include <iostream>
#include <cstdlib>

inline void AssertWithMessage(bool InCondition, const char* InConditionStr, const char* InMsg, const char* InFile, int InLine)
{
    if (!InCondition)
    {
        std::cerr << "Assertion failed at " << InFile << ":" << InLine << std::endl
                  << "Condition: " << InConditionStr << std::endl
                  << "Message: " << InMsg << std::endl;
        std::abort();
    }
}

#define ASSERT_MSG(CONDITION, MSG) AssertWithMessage((CONDITION), #CONDITION, (MSG), __FILE__, __LINE__)

#endif /* C06167EC_693E_4C48_8C3B_1A9E724F7505 */
