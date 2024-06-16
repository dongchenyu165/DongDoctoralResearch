#ifndef CEAFC75D_6D5D_4CA1_8AB2_83A886FED592
#define CEAFC75D_6D5D_4CA1_8AB2_83A886FED592

#include <string>
#include <chrono>

constexpr int FINGER_NUMBER = 2;

// True: Consider the gravity and the force from the cutting board
constexpr bool bCONSIDER_GRAVITY = false;



inline std::string CalculationParamJsonPath = "./params.json";
// inline std::string LogConfigJsonPath = "log_config.json";
inline std::string LogConfigJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/log_config.json";

inline std::string ViewerConfigJsonPath = "/home/cookteam/Workspace/CPP_Program/PythonForceCalculator_Refactor/viewer_config.json";

// (Global var) Storage the time of this program starting.
inline const std::chrono::system_clock::time_point ProgramStartTime = std::chrono::system_clock::now();

#endif /* CEAFC75D_6D5D_4CA1_8AB2_83A886FED592 */
