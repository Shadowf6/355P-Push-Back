#pragma once

#include <vector>

inline int auton = 5;
inline int variation = 1;
inline bool skills = false;
inline bool match = false;
inline bool driverControl = false;

constexpr int total = 10;
inline std::vector<const char*> descriptions = {
                               "6 Right", // 1
                               "6 Left", // 2
                               "4 Right", // 3
                               "4 Left", // 4
                               "Right Split", // 5
                               "Left Split", // 6
                               "Skills", // 7
                               "AWP Wing", // 8
                               "Move Forward", // 9
                               "Do Nothing", // 10
                               };

inline std::vector<std::vector<const char*>> variations = {
    {"Default"}, // 6 Right
    {"Default"}, // 6 Left
    {"Default"}, // 4 Right
    {"Default"}, // 4 Left
    {"Wing", "Stay"}, // Right Split
    {"Wing", "Stay"}, // Left Split
    {"Default"}, // Skills
    {"Right Goal", "Left Goal"}, // AWP Wing
    {"Default"}, // Move Forward
    {"Default"} // Do Nothing
};
