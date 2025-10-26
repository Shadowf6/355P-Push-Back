#pragma once

inline int auton = 2;

inline bool red = true;

inline bool skills = false;

inline int total = 4;
inline char* descriptions[] = {"Left (AWP)", "Right (Elim)", "Skills"};

constexpr float driveNoise = 0.1f;
constexpr float angleNoise = 0.001f;
constexpr float odomNoise = 0.01f;

constexpr float distXOffset = 0.0f;
constexpr float distYOffset = 0.0f;

constexpr float bounds[] = {0.0f, 144.0f, 0.0f, 144.0f};
