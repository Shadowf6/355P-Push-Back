#pragma once

#include <cmath>

inline float rad(float deg) { 
    return deg * 0.017453292f; 
}

inline float inch(float mm) { 
    return mm / 25.4f; 
}

inline float wrap(float t) { 
    t = std::fmod(t + 180, 360); 
    if (t < 0) t += 360; 
    return t - 180; 
}
