#pragma once

#include <cmath>

float rad(float deg) { 
    return deg * 0.01745329251f; 
}

float inch(float mm) { 
    return mm / 25.4f; 
}

float wrap(float t) { 
    t = std::fmod(t + 180, 360); 
    if (t < 0) t += 360; 
    return t - 180; 
}

