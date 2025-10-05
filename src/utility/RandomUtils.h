#ifndef RANDOM_UTILS_H
#define RANDOM_UTILS_H

#include <random>

namespace RndU
{
    inline std::mt19937& generator()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        return gen;
    }

    inline float uniformFloat(float min = -1.0f, float max = 1.0f)
    {
        static std::uniform_real_distribution<float> dist; // dummy
        return std::uniform_real_distribution<float>(min, max)(generator());
    }
}

#endif