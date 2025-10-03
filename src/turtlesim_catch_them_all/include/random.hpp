
#include<random>

class RandomNumberGenerator
{
    std::random_device m_randomDevice{};
    std::mt19937 m_engine{m_randomDevice()};

public:
    // Generates a random float in the range [low, high)
    float Generate(float low, float high)
    {
        return std::uniform_real_distribution<float>{low, high}(m_engine);
    }
};