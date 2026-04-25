#pragma once

#include <string>

namespace hako::robots::sensor::noise
{
    struct NoiseModelConfig
    {
        std::string type {"none"};
        double mean {0.0};
        double stddev {0.0};
        double bias_mean {0.0};
        double bias_stddev {0.0};
        double dynamic_bias_stddev {0.0};
        double dynamic_bias_correlation_time {0.0};
        double precision {0.0};
    };

    struct AxisNoiseConfig
    {
        NoiseModelConfig x {};
        NoiseModelConfig y {};
        NoiseModelConfig z {};
    };
}
