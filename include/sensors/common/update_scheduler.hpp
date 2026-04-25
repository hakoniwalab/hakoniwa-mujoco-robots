#pragma once

namespace hako::robots::sensor::common
{
    class UpdateScheduler
    {
    public:
        void Reset()
        {
            elapsed_sec_ = 0.0;
        }

        void StartReady(double period_sec)
        {
            elapsed_sec_ = period_sec;
        }

        bool ShouldUpdate(double delta_sec, double period_sec)
        {
            elapsed_sec_ += delta_sec;
            if (elapsed_sec_ + 1.0e-9 < period_sec) {
                return false;
            }
            elapsed_sec_ -= period_sec;
            if (elapsed_sec_ < 0.0) {
                elapsed_sec_ = 0.0;
            }
            return true;
        }

    private:
        double elapsed_sec_ {0.0};
    };
}
