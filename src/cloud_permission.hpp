#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

// Session and sequence guard against repeated/old authorization messages.
class CloudPermission
{
public:
    using Clock = std::chrono::steady_clock;
    explicit CloudPermission(double timeout = 0.5) { SetTimeout(timeout); }
    void SetTimeout(double timeout)
    {
        if (!std::isfinite(timeout) || timeout <= 0)
            throw std::invalid_argument("Cloud authorization timeout must be positive");
        timeout_ = std::chrono::duration<double>(timeout);
    }
    void BindSession(const std::string& session)
    {
        session_ = session;
        sequence_ = 0;
        enabled_ = has_update_ = false;
        bound_ = true;
    }
    bool Update(const std::string& session, uint64_t sequence, bool enabled,
                Clock::time_point now, double stamp_age = 0.0)
    {
        if (!std::isfinite(stamp_age) || stamp_age < -0.1 || stamp_age > timeout_.count())
            return false;
        if (session.empty() || (bound_ && session != session_) || (has_update_ && (session != session_ || sequence <= sequence_)))
            return false;
        session_ = session;
        sequence_ = sequence;
        enabled_ = enabled;
        received_ = now - std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(std::max(0.0, stamp_age)));
        has_update_ = true;
        return true;
    }
    bool Enabled(Clock::time_point now) const
    {
        return has_update_ && enabled_ && now >= received_ && now - received_ <= timeout_;
    }
private:
    std::string session_;
    uint64_t sequence_ = 0;
    bool enabled_ = false, has_update_ = false, bound_ = false;
    Clock::time_point received_{};
    std::chrono::duration<double> timeout_{0.5};
};
