#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>

// No vehicle, flight mode or autopilot types belong in this interface.
class InitializationPermission
{
public:
	using Clock = std::chrono::steady_clock;
	enum State : uint8_t { Blocked = 0, Allowed = 1, Released = 2, Aborted = 3 };

	struct Decision
	{
		State state;
		bool fresh;
	};

	void SetTimeout(double seconds)
	{
		if (!std::isfinite(seconds) || seconds <= 0.0)
		{
			throw std::invalid_argument("Initialization control timeout must be finite and positive");
		}
		timeout_ = std::chrono::duration<double>(seconds);
	}

	void Update(uint8_t value, Clock::time_point now)
	{
		state_ = value <= Aborted ? static_cast<State>(value) : Aborted;
		received_ = now;
		has_update_ = true;
	}

	Decision Read(Clock::time_point now) const
	{
		const auto age = now - received_;
		const bool fresh = has_update_ && age >= Clock::duration::zero() && age <= timeout_;
		return {fresh ? state_ : Blocked, fresh};
	}

private:
	State state_ = Blocked;
	bool has_update_ = false;
	Clock::time_point received_{};
	std::chrono::duration<double> timeout_{1.0};
};
