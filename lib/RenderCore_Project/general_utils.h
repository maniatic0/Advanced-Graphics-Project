#pragma once

namespace lh2core
{

/// <summary>
/// Approximately the same
/// </summary>
/// <param name="eps">Epsilon</param>
/// <param name="target">Target Val</param>
/// <param name="value">Val</param>
/// <returns>If they are almost the same</returns>
inline bool approximately(float eps, float target, float value)
{
	return fabs(target - value) < eps;
}

/// <summary>
/// General epsilon to use
/// </summary>
constexpr float kEps = 0.00001;

#define almost_equal(target, value) approximately(kEps, (target), (value))


}