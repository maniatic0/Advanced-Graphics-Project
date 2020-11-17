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

/// <summary>
/// 3D Cross product using float 4
/// </summary>
/// <param name="a"></param>
/// <param name="b"></param>
/// <returns></returns>
inline float4 cross(float4 a, float4 b) { return make_float4(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 0); }


inline void getLightComponents(const CoreMaterial& mat, float &diffuse, float &reflection, float &refraction)
{
	reflection = mat.reflection.value;
	refraction = mat.refraction.value;
	int maskIsNull = (reflection == 1e32 ? 1 : 0) | (refraction == 1e32 ? 2 : 0);
	switch (maskIsNull)
	{
	case 0:
		// All values
		break;
	case 1:
		// no reflection
		reflection = 0;
		break;
	case 2:
		// no refrection
		refraction = 0;
		break;
	default:
		// No values
		reflection = 0;
		refraction = 0;
		break;
	}
	diffuse = 1.0f - reflection - refraction;
}

}