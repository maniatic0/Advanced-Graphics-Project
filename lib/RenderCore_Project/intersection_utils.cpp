#include "core_settings.h" // not sure

namespace lh2core
{

bool interceptRayTriangle(
	const bool backCulling,
	const Ray& r, 
	const float4& v0, const float4& v1, const float4& v2, 
	RayTriangleInterceptInfo& hitInfo)
{
	/// Modified from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
	float4 v0v1 = v1 - v0;
	float4 v0v2 = v2 - v0;
	float4 pvec = cross(r.direction, v0v2);
	float det = dot(v0v1, pvec);

	if (
		(backCulling && (det < kEps)) // Back culling triangles
		|| (!backCulling && (fabs(det) < kEps))) // Only cull parallel rays
	{
		// No intersection
		return false;
	}
	hitInfo.backFacing = det < 0;

	float invDet = 1 / det;

	float4 tvec = r.origin - v0;
	hitInfo.u = dot(tvec, pvec) * invDet;
	if (hitInfo.u < 0 || hitInfo.u > 1)
	{
		return false;
	}

	float4 qvec = cross(tvec, v0v1);
	hitInfo.v = dot(r.direction, qvec) * invDet;
	if (hitInfo.v < 0 || hitInfo.u + hitInfo.v > 1) 
	{
		return false;
	}

	hitInfo.t = dot(v0v2, qvec) * invDet;

	return true;
}

}