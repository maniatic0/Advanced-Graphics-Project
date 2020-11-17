#include "core_settings.h"

[[nodiscard]]
bool depthRayTriangle(const Ray& r, const float4& v0, const float4& v1, const float4& v2, const float tD)
{
	/// Modified from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
	float4 v0v1 = v1 - v0;
	float4 v0v2 = v2 - v0;
	float4 pvec = cross(r.direction, v0v2);
	float det = dot(v0v1, pvec);

	// Cool C++17 trick
	if constexpr (backCulling)
	{
		if (det < kEps) // Back culling triangles
		{
			// No intersection
			return false;
		}
	}
	else
	{
		if (fabs(det) < kEps) // Only cull parallel rays (No backculling)
		{
			// No intersection
			return false;
		}
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

	if (hitInfo.t < kEps)
	{
		return false;
	}

	return true;
}

/// <summary>
/// Test depth for Ray Mesh Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="m">Mesh</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
[[nodiscard]]
bool depthRayMesh(const Ray& r, const Mesh& m, const int meshId, const int triId, const float tD);


/// <summary>
/// Test for Ray and a list of Meshes Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="meshes">Meshes</param>
/// /// <param name="meshId">Instance Id (TODO)</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
[[nodiscard]]
bool depthRayMeshes(const Ray& r, const vector<Mesh>& meshes, const int instId, const int meshId, const int triId, const float tD);