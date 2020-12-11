#pragma once


namespace lh2core
{

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	unique_ptr<float4[]> vertices = nullptr;							// vertex data received via SetGeometry
	unique_ptr<CoreTri[]> triangles = nullptr;							// 'fat' triangle data
	int vcount = 0;									// vertex count
	int meshID = 0;									// mesh id

	// No accidental copy
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	inline Mesh() noexcept  : vertices(nullptr), triangles(nullptr), vcount(0), meshID(0) {};

	inline Mesh(Mesh&& other) noexcept = default;
	inline Mesh& operator=(Mesh&& other) noexcept = default;
};

//  +-----------------------------------------------------------------------------+
//  |  Ray                                                                        |
//  |  Minimalistic ray storage.											  2020|
//  +-----------------------------------------------------------------------------+
struct Ray {
public:
	float4 origin;
	float4 direction;
	float4 invDir;

	/// <summary>
	/// Copy Ray
	/// </summary>
	/// <returns>New Copied Ray</returns>
	inline Ray Copy() const
	{
		return Ray(origin, direction);
	}

	inline Ray() : origin (make_float4(0)), direction(make_float4(1, 0, 0, 0)), invDir(1.0f / direction) { }

	inline Ray(const float4 &ori, const float4 &dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = ori;
		direction = dir;
		invDir = 1.0f / direction;
	}

	inline Ray(const float3& ori, const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = make_float4(ori);
		direction = make_float4(dir);
		invDir = 1.0f / direction;
	}

	inline void SetOrigin(const float4& ori)
	{
		origin = ori;
	}
	inline void SetOrigin(const float3& ori)
	{
		origin = make_float4(ori);
	}

	inline void SetDirection(const float4& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		direction = dir;
		invDir = 1.0f / direction;
	}
	inline void SetDirection(const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		direction = make_float4(dir);
		invDir = 1.0f / direction;
	}

	/// <summary>
	/// Evaluate the ray for certain t
	/// </summary>
	/// <param name="t">Parameter</param>
	/// <returns>Position</returns>
	inline float4 Evaluate(const float t) const
	{
		return origin + (t * direction);
	}


	inline bool TestAABBIntersection(const aabb &box) const
	{
		// From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
#if 0
		__m128 ori = _mm_setr_ps(origin.x, origin.y, origin.z, 0);
		__m128 dirInv = _mm_setr_ps(invDir.x, invDir.y, invDir.z, 0);

		__m128 t0 = _mm_mul_ps(_mm_sub_ps(box.bmin4, ori), dirInv);
		__m128 t1 = _mm_mul_ps(_mm_sub_ps(box.bmax4, ori), dirInv);
		
		__m128 tmin = _mm_min_ps(t0, t1);
		__m128 tmax = _mm_max_ps(t0, t1);
		
		return max_component(tmin) <= min_component(tmax);
#else
		float3 ori = make_float3(origin);
		float3 dirInv = make_float3(invDir);

		float3 t0 = (box.bmin3 - ori) * dirInv;
		float3 t1 = (box.bmax3 - ori) * dirInv;

		float3 tmin = fminf(t0, t1);
		float3 tmax = fmaxf(t0, t1);

		return max(tmin.x, max(tmin.y, tmin.z)) <= min(tmax.x, max(tmax.y, tmax.z));
#endif
	}
	
};


/// <summary>
/// Ray Triangle Interception Information
/// </summary>
struct RayTriangleInterceptInfo
{
public:
	/// <summary>
	/// Ray t Value
	/// </summary>
	float t;

	/// <summary>
	/// Barycentric u coordinate
	/// </summary>
	float u;

	/// <summary>
	/// Barycentric v coordinate
	/// </summary>
	float v;

	/// <summary>
	/// If the interception was back facing
	/// </summary>
	bool backFacing;

	inline RayTriangleInterceptInfo()
	{
		Reset();
	}

	inline void Reset()
	{
		t = std::numeric_limits<float>::infinity();
		u = 0;
		v = 0;
		backFacing = false;
	}

	inline float GetWCoord() const
	{
		assert(!isinf(t)); // Bad Interception
		return 1 - u - v;
	}

	// No copy by accident
	RayTriangleInterceptInfo(const RayTriangleInterceptInfo&) = delete;
	RayTriangleInterceptInfo& operator=(const RayTriangleInterceptInfo&) = delete;

	inline void CopyTo(RayTriangleInterceptInfo& info) const
	{
		info.t = t;
		info.u = u;
		info.v = v;
		info.backFacing = backFacing;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is closer than other to ray origin</returns>
	inline bool operator<(const RayTriangleInterceptInfo& other) const
	{
		return t < other.t;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is farther than other to ray origin</returns>
	inline bool operator>(const RayTriangleInterceptInfo& other) const
	{
		return !(*this < other);
	}

};

/// <summary>
/// Ray Mesh Interception Info
/// </summary>
struct RayMeshInterceptInfo
{
public:
	/// <summary>
	/// Triangle Interception Info
	/// </summary>
	RayTriangleInterceptInfo triIntercept;

	/// <summary>
	/// Mesh Id
	/// </summary>
	int meshId; 
	
	/// <summary>
	/// Triangle Id
	/// </summary>
	int triId;


	inline RayMeshInterceptInfo()
	{
		Reset();
	}

	inline void Reset()
	{
		triIntercept.Reset();
		meshId = -1;
		triId = -1;
	}

	// No copy by accident
	RayMeshInterceptInfo(const RayMeshInterceptInfo&) = delete;
	RayMeshInterceptInfo& operator=(const RayMeshInterceptInfo&) = delete;

	inline void CopyTo(RayMeshInterceptInfo& info) const
	{
		triIntercept.CopyTo(info.triIntercept);
		info.meshId = meshId;
		info.triId = triId;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is closer than other to ray origin</returns>
	inline bool operator<(const RayMeshInterceptInfo& other) const
	{
		return triIntercept < other.triIntercept;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is farther than other to ray origin</returns>
	inline bool operator>(const RayMeshInterceptInfo& other) const
	{
		return !(*this < other);
	}
};


/// <summary>
/// Test for Ray Triangle Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="v0">Vertex 0</param>
/// <param name="v1">Vertex 1</param>
/// <param name="v2">Vertex 2</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayTriangle(const Ray& r, const float4& v0, const float4& v1, const float4& v2, RayTriangleInterceptInfo& hitInfo);

/// <summary>
/// Test for Ray Mesh Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="m">Mesh</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayMesh(const Ray& r, const Mesh& m, RayMeshInterceptInfo& hitInfo);


/// <summary>
/// Test for Ray and a Scene (list of meshes) Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="meshes">Meshes</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayScene(const Ray& r, const vector<Mesh>& meshes, RayMeshInterceptInfo& hitInfo);

/// <summary>
/// Test depth for Ray Triangle Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="v0">Vertex 0</param>
/// <param name="v1">Vertex 1</param>
/// <param name="v2">Vertex 2</param>
/// <param name="tD">Depth Squared to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayTriangle(const Ray& r, const float4& v0, const float4& v1, const float4& v2, const float tD);

/// <summary>
/// Test depth for Ray Mesh Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="m">Mesh</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth Squared to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayMesh(const Ray& r, const Mesh& m, const int meshId, const int triId, const float tD);


/// <summary>
/// Test for Ray and a Sccene (list of meshes) Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="meshes">Meshes</param>
/// /// <param name="meshId">Instance Id (TODO)</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayScene(const Ray& r, const vector<Mesh>& meshes, const int instId, const int meshId, const int triId, const float tD);

}

#include "intersection_utils.impl.h"