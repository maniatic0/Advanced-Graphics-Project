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
	float4* vertices = 0;							// vertex data received via SetGeometry
	CoreTri* triangles = 0;							// 'fat' triangle data
	int vcount = 0;									// vertex count
	int meshID = 0;									// mesh id
};

// -----------------------------------------------------------
// Scene class
// owner of the scene graph;
// owner of the material and texture list
// -----------------------------------------------------------
class Scene {
public:
	// constructor / destructor
	Scene() = default;
	// data members
	vector<CoreMaterial> matList;
	CoreMaterial air;

	vector<CoreTexDesc> texList;

	// Lights
	vector<CorePointLight>			pointLights;
	vector<CoreSpotLight>			spotLights;
	vector<CoreDirectionalLight>	directionalLights;

	vector<CoreLightTri>	areaLights;

};

//  +-----------------------------------------------------------------------------+
//  |  Ray                                                                        |
//  |  Minimalistic ray storage.											  2020|
//  +-----------------------------------------------------------------------------+
struct Ray {
public:
	float4 origin;
	float4 direction;

	// No copy by accident
	Ray(const Ray&) = delete;
	Ray& operator=(const Ray&) = delete;

	/// <summary>
	/// Copy Ray
	/// </summary>
	/// <returns>New Copied Ray</returns>
	inline Ray Copy() const
	{
		return Ray(origin, direction);
	}

	inline Ray() { origin = make_float4(0);  direction = make_float4(1, 0, 0, 0); }

	inline Ray(const float4 &ori, const float4 &dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = ori;
		direction = dir;
	}

	inline Ray(const float3& ori, const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = make_float4(ori);
		direction = make_float4(dir);
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
	}
	inline void SetDirection(const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		direction = make_float4(dir);
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