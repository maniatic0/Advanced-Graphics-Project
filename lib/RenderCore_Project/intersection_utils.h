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
	int vcount = 0;									// vertex count
	CoreTri* triangles = 0;							// 'fat' triangle data
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
	inline Ray Copy()
	{
		return Ray(origin, direction);
	}

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
		direction = dir;
	}
	inline void SetDirection(const float3& dir)
	{
		direction = make_float4(dir);
	}

	/// <summary>
	/// Evalute the ray for certain t
	/// </summary>
	/// <param name="t">Parameter</param>
	/// <returns>Position</returns>
	inline float4 Evaluate(const float t)
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
		t = 0;
		u = 0;
		v = 0;
		backFacing = false;
	}


	inline float GetWCoord()
	{
		return 1 - u - v;
	}

	// No copy by accident
	RayTriangleInterceptInfo(const RayTriangleInterceptInfo&) = delete;
	RayTriangleInterceptInfo& operator=(const RayTriangleInterceptInfo&) = delete;

	inline void CopyTo(RayTriangleInterceptInfo& info)
	{
		info.t = t;
		info.u = u;
		info.v = v;
		info.backFacing = backFacing;
	}

};


}