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
		assert(almost_equal(1, length(dir)), "Unormalized Vector");
		origin = ori;
		direction = dir;
	}

	inline Ray(const float3& ori, const float3& dir)
	{
		assert(almost_equal(1, length(dir)), "Unormalized Vector");
		origin = make_float4(ori);
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



}