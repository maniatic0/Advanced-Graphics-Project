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

	// No Copy
	Ray(const Ray&) = delete;
	Ray& operator=(const Ray&) = delete;

	inline Ray() {}

	inline Ray(const float4 &ori, const float4 &dir)
	{
		origin = ori;
		direction = dir;
	}

	inline Ray(const float3& ori, const float3& dir)
	{
		origin = make_float4(ori);
		direction = make_float4(dir);
	}

	inline float4 Evaluate(const float t)
	{
		return origin + (t * direction);
	}
};



}