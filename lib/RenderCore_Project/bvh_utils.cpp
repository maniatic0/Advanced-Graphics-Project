#include "core_settings.h"
#include "bvh_utils.h"

namespace lh2core
{

	bool IntersectRayBVHNode(const Ray& r, const BVHNode& node)
	{
		float3 dirfrac;

		// Source: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
		dirfrac.x = 1.0f / r.direction.x;
		dirfrac.y = 1.0f / r.direction.y;
		dirfrac.z = 1.0f / r.direction.z;

		float t1 = (node.lb.x - r.origin.x) * dirfrac.x;
		float t2 = (node.rt.x - r.origin.x) * dirfrac.x;
		float t3 = (node.lb.y - r.origin.y) * dirfrac.y;
		float t4 = (node.rt.y - r.origin.y) * dirfrac.y;
		float t5 = (node.lb.z - r.origin.z) * dirfrac.z;
		float t6 = (node.rt.z - r.origin.z) * dirfrac.z;

		float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
		float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

		// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
		if (tmax < 0)
		{
			//t = tmax;
			return false;
		}

		// if tmin > tmax, ray doesn't intersect AABB
		if (tmin > tmax)
		{
			//t = tmax;
			return false;
		}

		//t = tmin;
		return true;
	}

}