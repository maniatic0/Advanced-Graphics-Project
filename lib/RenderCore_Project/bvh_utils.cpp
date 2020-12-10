#include "core_settings.h"
#include "bvh_utils.h"

namespace lh2core
{
	void BVH::ConstructBVH(Mesh&& mesh)
	{
		// Steal pointers
		mesh = std::move(mesh);
		
		// create index array
		indices = new uint[mesh.vcount];
		for (int i = 0; i < mesh.vcount; i++) {
			indices[i] = i; 
		}
		// allocate BVH root node
		const int poolSize = mesh.vcount * 2 - 1;
		pool = new BVHNode[poolSize];
		root = &pool[0];
		poolPtr = 2; 
		// subdivide root node
		root->leftFirst = 0;
		root->count = mesh.vcount;
		CalculateBounds(root);
		Subdivide(root);
	}

	void BVH::CalculateBounds(BVHNode* node)
	{
		aabb& bounds = node->bounds;
		bounds.Reset();
		
		assert(!node->IsLeaf());
		const int tStart = node->leftFirst;
		const int tSize = node->count;
		const int tMax = tStart + tSize;
		assert(0 <= tStart && 0 <= tSize);
		assert(tMax <= mesh.vcount);
		for (int i = tStart; i < tMax; i++)
		{
			const int index = indices[i];
			const int realIndex = 3 * index;
			bounds.Grow(mesh.vertices[realIndex + 0]);
			bounds.Grow(mesh.vertices[realIndex + 1]);
			bounds.Grow(mesh.vertices[realIndex + 2]);
		}
	}

	void BVH::Subdivide(BVHNode* node)
	{
		assert(node->count > 0 && !node->IsLeaf());
		if (node->count < 3) { 
			return; 
		}
		const int first = node->leftFirst;
		const int leftChild = poolPtr++;
		const int rightChild = poolPtr++;
		node->leftFirst = -leftChild;
		const int p = Partition(node);

		BVHNode* left = &pool[leftChild];
		BVHNode* right = &pool[rightChild];

		left->leftFirst = first;
		left->count = p;
		Subdivide(left);

		right->leftFirst = p;
		right->count = node->count;
		Subdivide(right);
	}

	inline float getTriangleCenterAxis(const float4& a, const float4& b, const float4& c, const int axis)
	{
		switch (axis)
		{
		case 0:
			return (a.x + b.x + c.x) / 3.0f;
		case 1:
			return (a.y + b.y + c.y) / 3.0f;
		case 3:
			return (a.z + b.z + c.z) / 3.0f;
		default:
			assert(false); // What
			break;
		}
		assert(false); // What
		return 0;
	}

	int BVH::Partition(BVHNode* node)
	{
		assert(node->IsLeaf());
		// Lomut's partition https://en.wikipedia.org/wiki/Quicksort
		const int splitAxis = node->bounds.LongestAxis();
		float4* vertices = mesh.vertices;

		int hi = node->count;
		int lo = node->FirstPrimitive();

		int triangleIndex = indices[(hi + lo) / 2] * 3;

		const float pivot = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);

		int i = lo - 1;
		int j = hi - 1;

		float center = 0;
		while (true)
		{
			do
			{
				++i;
				triangleIndex = indices[i] * 3;
				center = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);
			} while (center < pivot);
			
			do
			{
				--j;
				triangleIndex = indices[j] * 3;
				center = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);
			} while (center > pivot);

			if (i >= j)
			{
				return j;
			}
			std::swap(indices[i], indices[j]);
		}
		return -1;
	}

	/*
	bool BVHNode::IntersectRay(const Ray& r, float& t)
	{
		float3 dirfrac;

		// Source: https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
		dirfrac.x = 1.0f / r.direction.x;
		dirfrac.y = 1.0f / r.direction.y;
		dirfrac.z = 1.0f / r.direction.z;

		float t1 = (lb.x - r.origin.x) * dirfrac.x;
		float t2 = (rt.x - r.origin.x) * dirfrac.x;
		float t3 = (lb.y - r.origin.y) * dirfrac.y;
		float t4 = (rt.y - r.origin.y) * dirfrac.y;
		float t5 = (lb.z - r.origin.z) * dirfrac.z;
		float t6 = (rt.z - r.origin.z) * dirfrac.z;

		float tmin = max(max(min(t1, t2), min(t3, t4)), min(t5, t6));
		float tmax = min(min(max(t1, t2), max(t3, t4)), max(t5, t6));

		// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
		if (tmax < 0)
		{
			t = tmax;
			return false;
		}

		// if tmin > tmax, ray doesn't intersect AABB
		if (tmin > tmax)
		{
			t = tmax;
			return false;
		}

		t = tmin;
		return true;
	}
	*/

}