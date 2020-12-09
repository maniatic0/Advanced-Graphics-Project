#include "core_settings.h"
#include "bvh_utils.h"

namespace lh2core
{
	void BVH::ConstructBVH(vector<Mesh> meshes)
	{
		int N = 12; // not sure yet what this should be
		
		// create index array
		indices = new uint[N];
		for( int i= 0; i < N; i++ ) indices[i] = i;
		// allocate BVH root node
		int poolSize = N * 2 - 1;
		pool = new BVHNode[poolSize];
		root = pool[0];
		poolPtr = 2; 
		// subdivide root node
		root.first = 0;
		root.count = N;
		//root->bounds = CalculateBounds( primitives, root->first, root->count );
		Subdivide(&root);
	}

	void BVH::Subdivide(BVHNode* node)
	{
		//if (count < 3) return;
		node->left = &pool[poolPtr++];
		node->right = &pool[poolPtr++];
		Partition(node);
		Subdivide(node->left);
		Subdivide(node->right);
		node->isLeaf = false;
	}

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

}