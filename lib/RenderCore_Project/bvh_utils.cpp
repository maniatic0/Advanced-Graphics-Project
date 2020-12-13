#include "core_settings.h"
#include "bvh_utils.h"

namespace lh2core
{
	inline float getTriangleCenterAxis(const float4& a, const float4& b, const float4& c, const int axis)
	{
		switch (axis)
		{
		case 0:
			return (a.x + b.x + c.x) / 3.0f;
		case 1:
			return (a.y + b.y + c.y) / 3.0f;
		case 2:
			return (a.z + b.z + c.z) / 3.0f;
		default:
			assert(false); // What
			break;
		}
		assert(false); // What
		return 0;
	}
	
	void BVH::ConstructBVH()
	{
		assert(mesh.vcount >= 0);
		assert(mesh.vcount % 3 == 0);
		assert(mesh.IsValid());
		assert(mesh.vcount / 3 <= INT_MAX); // 4 GB of triangles means a bad time

		const int tcount = mesh.vcount / 3;
		// create index array
		indices = std::make_unique<uint[]>(tcount);
		for (int i = 0; i < tcount; i++) {
			indices[i] = i; 
		}
		// allocate BVH root node
		poolSize = tcount * 2 - 1;
		pool = std::make_unique<BVHNode[]>(poolSize);
		assert(reinterpret_cast<uintptr_t>(pool.get()) % alignof(BVHNode) == 0);
		root = &pool[1];
		poolPtr = 2; 
		// subdivide root node
		root->leftFirst = 0;
		root->count = tcount;
		CalculateBounds(root);
		Subdivide(1);
	}

	void BVH::CalculateBounds(BVHNode* node)
	{
		aabb& bounds = node->bounds;
		bounds.Reset();
		
		assert(node->IsLeaf());
		assert(node->count > 0);
		const int tStart = node->FirstPrimitive();
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

	void BVH::Subdivide(int nodeId)
	{
		assert(mesh.IsValid());
		assert(mesh.vcount / 3 <= INT_MAX); // 4 GB of triangles means a bad time

		int stack[64]; // if we have more than 64 levels in a tree, we have more than 4GB of triangles and we are going to have a bad time
		int stackCurr = 0;
		stack[stackCurr] = nodeId;

		int first;
		int count;
		int leftChild;
		int rightChild;
		int p;
		BucketInfo buckets[nBuckets];
		float cost[nBuckets - 1];

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			nodeId = stack[stackCurr--];

			assert(1 <= nodeId && nodeId < poolSize);
			BVHNode& node = pool[nodeId];
			assert(node.IsLeaf());
			assert(node.count > 0);

			if (node.count < 5) {
				Partition(&node);
				continue;
			}

			first = node.FirstPrimitive();
			count = node.count;
			const int splitAxis = node.bounds.LongestAxis();
			const float4* vertices = mesh.vertices.get();

			for (int i = 0; i < nBuckets; i++)
			{
				buckets[i].count = 0;
				buckets[i].bounds.Reset();
			}

			for (int i = first; i < count; ++i) {
				int triangleIndex = indices[i] * 3;
				const float centroid = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);

				int b = nBuckets * ((centroid - node.bounds.Minimum(splitAxis)) / (node.bounds.Maximum(splitAxis) - node.bounds.Minimum(splitAxis)));
				if (b == nBuckets) b = nBuckets - 1;
				buckets[b].count++;
				buckets[b].bounds.Grow(vertices[triangleIndex + 0]);
				buckets[b].bounds.Grow(vertices[triangleIndex + 1]);
				buckets[b].bounds.Grow(vertices[triangleIndex + 2]);
			}

			aabb b0, b1;
			for (int i = 0; i < nBuckets - 1; ++i) {
				b0.Reset();
				b1.Reset();
				int count0 = 0, count1 = 0;
				for (int j = 0; j <= i; ++j) {
					b0.Grow(buckets[j].bounds);
					count0 += buckets[j].count;
				}
				for (int j = i + 1; j < nBuckets; ++j) {
					b1.Grow(buckets[j].bounds);
					count1 += buckets[j].count;
				}
				cost[i] = .125f + (count0 * b0.Area() +
					count1 * b1.Area()) / node.bounds.Area();
			}

			float minCost = cost[0];
			int minCostSplitBucket = 0;
			for (int i = 1; i < nBuckets - 1; ++i) {
				if (cost[i] < minCost) {
					minCost = cost[i];
					minCostSplitBucket = i;
				}
			}

			if (count < 5 && minCost >= count) {
				Partition(&node);
				continue;
			}
			
			leftChild = poolPtr++;
			rightChild = poolPtr++;
			p = PartitionBucket(&node, minCostSplitBucket, buckets);
			assert(p >= first);
			assert(p + 1 < first + count);

			node.SetLeftChild(leftChild);

			BVHNode& left = pool[leftChild];
			BVHNode& right = pool[rightChild];

			// Prepare Children
			left.leftFirst = first;
			left.count = p - first + 1; // hi - lo + 1 = count
			CalculateBounds(&left);

			right.leftFirst = left.leftFirst + left.count;
			right.count = count - left.count;
			CalculateBounds(&right);
			assert(count == left.count + right.count);
			assert(right.leftFirst == left.leftFirst + left.count);

			stack[++stackCurr] = rightChild;
			stack[++stackCurr] = leftChild;
		}
	}

	int BVH::Partition(BVHNode* node)
	{
		assert(node->IsLeaf());
		assert(node->count > 0);
		// Hoare's partition https://en.wikipedia.org/wiki/Quicksort
		const int splitAxis = node->bounds.LongestAxis();
		const float4* vertices = mesh.vertices.get();

		// Both inclusive
		int lo = node->FirstPrimitive();
		int hi = lo + node->count - 1;
		int temp;

		int triangleIndex = indices[(hi + lo) / 2] * 3;

		const float pivot = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);

		int i = lo - 1;
		int j = hi + 1;

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

			temp = indices[i];
			indices[i] = indices[j];
			indices[j] = temp;
		}
		assert(false);
		return -1;
	}

	int BVH::PartitionBucket(BVHNode* node, int bucketId, BucketInfo* bucketInfo) 
	{
		assert(node->IsLeaf());
		assert(node->count > 0);
		// Hoare's partition https://en.wikipedia.org/wiki/Quicksort
		const int splitAxis = node->bounds.LongestAxis();
		const float4* vertices = mesh.vertices.get();

		// Both inclusive
		int lo = node->FirstPrimitive();
		int hi = lo + node->count - 1;
		int temp;
		int b = 0;

		int triangleIndex = indices[(hi + lo) / 2] * 3;

		int i = lo - 1;
		int j = hi + 1;

		while (true)
		{
			do
			{
				++i;
				int triangleIndex = indices[i] * 3;
				const float centroid = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);
				b = nBuckets * ((centroid - node->bounds.Minimum(splitAxis)) / (node->bounds.Maximum(splitAxis) - node->bounds.Minimum(splitAxis)));
				if (b == nBuckets) b = nBuckets - 1;
			} while (b < bucketId);

			do
			{
				--j;
				int triangleIndex = indices[j] * 3;
				const float centroid = getTriangleCenterAxis(vertices[triangleIndex + 0], vertices[triangleIndex + 1], vertices[triangleIndex + 2], splitAxis);
				b = nBuckets * ((centroid - node->bounds.Minimum(splitAxis)) / (node->bounds.Maximum(splitAxis) - node->bounds.Minimum(splitAxis)));
				if (b == nBuckets) b = nBuckets - 1;
			} while (b > bucketId);

			if (i >= j)
			{
				return j;
			}

			temp = indices[i];
			indices[i] = indices[j];
			indices[j] = temp;
		}
		assert(false);
		return -1;
	}

}