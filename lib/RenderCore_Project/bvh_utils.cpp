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

		const float4* vertices = mesh.vertices.get();

		const int tcount = mesh.vcount / 3;
		// create index and bounds array
		indices = std::make_unique<uint[]>(tcount);
		primitiveBounds = std::make_unique<aabb[]>(tcount);
		int triangleIndex;
		for (int i = 0; i < tcount; i++) {
			indices[i] = i;
			triangleIndex = i * 3;
			primitiveBounds[i].Reset();
			primitiveBounds[i].Grow(vertices[triangleIndex + 0]);
			primitiveBounds[i].Grow(vertices[triangleIndex + 1]);
			primitiveBounds[i].Grow(vertices[triangleIndex + 2]);
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
		Subdivide(1);
	}

	void BVH::CalculateBounds(BVHNode* node, aabb &centroidBounds)
	{
		aabb& bounds = node->bounds;
		bounds.Reset();
		centroidBounds.Reset();
		
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
			const aabb& triangleBounds = primitiveBounds[index];
			bounds.Grow(triangleBounds);
			centroidBounds.Grow(triangleBounds.Center());
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
		int tMax;
		int leftChild;
		int rightChild;
		int p;

		aabb centroidBounds;

		// Binning
		int splitAxis;
		float invCentroidsExtent;
		int b;
		aabb b0, b1;
		int count0, count1;
		float minCost;
		int minCostSplitBucket;
		BucketInfo buckets[nBuckets];
		float cost[nBuckets - 1];

		const float4* vertices = mesh.vertices.get();

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			nodeId = stack[stackCurr--];

			assert(1 <= nodeId && nodeId < poolSize);
			BVHNode& node = pool[nodeId];
			assert(node.IsLeaf());
			assert(node.count > 0);

			first = node.FirstPrimitive();
			count = node.count;
			tMax = first + count;
			
			
			CalculateBounds(&node, centroidBounds);
			splitAxis = centroidBounds.LongestAxis();

			if (node.count < 5 || centroidBounds.Maximum(splitAxis) == centroidBounds.Minimum(splitAxis)) {
				// Too small or no volume
				Partition(&node, splitAxis);
				continue;
			}
			
			// SAH + Binning based on http://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies.html#fragment-CreateleafmonoBVHBuildNode-0

			for (int i = 0; i < nBuckets; ++i)
			{
				buckets[i].count = 0;
				buckets[i].bounds.Reset();
			}
			
			invCentroidsExtent = 1.0f / (centroidBounds.Maximum(splitAxis) - centroidBounds.Minimum(splitAxis));
			for (int i = first; i < tMax; ++i) {
				// Bucket partition (like bucket sorting)
				const aabb& centroid = primitiveBounds[indices[i]];
				b = nBucketsFloat * ((centroid.Center(splitAxis) - centroidBounds.Minimum(splitAxis)) * invCentroidsExtent);
				if (b == nBuckets) { b = nBuckets - 1; }
				++buckets[b].count;
				buckets[b].bounds.Grow(centroid);
			}

			// Left
			b0.Reset();
			count0 = 0;
			for (int i = 0; i < nBuckets - 1; ++i) {
				// Calculate Left and Right Cost
				b0.Grow(buckets[i].bounds);
				count0 += buckets[i].count;

				// Right
				b1.Reset();
				count1 = 0;
				for (int j = i + 1; j < nBuckets; ++j) {
					b1.Grow(buckets[j].bounds);
					count1 += buckets[j].count;
				}
				cost[i] = .125f + (count0 * b0.Area() + count1 * b1.Area()) / node.bounds.Area();
			}

			minCost = std::numeric_limits<float>::infinity();
			minCostSplitBucket = -1;
			for (int i = 0; i < nBuckets - 1; ++i) {
				if (cost[i] < minCost) {
					minCost = cost[i];
					minCostSplitBucket = i;
				}
			}
			assert(minCostSplitBucket != -1);

			if (count < 5 && minCost >= (float)count) {
				// Bad Node
				Partition(&node, splitAxis);
				continue;
			}
			
			leftChild = poolPtr++;
			rightChild = poolPtr++;
			p = PartitionBucket(&node, minCostSplitBucket, centroidBounds, splitAxis);
			assert(p >= first);
			assert(p + 1 < first + count);

			node.SetLeftChild(leftChild);

			BVHNode& left = pool[leftChild];
			BVHNode& right = pool[rightChild];

			// Prepare Children
			left.leftFirst = first;
			left.count = p - first + 1; // hi - lo + 1 = count

			right.leftFirst = left.leftFirst + left.count;
			right.count = count - left.count;
			assert(count == left.count + right.count);
			assert(right.leftFirst == left.leftFirst + left.count);

			stack[++stackCurr] = rightChild;
			stack[++stackCurr] = leftChild;
		}
	}

	int BVH::Partition(BVHNode* node, int splitAxis)
	{
		assert(node->IsLeaf());
		assert(node->count > 0);
		// Hoare's partition https://en.wikipedia.org/wiki/Quicksort
		const float4* vertices = mesh.vertices.get();

		// Both inclusive
		int lo = node->FirstPrimitive();
		int hi = lo + node->count - 1;
		int temp;

		const float pivot = primitiveBounds[indices[(hi + lo) / 2]].Center(splitAxis);

		int i = lo - 1;
		int j = hi + 1;

		float center = 0;
		while (true)
		{
			do
			{
				++i;
				center = primitiveBounds[indices[i]].Center(splitAxis);
			} while (center < pivot);
			
			do
			{
				--j;
				center = primitiveBounds[indices[j]].Center(splitAxis);
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

	int BVH::PartitionBucket(BVHNode* node, int bucketId, const aabb& centroidBounds, int splitAxis)
	{
		assert(node->IsLeaf());
		assert(node->count > 0);
		// Hoare's partition https://en.wikipedia.org/wiki/Quicksort
		const float invExtent = 1.0f / (centroidBounds.Maximum(splitAxis) - centroidBounds.Minimum(splitAxis));
		const float4* vertices = mesh.vertices.get();

		// Both inclusive
		int lo = node->FirstPrimitive();
		int hi = lo + node->count - 1;
		int temp;
		int b = 0;

		int i = lo - 1;
		int j = hi + 1;

		while (true)
		{
			do
			{
				++i;
				const float centroid = primitiveBounds[indices[i]].Center(splitAxis);
				b = nBucketsFloat * ((centroid - centroidBounds.Minimum(splitAxis)) * invExtent);
				if (b == nBuckets) { b = nBuckets - 1; }
			} while (b < bucketId);

			do
			{
				--j;
				const float centroid = primitiveBounds[indices[j]].Center(splitAxis);
				b = nBucketsFloat * ((centroid - centroidBounds.Minimum(splitAxis)) * invExtent);
				if (b == nBuckets) { b = nBuckets - 1; }
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