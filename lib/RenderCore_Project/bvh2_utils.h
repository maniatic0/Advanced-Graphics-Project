#pragma once

namespace lh2core
{
	struct ALIGN(64) BVHNode // Note: aligned to cache line
	{
	private:
		aabb bounds;
		int leftFirst;
		int count;
	public:
		// constructor / destructor
		inline BVHNode() : leftFirst(0), count(-1) {};

		// No accidental copy
		BVHNode(const BVHNode&) = delete;
		BVHNode& operator=(const BVHNode&) = delete;

		inline bool IsLeaf() const { return leftFirst >= 0; }
		inline bool HasChildren() const { return !IsLeaf(); }
		inline int LeftChild() const { assert(!IsLeaf());  return -leftFirst; }
		inline int RightChild() const { assert(!IsLeaf());  return LeftChild() + 1; }

		inline int FirstPrimitive() const { assert(IsLeaf()); return leftFirst; }
		inline int Count() const { assert(IsLeaf()); return count; }

		inline void SetLeftChild(int nodeId) { assert(nodeId > 0); leftFirst = -nodeId; }

		inline aabb GetBounds() const { return bounds; }
		
		friend class BVH2;
	};

	constexpr int nBuckets = 12;
	constexpr float nBucketsFloat = nBuckets;

	struct ALIGN(16) BucketInfo {
	public:
		aabb bounds;
		int count = 0;
	};

	class BVH2
	{
	private:

		template<bool backCulling>
		[[nodiscard]]
		bool IntersectRayBVHInternal(const Ray& r, RayMeshInterceptInfo& hit, const int nodeId) const;

		template<bool backCulling>
		[[nodiscard]]
		bool DepthRayBVHInternal(const Ray& r, const int meshId, const int triId, const float tD, const int nodeId) const;


		unique_ptr<uint[]> indices;
		unique_ptr<aabb[]> primitiveBounds;
		unique_ptr<BVHNode[]> pool;
		BVHNode* root;
		uint poolPtr;
		Mesh mesh;

		int poolSize;

		void CalculateBounds(BVHNode* node, aabb& centroidBounds);
		void Subdivide(int nodeId);
		int Partition(BVHNode* node, int splitAxis);
		int PartitionBucket(BVHNode* node, int bucketId, const aabb &centroidBounds, int splitAxis);

	public:

		inline BVH2() : indices(nullptr), pool(nullptr), root(nullptr), poolPtr(0), poolSize(0) {}

		inline BVH2(const BVH2&) = delete;
		BVH2& operator=(const BVH2&) = delete;
		
		BVH2(BVH2&& other) noexcept = default;
		BVH2& operator=(BVH2&& other) noexcept = default;

		inline const Mesh& GetMesh() const {
			return mesh;
		}

		inline Mesh &GetMesh() {
			return mesh;
		}

		inline aabb GetBounds() const
		{
			return pool[rootIndex].bounds;
		}

		void ConstructBVH(Mesh&& newMesh);

		static constexpr int rootIndex = 1;

		template<bool backCulling>
		[[nodiscard]]
		inline bool IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hit) const
		{
			return IntersectRayBVHInternal<backCulling>(r, hit, rootIndex);
		}

		template <bool backCulling>
		[[nodiscard]]
		inline bool DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const
		{
			return DepthRayBVHInternal<backCulling>(r, meshId, triId, tD, rootIndex);
		}

		friend class BVH4;
	};

	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& v, const vector<BVH2>& meshes, RayMeshInterceptInfo& hitInfo);

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH2>& meshes, const int instId, const int meshId, const int triId, const float tD);
}

#include "bvh2_utils_impl.h"
