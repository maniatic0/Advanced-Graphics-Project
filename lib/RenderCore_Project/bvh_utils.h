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
		inline bool HasChildren() const { return leftFirst < 0; }
		inline int LeftChild() const { assert(!IsLeaf());  return -leftFirst; }
		inline int RightChild() const { assert(!IsLeaf());  return LeftChild() + 1; }

		inline int FirstPrimitive() const { assert(IsLeaf()); return leftFirst; }

		inline void SetLeftChild(int nodeId) { assert(nodeId > 0); leftFirst = -nodeId; }
		
		friend class BVH;
	};

	class BVH
	{
	private:

		template<bool backCulling>
		[[nodiscard]]
		bool IntersectRayBVHInternal(const Ray& r, RayMeshInterceptInfo& hit, const int nodeId) const;

		template<bool backCulling>
		[[nodiscard]]
		bool DepthRayBVHInternal(const Ray& r, const int meshId, const int triId, const float tD, const int nodeId) const;


		unique_ptr<uint[]> indices;
		unique_ptr<BVHNode[]> pool;
		BVHNode* root;
		uint poolPtr;
		Mesh mesh;

		int poolSize;

	public:

		inline BVH() : indices(nullptr), pool(nullptr), root(nullptr), poolPtr(0), poolSize(0) {}

		inline BVH(const BVH&) = delete;
		BVH& operator=(const BVH&) = delete;
		
		BVH(BVH&& other) noexcept = default;
		BVH& operator=(BVH&& other) noexcept = default;

		inline const Mesh& GetMesh() const {
			return mesh;
		}

		inline Mesh &GetMesh() {
			return mesh;
		}


		void ConstructBVH();
		void CalculateBounds(BVHNode* node);
		void Subdivide(int nodeId);
		int Partition(BVHNode* node);

		template<bool backCulling>
		[[nodiscard]]
		inline bool IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hit) const
		{
			return IntersectRayBVHInternal<backCulling>(r, hit, 1);
		}

		template <bool backCulling>
		[[nodiscard]]
		inline bool DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const
		{
			return DepthRayBVHInternal<backCulling>(r, meshId, triId, tD, 1);
		}

	};

	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& v, const vector<BVH>& meshes, RayMeshInterceptInfo& hitInfo);

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH>& meshes, const int instId, const int meshId, const int triId, const float tD);
}

#include "bvh_utils_impl.h"
