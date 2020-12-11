#pragma once

namespace lh2core
{
	struct alignas(64) BVHNode // Note: aligned to cache line
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
		inline bool LeftChild() const { assert(!IsLeaf());  return -leftFirst; }
		inline bool RightChild() const { assert(!IsLeaf());  return LeftChild() + 1; }

		inline bool FirstPrimitive() const { assert(IsLeaf()); return leftFirst; }

		
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


		uint* indices;
		BVHNode* pool;
		BVHNode* root;
		uint poolPtr;
		Mesh mesh;

	public:

		inline BVH() : indices(nullptr), pool(nullptr), root(nullptr), poolPtr(0) {}

		BVH(const BVH&) = default;
		BVH& operator=(const BVH&) = default;
		
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
		void Subdivide(BVHNode* node);
		int Partition(BVHNode* node);

		template<bool backCulling>
		[[nodiscard]]
		inline bool IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hit) const
		{
			return IntersectRayBVHInternal<backCulling>(r, hit, 0);
		}

		template <bool backCulling>
		[[nodiscard]]
		inline bool DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const
		{
			return DepthRayBVHInternal<backCulling>(r, meshId, triId, tD, 0);
		}

		inline ~BVH()
		{
			root = nullptr;
			delete[] pool;
			pool = nullptr;
			delete[] indices;
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
