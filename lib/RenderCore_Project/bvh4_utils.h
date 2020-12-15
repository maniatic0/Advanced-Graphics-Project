#pragma once

namespace lh2core
{
	enum class NodeClusterName : int
	{
		LeftLeft = 0,
		LeftRight = 1,
		RightLeft = 2,
		RightRight = 3,
		Count = 4
	};


	struct ALIGN(8) BVH4NodeCluster
	{
	private:
		int child;
		signed char index;
		unsigned char active;
		unsigned char perm;
	public:

		inline BVH4NodeCluster() : active(0), perm(0), index(-1), child(0) {};

		// No accidental copy
		BVH4NodeCluster(const BVH4NodeCluster&) = delete;
		BVH4NodeCluster& operator=(const BVH4NodeCluster&) = delete;

		inline bool IsActive() const {
			return index >= 0;
		}

		inline signed char Index() {
			assert(IsActive()); return index;
		}

		inline void SetIndex(int clusterId) {
			assert(!IsActive());
			index = static_cast<signed char>(clusterId);
		}

		inline bool IsLeaf() const { assert(IsActive());  return active == 0; }
		inline bool HasChildren() const { return !IsLeaf(); }

		inline int ChildrenCluster() const { assert(!IsLeaf()); return -child; }
		inline void SetChildrenCluster(int id) { assert(id > 0);  child = -id; }
		inline void SetActiveChildren(const int childrenId[4]) { 
			assert(IsActive()); 
			active = 0; 
			active |= (childrenId[(int)NodeClusterName::LeftLeft] != -1);
			active |= ((childrenId[(int)NodeClusterName::LeftRight] != -1) << 1);
			active |= ((childrenId[(int)NodeClusterName::RightLeft] != -1) << 2);
			active |= ((childrenId[(int)NodeClusterName::RightRight] != -1) << 3);
		}

		inline int GetFirstPrimitive() const { assert(IsLeaf()); return child; }
		inline int GetPrimitiveCount() const { assert(IsLeaf()); return perm; }

		inline void SetFirstPrimitive(int id) { assert(IsActive()); assert(id >= 0);  child = id; active = 0; }
		inline void SetPrimitiveCount(int count) { assert(IsActive());  assert(0 < count && count < 256);  perm = (unsigned char)count; active = 0; }

		friend class BVH4;
	};

	struct ALIGN(64) BVH4Node // Note: aligned to cache line
	{
	private:
		aabb bounds[4];
		BVH4NodeCluster children[4];
	public:
		// constructor / destructor
		inline BVH4Node() {};

		// No accidental copy
		BVH4Node(const BVH4Node&) = delete;
		BVH4Node& operator=(const BVH4Node&) = delete;

		friend class BVH4;
	};

	class BVH4
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
		unique_ptr<BVH4Node[]> pool;
		BVH4Node* root;
		uint poolPtr;
		Mesh mesh;

		int poolSize;

	public:

		inline BVH4() : indices(nullptr), primitiveBounds(nullptr), pool(nullptr), root(nullptr), poolPtr(0), poolSize(0) {}

		inline BVH4(const BVH4&) = delete;
		BVH4& operator=(const BVH4&) = delete;

		BVH4(BVH4&& other) noexcept = default;
		BVH4& operator=(BVH4&& other) noexcept = default;

		inline const Mesh& GetMesh() const {
			return mesh;
		}

		inline Mesh& GetMesh() {
			return mesh;
		}

		void ConstructBVH(BVH&& original);

		static constexpr int rootIndex = 0;
		static constexpr int rootClusterIndex = 0;

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
	bool interceptRayScene(const Ray& v, const vector<BVH4>& meshes, RayMeshInterceptInfo& hitInfo);

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH4>& meshes, const int instId, const int meshId, const int triId, const float tD);


}

#include "bvh4_utils_impl.h"
