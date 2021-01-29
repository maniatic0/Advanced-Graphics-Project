#pragma once

namespace lh2core
{
	typedef unsigned int uint;
	typedef unsigned char uchar;

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
		uchar active;
		uchar perm;
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

		inline void SetPerm(int axis1, int axis2, int axis3, int topologyId)
		{
			assert(0 <= axis1 && axis1 < 4);
			assert(0 <= axis2 && axis2 < 4);
			assert(0 <= axis3 && axis3 < 4);
			assert(0 <= topologyId && topologyId < 6);
			perm = (uchar)axis1 + (uchar)axis2 * 3 + (uchar)axis3 * 9 + (uchar)topologyId * 27;
		}

		inline uchar GetPerm() const { assert(IsActive()); return perm; }

		inline bool IsLeaf() const { assert(IsActive());  return active == 0; }
		inline bool HasChildren() const { return !IsLeaf(); }

		inline int GetChildrenCluster() const { assert(!IsLeaf()); return -child; }
		inline void SetChildrenCluster(int id) { assert(id > 0);  child = -id; }

		inline void SetActiveChildren(const int childrenId[4]) { 
			assert(IsActive()); 
			active = 0; 
			active |= ((childrenId[(int)NodeClusterName::LeftLeft] != -1) << 0);
			active |= ((childrenId[(int)NodeClusterName::LeftRight] != -1) << 1);
			active |= ((childrenId[(int)NodeClusterName::RightLeft] != -1) << 2);
			active |= ((childrenId[(int)NodeClusterName::RightRight] != -1) << 3);
		}
		inline uchar GetActiveChildren() const { assert(!IsLeaf()); return active; }

		inline int GetFirstPrimitive() const { assert(IsLeaf()); return child; }
		inline int GetPrimitiveCount() const { assert(IsLeaf()); return perm; }

		inline void SetFirstPrimitive(int id) { assert(IsActive()); assert(id >= 0);  child = id; active = 0; }
		inline void SetPrimitiveCount(int count) { assert(IsActive());  assert(0 < count && count < 256);  perm = (unsigned char)count; active = 0; }

		friend class BVH4;
	};

	struct ALIGN(256) BVH4Node // Note: aligned to cache line
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

		unique_ptr<uint[]> indices;
		unique_ptr<aabb[]> primitiveBounds;
		unique_ptr<BVH4Node[]> pool;
		BVH4Node* root;
		uint poolPtr;
		Mesh mesh;

		int poolSize;

		// Paper Helpers

		// order generation macro
#define P(n0, n1, n2, n3) ((n0 << 6) | (n1 << 4) | (n2 << 2) | n3)

		// arbitrary order index to order mapping
		static constexpr uchar indexToOrderLUT[24] = {
			P(0, 1, 2, 3), P(0, 2, 1, 3), P(2, 0, 1, 3), P(1, 0, 2, 3), P(1, 2, 0, 3), P(2, 1, 0, 3),
			P(0, 1, 3, 2), P(0, 2, 3, 1), P(2, 0, 3, 1), P(1, 0, 3, 2), P(1, 2, 3, 0), P(2, 1, 3, 0),
			P(0, 3, 1, 2), P(0, 3, 2, 1), P(2, 3, 0, 1), P(1, 3, 0, 2), P(1, 3, 2, 0), P(2, 3, 1, 0),
			P(3, 0, 1, 2), P(3, 0, 2, 1), P(3, 2, 0, 1), P(3, 1, 0, 2), P(3, 1, 2, 0), P(3, 2, 1, 0)
		};
#undef P

		// LUTs to fill
		static uchar orderLUT[8][136];
		static uchar compactLUT[24][16];
		static uchar bitCountLUT[16];
		static uchar bitFirstLUT[16];

		// helper LUT
		static constexpr uchar shiftLUT[4] = { 6, 4, 2, 0 };

		// helper function
		static uchar orderToIndex(const uchar order);
			
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

		inline aabb GetBounds() const
		{
			return pool[rootIndex].bounds[rootClusterIndex];
		}

		void ConstructBVH(BVH2&& original);

		static constexpr int rootIndex = 0;
		static constexpr int rootClusterIndex = 0;

		template<bool backCulling>
		[[nodiscard]]
		bool IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hit) const;

		template <bool backCulling>
		void IntersectRayBVH(const RayPacket& p, const Frustum& f, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize]) const;

		template <bool backCulling>
		[[nodiscard]]
		bool DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const;


		static void PrepareBVH4Tables();

		static inline int GetChildOrderMask(int orderMask, int index)
		{
			assert(0 <= index && index < 4);
			assert(0 <= ((int)((orderMask >> 2 * index) & 0x03)) && ((int)((orderMask >> 2 * index) & 0x03)) < 4);
			// low bits are the last nodes
			return (int)((orderMask >> 2 * index) & 0x03);
		}
	};

	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& r, const vector<BVH4>& meshes, RayMeshInterceptInfo& hitInfo);

	template <bool backCulling>
	void interceptRayScene(const RayPacket& p, const vector<BVH4>& meshes, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize]);

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH4>& meshes, const int instId, const int meshId, const int triId, const float tD);


}

#include "bvh4_utils_impl.h"
