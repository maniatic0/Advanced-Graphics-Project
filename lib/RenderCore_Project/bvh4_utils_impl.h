#pragma once

namespace lh2core
{
#define BVH4_USE_BOUNDS_TEST 1
	template<bool backCulling>
	[[nodiscard]]
	bool BVH4::IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hitInfo) const
	{
		// Intersection
		const float3 invDir = make_float3(r.InverseDirection());

		// Traversal
		const uchar signs = ((r.direction.x >= 0) << 0) | ((r.direction.y >= 0) << 1) | ((r.direction.z >= 0) << 2);
		uchar orderMask;

		// Hit info
		RayTriangleInterceptInfo tempHitInfo;
		bool hit = false;
		int vPos;
		int tIndex;

		const int rootAxis = pool[0].bounds[0].LongestAxis();
		const int rootPerm = (uchar)rootAxis + (uchar)rootAxis * 3 + (uchar)rootAxis * 9 + (uchar)0 * 27; // (uchar)axis1 + (uchar)axis2 * 3 + (uchar)axis3 * 9 + (uchar)topologyId * 27


		int stackPtr = 0;
		int stackPos = 0 * 3;
		uint stack[512]; // ParentnodeId + perm + activeMask
		stack[stackPos + 0] = rootIndex;
		stack[stackPos + 1] = rootPerm; // Perm for first child
		stack[stackPos + 2] = 1; // Only the first node is active for root

#ifdef MEASURE_BVH
		int nodeIntersections = 0;
		int aabbIntersections = 0;
		int triangleIntersections = 0;
#endif

		while (stackPtr >= 0)
		{
			assert(stackPtr < 512);

			stackPos = (stackPtr--) * 3;

			const int parentNodeId = stack[stackPos + 0];
			const int perm = stack[stackPos + 1];
			const int activeMask = stack[stackPos + 2];



			const BVH4Node& node = pool[parentNodeId];

			const int test = (int)TestAABB4Intersection(r, node.bounds, invDir) & activeMask;
			//const int test = (int)TestAABB4IntersectionBounds(r, node.bounds, invDir, -kEps, hitInfo.triIntercept.t) & activeMask;

#ifdef MEASURE_BVH
			++nodeIntersections;
			aabbIntersections += 4;
#endif

			orderMask = compactLUT[orderLUT[signs][perm]][activeMask];

			// low bits are the last nodes
			const int childCount = bitCountLUT[activeMask];
			for (int i = 0; i < childCount; ++i)
			{
				const uchar childId = (int)(orderMask & 0x03);
				orderMask >>= 2;
				const BVH4NodeCluster& cluster = node.children[childId];

				if (!(test & (1 << childId)))
				{
					// No intersection
					assert(!cluster.IsActive() || !(test & (1 << childId)));
					continue;
				}

				if (cluster.IsLeaf())
				{
#ifdef BVH4_USE_BOUNDS_TEST 
#ifdef MEASURE_BVH
					++aabbIntersections;
#endif
					if (!TestAABBIntersectionBounds(r, node.bounds[childId], invDir, -kEps, hitInfo.triIntercept.t))
					{
						// No intersection
						// Not sure why it is faster when repeating the test
						continue;
					}
#endif

					assert(mesh.vcount % 3 == 0); // No weird meshes
					const int triMax = cluster.GetPrimitiveCount() + cluster.GetFirstPrimitive();
					tempHitInfo.Reset();

					for (int i = cluster.GetFirstPrimitive(); i < triMax; i++)
					{
						tIndex = indices[i];
						vPos = tIndex * 3;

#ifdef MEASURE_BVH
						++triangleIntersections;
#endif

						if (interceptRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tempHitInfo))
						{
							if (tempHitInfo < hitInfo.triIntercept)
							{
								hit = true;
								hitInfo.meshId = mesh.meshID;
								tempHitInfo.CopyTo(hitInfo.triIntercept);
								hitInfo.triId = tIndex;
							}
						}
					}
					continue;
				}

				// New node to traverse
				stackPos = (++stackPtr) * 3;
				stack[stackPos + 0] = cluster.GetChildrenCluster(); // Children cluster
				stack[stackPos + 1] = cluster.GetPerm();
				stack[stackPos + 2] = cluster.GetActiveChildren();
			}

		}

#ifdef MEASURE_BVH
		printf("BVH4 Intersect nodes=%d/%d aabbIntersects=%d (4 at the same time + 1 extra for leaves) triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif
		return hit;
	}

	template <bool backCulling>
	void BVH4::IntersectRayBVH(const RayPacket& pR, const Frustum& f, RayMeshInterceptInfo hit[RayPacket::kPacketSize])
	{
		// Note that this could be replaced with an int offset to the base root
		struct StackNode {
			int nodeId;
			int childId;
		};

		struct StackElement {
			StackNode node;
			int rayId;
			float t;
		};

		// TODO: fill array of rays
		Ray rays[RayPacket::kPacketSize];
		float4 invDirs[RayPacket::kPacketSize];
		pR.InverseDirection(invDirs);
		const int maxActive = pR.maxActive;
		const int maxActiveDiv4 = (maxActive + 4 - 1) / 4;


		StackElement stack[RayPacket::kPacketSize * 32 * 4]; // In theory all the rays could put their kids here. So a BVH of depth 32, times 4 kids per level

		StackNode node;
		int stackPointer = 0;
		int activeRID = 0;
		float t = 0;

		// This is not written but it is probably what happens
		for (int i = 0; i < maxActive; i++)
		{
			stack[stackPointer++] = { {rootIndex , rootClusterIndex}, i, hit[i].triIntercept.t + kEps };
		}

		// This is not written but it is probably what happens
		goto LINE_42;

		while (true)
		{
		LINE_5:
			;;;

		LINE_42:
			while (--stackPointer >= 0)
			{
				assert(stackPointer >= 0);
				const StackElement& elem = stack[stackPointer];
				node = elem.node;
				activeRID = elem.rayId;
				t = elem.t;

				// This might be dangerous
				if (*reinterpret_cast<uint *>(&t) < *reinterpret_cast<uint *>(&hit[activeRID].triIntercept.t))
				{
					assert(t < hit[activeRID].triIntercept.t); // Just to be sure
					goto LINE_5;
				}

				for (int p = activeRID/4; p < maxActiveDiv4; p++)
				{
					assert(0 <= node.nodeId && node.nodeId < poolSize);
					assert(0 <= node.childId && node.childId < 4);
					const uint mask = Test4AABBIntersection(pool[node.nodeId].bounds[node.childId], &rays[4 * p], &invDirs[4 * p]);
				}
			}
			break;
		}


		// TODO: Have proper packet interception
		Ray r;
		for (size_t i = 0; i < RayPacket::kPacketSize; i++)
		{
			pR.GetRay(r, i);
			IntersectRayBVH<backCulling>(r, hit[i]);
		}
	}

	template <bool backCulling>
	[[nodiscard]]
	bool BVH4::DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const
	{
		// Intersection
		const float3 invDir = make_float3(r.InverseDirection());

		// Traversal
		const uchar signs = ((r.direction.x >= 0) << 0) | ((r.direction.y >= 0) << 1) | ((r.direction.z >= 0) << 2);
		uchar orderMask;

		// Hit info
		int vPos;
		int tIndex;
		const bool sameMesh = mesh.meshID == meshId;

		const int rootAxis = pool[0].bounds[0].LongestAxis();
		const int rootPerm = (uchar)rootAxis + (uchar)rootAxis * 3 + (uchar)rootAxis * 9 + (uchar)0 * 27; // (uchar)axis1 + (uchar)axis2 * 3 + (uchar)axis3 * 9 + (uchar)topologyId * 27


		int stackPtr = 0;
		int stackPos = 0 * 3;
		uint stack[512]; // ParentnodeId + perm + activeMask
		stack[stackPos + 0] = rootIndex;
		stack[stackPos + 1] = rootPerm; // Perm for first child
		stack[stackPos + 2] = 1; // Only the first node is active for root

#ifdef MEASURE_BVH
		int nodeIntersections = 0;
		int aabbIntersections = 0;
		int triangleIntersections = 0;
#endif

		while (stackPtr >= 0)
		{
			assert(stackPtr < 512);

			stackPos = (stackPtr--) * 3;

			const int parentNodeId = stack[stackPos + 0];
			const int perm = stack[stackPos + 1];
			const int activeMask = stack[stackPos + 2];

			const BVH4Node& node = pool[parentNodeId];			

			const int test = (int)TestAABB4Intersection(r, node.bounds, invDir) & activeMask;
			//const int test = (int)TestAABB4IntersectionBounds(r, node.bounds, invDir, -kEps, tD) & activeMask;

#ifdef MEASURE_BVH
			++nodeIntersections;
			aabbIntersections += 4;
#endif

			orderMask = compactLUT[orderLUT[signs][perm]][activeMask];

			// low bits are the last nodes
			const int childCount = bitCountLUT[activeMask];
			for (int i = 0; i < childCount; ++i)
			{
				const uchar childId = (int)(orderMask & 0x03);
				orderMask >>= 2;
				const BVH4NodeCluster& cluster = node.children[childId];

				if (!(test & (1 << childId)))
				{
					// No intersection
					assert(!cluster.IsActive() || !(test & (1 << childId)));
					continue;
				}

				if (cluster.IsLeaf())
				{
#ifdef BVH4_USE_BOUNDS_TEST
#ifdef MEASURE_BVH
					++aabbIntersections;
#endif
					if (!TestAABBIntersectionBounds(r, node.bounds[childId], invDir, -kEps, tD))
					{
						// No intersection
						// Not sure why it is faster when repeating the test
						continue;
					}
#endif

					assert(mesh.vcount % 3 == 0); // No weird meshes
					const int triMax = cluster.GetPrimitiveCount() + cluster.GetFirstPrimitive();

					for (int i = cluster.GetFirstPrimitive(); i < triMax; i++)
					{
						tIndex = indices[i];
						vPos = tIndex * 3;

#ifdef MEASURE_BVH
						++triangleIntersections;
#endif

						if ((!sameMesh || tIndex != triId) && depthRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tD))
						{
							if (!sameMesh)
							{
#ifdef MEASURE_BVH
								printf("BVH4 Depth nodes=%d/%d aabbIntersects=%d (4 at the same time + 1 for leaves) triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif
								return true;
							}
						}
					}
					continue;
				}

				// New node to traverse
				stackPos = (++stackPtr) * 3;
				stack[stackPos + 0] = cluster.GetChildrenCluster(); // Children cluster
				stack[stackPos + 1] = cluster.GetPerm();
				stack[stackPos + 2] = cluster.GetActiveChildren();
			}

		}

#ifdef MEASURE_BVH
		printf("BVH4 Depth nodes=%d/%d aabbIntersects=%d (4 at the same time + 1 for leaves) triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif
		return false;
	}


	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& r, const vector<BVH4>& meshes, RayMeshInterceptInfo& hitInfo)
	{
		RayMeshInterceptInfo tempInfo;
		bool hit = false;
		hitInfo.Reset();
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH4& m = meshes[i];
			if (m.IntersectRayBVH<backCulling>(r, tempInfo))
			{
				if (tempInfo < hitInfo)
				{
					hit = true;
					tempInfo.CopyTo(hitInfo);
				}
			}
		}
		return hit;
	}

	template <bool backCulling>
	void interceptRayScene(const RayPacket& p, const vector<BVH4>& meshes, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize])
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH4& m = meshes[i];
			m.interceptRayScene<backCulling>(p, hitInfo);
		}
	}

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH4>& meshes, const int instId, const int meshId, const int triId, const float tD)
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH4& m = meshes[i];
			if (m.DepthRayBVH<backCulling>(r, meshId, triId, tD))
			{
				return true;
			}
		}

		return false;
	}
#ifdef BVH4_USE_BOUNDS_TEST 
#undef BVH4_USE_BOUNDS_TEST
#endif

}