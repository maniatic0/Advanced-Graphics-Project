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
	void BVH4::IntersectRayBVH(const RayPacket& pR, const Frustum& f, RayMeshInterceptInfo hit[RayPacket::kPacketSize]) const
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

		const Ray* rays = pR.rays;

		float3 invDirs[RayPacket::kPacketSize];
		pR.InverseDirection(invDirs);

		uchar signs[RayPacket::kPacketSize];
		pR.GetSigns(signs);

		const int maxActive = pR.maxActive;
		const int maxActiveDiv4 = (maxActive + 4 - 1) / 4;
		constexpr int maxPacketDiv4 = (RayPacket::kPacketSize + 4 - 1) / 4;

		constexpr size_t stackSize = RayPacket::kPacketSize * 32 * 4; // In theory all the rays could put their kids here. So a BVH of depth 32, times 4 kids per level
		StackElement stack[stackSize]; 

		StackNode node;
		int stackPointer = 0;
		int activeRID = 0;
		int savedRID = 0;
		int o = -1;
		int order = 0;
		float t = 0;
		uchar activeMask = 0;
		uchar hitMsk = 0;
		float t4[4]{ -1, -1, -1, -1 };
		int cnt = -1;

		BVH4Node* nodeInfo = nullptr;
		BVH4NodeCluster* nodeClusterInfo = nullptr;
		aabb* nodeBounds = nullptr;
		const Ray* intRay = nullptr;
		float3* intInvDir = nullptr;
		uchar* sign = nullptr;

		// Leaves intersection info
		RayTriangleInterceptInfo tempHitInfo;
		RayMeshInterceptInfo *hitInfo = nullptr;
		int vPos;
		int tIndex;

		// This is not written but it is probably what happens
		for (int p = 0; p < maxActiveDiv4; p++)
		{
			const int p4 = 4 * p;
			assert(0 <= p4 && p4 + 3 < RayPacket::kPacketSize);
			const uint mask = Test4AABBIntersectionDistance(root->bounds[rootClusterIndex], &rays[p4], &invDirs[p4], t4);
			if (mask != 0)
			{
				assert(0 <= mask && mask < 16);

				for (int i = 0; i < 4; i++)
				{
					if ((mask & (1 << i)) != 0)
					{
						assert(hit[p4 + i].triIntercept.t >= 0);
						assert(stackPointer < stackSize);
						stack[stackPointer++] = { {rootIndex , rootClusterIndex}, p4 + i, t4[i]};
					}
				}
			}
		}

		// This is not written but it is probably what happens
		goto LINE_42;

		// TODO: Check pixels that return black
		while (true)
		{
		LINE_5:
			// Node Stuff
			assert(nodeInfo != nullptr);
			assert(nodeClusterInfo != nullptr);
			assert(nodeBounds != nullptr);
			// Ray Stuff
			assert(intRay != nullptr);
			assert(intInvDir != nullptr);
			assert(sign != nullptr);
			assert(hitInfo != nullptr);
			if (!nodeClusterInfo->IsLeaf())
			{
				hitMsk = TestAABB4IntersectionDistance(*intRay, pool[nodeClusterInfo->GetChildrenCluster()].bounds, *intInvDir, t4);
				activeMask = hitMsk & nodeClusterInfo->GetActiveChildren();
				if (activeMask == 0)
				{
					goto LINE_42;
				}
				savedRID = activeRID;
				const uchar orderIdx = orderLUT[*sign][nodeClusterInfo->GetPerm()];
				order = compactLUT[orderIdx][activeMask];
				o = 0;
				for (cnt = bitCountLUT[activeMask] - 1; cnt >= 0; cnt--)
				{
					o = GetChildOrderMask(order, cnt);
					if ((hitMsk & (1 << o)) != 0)
					{
						goto LINE_31;
					}

					for (int p = activeRID / 4; p < maxActiveDiv4; p++)
					{
						const int p4 = 4 * p;
						assert(0 <= p4 && p4 + 3 < RayPacket::kPacketSize);
						const uint mask = Test4AABBIntersection(*nodeBounds, &rays[p4], &invDirs[p4]);
						if (mask != 0)
						{
							assert(0 <= mask && mask < 16);
							activeRID = p4 + bitFirstLUT[mask];
							assert(0 <= activeRID && activeRID < RayPacket::kPacketSize);

							if (activeRID >= maxActive)
							{
								break;
							}

							assert(0 <= activeRID && activeRID < maxActive);
							intRay = &rays[activeRID];
							intInvDir = &invDirs[activeRID];
							sign = &signs[activeRID];
							hitInfo = &hit[activeRID];

							goto LINE_31;
						}
					}
				}
				goto LINE_42;
			LINE_31:
				for (int i = 0; i < cnt; i++)
				{
					const int so = GetChildOrderMask(order, i);
					assert((hitMsk & (1 << so)) != 0);
					assert(t4[so] >= 0);
					assert(0 <= stackPointer && stackPointer < stackSize);
					stack[stackPointer++] = { {nodeClusterInfo->GetChildrenCluster(), so}, savedRID, t4[so] };
				}

				// Node Stuff
				node.nodeId = nodeClusterInfo->GetChildrenCluster();
				assert(0 <= node.nodeId && node.nodeId < poolSize);
				nodeInfo = &pool[node.nodeId];

				assert(o == GetChildOrderMask(order, cnt));
				node.childId = o;
				assert(0 <= node.childId && node.childId < 4);
				nodeClusterInfo = &nodeInfo->children[node.childId];
				nodeBounds = &nodeInfo->bounds[node.childId];

				continue;
			}
			else
			{
				// Leaf Intersection
				// Check if it actually useful to test all triangles
				if (TestAABBIntersectionBounds(*intRay, *nodeBounds, *intInvDir, -kEps, hitInfo->triIntercept.t))
				{
					assert(mesh.vcount % 3 == 0); // No weird meshes
					const int triMax = nodeClusterInfo->GetPrimitiveCount() + nodeClusterInfo->GetFirstPrimitive();
					tempHitInfo.Reset();

					for (int i = nodeClusterInfo->GetFirstPrimitive(); i < triMax; i++)
					{
						tIndex = indices[i];
						vPos = tIndex * 3;

						if (interceptRayTriangle<backCulling>(*intRay, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tempHitInfo))
						{
							if (tempHitInfo < hitInfo->triIntercept)
							{
								hitInfo->meshId = mesh.meshID;
								tempHitInfo.CopyTo(hitInfo->triIntercept);
								hitInfo->triId = tIndex;
							}
						}
					}
				}
			}
		LINE_42:
			while (--stackPointer >= 0)
			{
				assert(stackPointer >= 0);
				const StackElement& elem = stack[stackPointer];

				// Node Stuff
				node = elem.node;
				assert(0 <= node.nodeId && node.nodeId < poolSize);
				nodeInfo = &pool[node.nodeId];
				assert(0 <= node.childId && node.childId < 4);
				nodeClusterInfo = &nodeInfo->children[node.childId];
				nodeBounds = &nodeInfo->bounds[node.childId];
				
				// Ray Stuff
				activeRID = elem.rayId;
				assert(0 <= activeRID && activeRID < maxActive);
				intRay = &rays[activeRID];
				intInvDir = &invDirs[activeRID];
				sign = &signs[activeRID];
				hitInfo = &hit[activeRID];

				// Distance
				t = elem.t;
				assert(t >= 0);

				// This might be dangerous
				if (*reinterpret_cast<uint *>(&t) < *reinterpret_cast<uint *>(&hit[activeRID].triIntercept.t))
				{
					assert(t < hit[activeRID].triIntercept.t); // Just to be sure
					goto LINE_5;
				}
				assert(!(t < hit[activeRID].triIntercept.t)); // Just to be sure

				for (int p = activeRID / 4; p < maxActiveDiv4; p++)
				{
					const int p4 = 4 * p;
					assert(0 <= p4 && p4 + 3 < RayPacket::kPacketSize);
					const uint mask = Test4AABBIntersection(*nodeBounds, &rays[p4], &invDirs[p4]);
					if (mask != 0)
					{
						assert(0 <= mask && mask < 16);
						activeRID = p4 + bitFirstLUT[mask];
						assert(0 <= activeRID && activeRID < RayPacket::kPacketSize);

						if (activeRID >= maxActive)
						{
							break;
						}

						assert(0 <= activeRID && activeRID < maxActive);
						intRay = &rays[activeRID];
						intInvDir = &invDirs[activeRID];
						sign = &signs[activeRID];
						hitInfo = &hit[activeRID];

						goto LINE_5;
					}
				}
			}
			break;
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