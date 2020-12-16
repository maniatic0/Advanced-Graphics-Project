#pragma once

namespace lh2core
{
	template<bool backCulling>
	[[nodiscard]]
	bool BVH4::IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hitInfo) const
	{
		// Intersection
		const float3 invDir = make_float3(r.InverseDirection());

		// Traversal
		const uchar signs = ((r.direction.x >= 0) << 0) | ((r.direction.y >= 0) << 1) | ((r.direction.z >= 0) << 2);
		uchar orderMask;
		int childrenCluster;

		// Hit info
		RayTriangleInterceptInfo tempHitInfo;
		bool hit = false;
		int vPos;
		int tIndex;

		int stackPtr = 0;
		int stackPos = 0 * 2;
		uint stack[256]; // ParentnodeId + clusterPosition + oldNodeId 
		stack[stackPos + 0] = rootIndex;
		stack[stackPos + 1] = rootClusterIndex;

		while (stackPtr >= 0)
		{
			assert(stackPtr < 256);
			stackPos = stackPtr * 2;
			--stackPtr;

			const int parentNodeId = stack[stackPos + 0];
			const int nodeClusterId = stack[stackPos + 1];

			const BVH4Node &node = pool[parentNodeId];

			// New Parent
			const aabb& clusterBounds = node.bounds[nodeClusterId];
			const BVH4NodeCluster &cluster = node.children[nodeClusterId];

			if (!cluster.IsActive() || !TestAABBIntersection(r, clusterBounds, invDir))
			{
				// No intersection
				continue;
			}

			if (cluster.IsLeaf())
			{
				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = cluster.GetPrimitiveCount() + cluster.GetFirstPrimitive();
				tempHitInfo.Reset();

				for (int i = cluster.GetFirstPrimitive(); i < triMax; i++)
				{
					tIndex = indices[i];
					vPos = tIndex * 3;
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

			orderMask = compactLUT[orderLUT[signs][cluster.GetPerm()]][cluster.GetActiveChildren()];

			childrenCluster = cluster.GetChildrenCluster();
			// low bits are the last nodes
			for (int i = 0; i < 4; ++i)
			{
				++stackPtr;
				stackPos = stackPtr * 2;

				// parentNodeId + clusterPosition + oldNodeId
				stack[stackPos + 0] = childrenCluster; // Children cluster
				stack[stackPos + 1] = (int)(orderMask & 0x03);
				orderMask >>= 2;
			}

		}

		return hit;
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
		int childrenCluster;

		// Hit info
		int vPos;
		int tIndex;
		const bool sameMesh = mesh.meshID == meshId;

		int stackPtr = 0;
		int stackPos = 0 * 2;
		uint stack[256]; // ParentnodeId + clusterPosition + oldNodeId 
		stack[stackPos + 0] = rootIndex;
		stack[stackPos + 1] = rootClusterIndex;

		while (stackPtr >= 0)
		{
			assert(stackPtr < 256);
			stackPos = stackPtr * 2;
			--stackPtr;

			const int parentNodeId = stack[stackPos + 0];
			const int nodeClusterId = stack[stackPos + 1];

			const BVH4Node& node = pool[parentNodeId];

			// New Parent
			const aabb& clusterBounds = node.bounds[nodeClusterId];
			const BVH4NodeCluster& cluster = node.children[nodeClusterId];

			if (!cluster.IsActive() || !TestAABBIntersection(r, clusterBounds, invDir))
			{
				// No intersection
				continue;
			}

			if (cluster.IsLeaf())
			{
				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = cluster.GetPrimitiveCount() + cluster.GetFirstPrimitive();

				for (int i = cluster.GetFirstPrimitive(); i < triMax; i++)
				{
					tIndex = indices[i];
					vPos = tIndex * 3;
					if ((!sameMesh || tIndex != triId) && depthRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tD))
					{
						if (!sameMesh)
						{
							return true;
						}
					}
				}
				continue;
			}

			orderMask = compactLUT[orderLUT[signs][cluster.GetPerm()]][cluster.GetActiveChildren()];

			childrenCluster = cluster.GetChildrenCluster();
			// low bits are the last nodes
			for (int i = 0; i < 4; ++i)
			{
				++stackPtr;
				stackPos = stackPtr * 2;

				// parentNodeId + clusterPosition + oldNodeId
				stack[stackPos + 0] = childrenCluster; // Children cluster
				stack[stackPos + 1] = (int)(orderMask & 0x03);
				orderMask >>= 2;
			}

		}

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
}