#pragma once
namespace lh2core
{
	template<bool backCulling>
	[[nodiscard]]
	bool BVH::IntersectRayBVHInternal(const Ray& r, RayMeshInterceptInfo& hitInfo, const int nodeId) const
	{
		// Slow version (also do this with stack)
		const BVHNode& node = pool[nodeId];

		if (!r.TestAABBIntersection(node.bounds))
		{
			return false;
		}

		if (node.IsLeaf())
		{
			assert(mesh.vcount % 3 == 0); // No weird meshes
			const int triMax = node.count + node.FirstPrimitive();
			RayTriangleInterceptInfo tempHitInfo;
			bool hit = false;
			int vPos;

			for (int i = node.FirstPrimitive(); i < triMax; i++)
			{
				vPos = i * 3;
				if (interceptRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tempHitInfo))
				{
					if (tempHitInfo < hitInfo.triIntercept)
					{
						hit = true;
						tempHitInfo.CopyTo(hitInfo.triIntercept);
						hitInfo.triId = i;
					}
				}
			}

			if (!hit)
			{
				return false;
			}


			hitInfo.meshId = mesh.meshID;
			return true;
		}
		else
		{
			return IntersectRayBVHInternal<backCulling>(r, hitInfo, node.LeftChild()) || IntersectRayBVHInternal<backCulling>(r, hitInfo, node.RightChild());
		}
	}

	template <bool backCulling>
	[[nodiscard]]
	bool BVH::DepthRayBVHInternal(const Ray& r, const int meshId, const int triId, const float tD, const int nodeId) const
	{
		// Slow version (also do this with stack)
		const BVHNode& node = pool[nodeId];
		const bool sameMesh = mesh.meshID == meshId;

		if (!r.TestAABBIntersection(node.bounds))
		{
			return false;
		}

		if (node.IsLeaf())
		{
			assert(mesh.vcount % 3 == 0); // No weird meshes
			const int triMax = node.count + node.FirstPrimitive();
			RayTriangleInterceptInfo tempHitInfo;
			bool hit = false;
			int vPos;

			for (int i = node.FirstPrimitive(); i < triMax; i++)
			{
				vPos = i * 3;
				if ((!sameMesh || i != triId) && depthRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tD))
				{
					if (!sameMesh)
					{
						return true;
					}
				}
			}

			return false;
		}
		else
		{
			return DepthRayBVHInternal<backCulling>(r, meshId, triId, tD, node.LeftChild()) || DepthRayBVHInternal<backCulling>(r, meshId, triId, tD, node.RightChild());
		}
	}


	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& r, const vector<BVH>& meshes, RayMeshInterceptInfo& hitInfo)
	{
		RayMeshInterceptInfo tempInfo;
		bool hit = false;
		hitInfo.Reset();
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH& m = meshes[i];
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
	bool depthRayScene(const Ray& r, const vector<BVH>& meshes, const int instId, const int meshId, const int triId, const float tD)
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH& m = meshes[i];
			if (m.DepthRayBVH<backCulling>(r, meshId, triId, tD))
			{
				return true;
			}
		}

		return false;
	}
}
