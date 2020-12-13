#pragma once
namespace lh2core
{
	template<bool backCulling>
	[[nodiscard]]
	bool BVH::IntersectRayBVHInternal(const Ray& r, RayMeshInterceptInfo& hitInfo, const int nodeId) const
	{
		assert(poolSize > 0);
		assert(1 <= nodeId && nodeId < poolSize);
		assert(mesh.IsValid());
		assert(mesh.vcount / 3 <= INT_MAX);

		int stack[64]; // if we have more than 64 levels in a tree, we have more than 4GB of triangles and we are going to have a bad time
		int stackCurr = 0;
		stack[stackCurr] = nodeId;

		const float3 invDir = make_float3(r.InverseDirection());
		int isDirNeg[3] = { r.direction.x > 0, r.direction.y > 0, r.direction.z > 0 }; // Trick from http://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies.html#BVHAccel::splitMethod


		// Algo Info
		RayTriangleInterceptInfo tempHitInfo;
		bool hit = false;
		int vPos;
		int tIndex;

		int splitAxis;

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			const BVHNode& node = pool[stack[stackCurr--]]; // Get from stack and pop

			if (!TestAABBIntersection(r, node.bounds, invDir))
			{
				// No intersection
				continue;
			}

			if (node.IsLeaf())
			{
				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = node.count + node.FirstPrimitive();
				tempHitInfo.Reset();

				for (int i = node.FirstPrimitive(); i < triMax; i++)
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
			else
			{
				splitAxis = node.bounds.LongestAxis();

				if (isDirNeg[splitAxis])
				{
					// Ray is negative in axis, test first left then right
					stack[++stackCurr] = node.RightChild();
					stack[++stackCurr] = node.LeftChild();
				}
				else
				{
					// Ray is positive in axis, test first right then left
					stack[++stackCurr] = node.LeftChild();
					stack[++stackCurr] = node.RightChild();
				}
				continue;
			}
		}


		return hit;
	}

	template <bool backCulling>
	[[nodiscard]]
	bool BVH::DepthRayBVHInternal(const Ray& r, const int meshId, const int triId, const float tD, const int nodeId) const
	{
		assert(poolSize > 0);
		assert(1 <= nodeId && nodeId < poolSize);
		assert(mesh.IsValid());
		assert(mesh.vcount / 3 <= INT_MAX);

		int stack[64]; // if we have more than 64 levels in a tree, we have more than 4GB of triangles and we are going to have a bad time
		int stackCurr = 0;
		stack[stackCurr] = nodeId;

		const float3 invDir = make_float3(r.InverseDirection());
		int isDirNeg[3] = { r.direction.x > 0, r.direction.y > 0, r.direction.z > 0 }; // Trick from http://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies.html#BVHAccel::splitMethod

		const bool sameMesh = mesh.meshID == meshId;

		RayTriangleInterceptInfo tempHitInfo;
		int vPos;
		int tIndex;

		int splitAxis;

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			const BVHNode& node = pool[stack[stackCurr--]];

			if (!TestAABBIntersection(r, node.bounds, invDir))
			{
				return false;
			}

			if (node.IsLeaf())
			{
				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = node.count + node.FirstPrimitive();
				tempHitInfo.Reset();

				for (int i = node.FirstPrimitive(); i < triMax; i++)
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
			else
			{
				splitAxis = node.bounds.LongestAxis();

				if (isDirNeg[splitAxis])
				{
					// Ray is negative in axis, test first left then right
					stack[++stackCurr] = node.RightChild();
					stack[++stackCurr] = node.LeftChild();
				}
				else
				{
					// Ray is positive in axis, test first right then left
					stack[++stackCurr] = node.LeftChild();
					stack[++stackCurr] = node.RightChild();
				}
				
				continue;
			}
		}

		return false;
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
