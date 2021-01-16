#pragma once
namespace lh2core
{
	template<bool backCulling>
	[[nodiscard]]
	bool BVH2::IntersectRayBVHInternal(const Ray& r, RayMeshInterceptInfo& hitInfo, const int nodeId) const
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

#ifdef MEASURE_BVH
		int nodeIntersections = 0;
		int aabbIntersections = 0;
		int triangleIntersections = 0;
#endif

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			const BVHNode& node = pool[stack[stackCurr--]]; // Get from stack and pop

#ifdef MEASURE_BVH
			++nodeIntersections;
			++aabbIntersections;
#endif

			if (!TestAABBIntersection(r, node.bounds, invDir))
			{
				// No intersection
				continue;
			}

			if (node.IsLeaf())
			{
#ifdef MEASURE_BVH
				++aabbIntersections;
#endif
				if (!TestAABBIntersectionBounds(r, node.bounds, invDir, -kEps, hitInfo.triIntercept.t))
				{
					// No intersection
					// Not sure why it is faster when repeating the test
					continue;
				}

				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = node.count + node.FirstPrimitive();
				tempHitInfo.Reset();

				for (int i = node.FirstPrimitive(); i < triMax; i++)
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

#ifdef MEASURE_BVH
		printf("BVH2 Intersect nodes=%d/%d aabbIntersects=%d triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif

		return hit;
	}

	template<bool backCulling>
	[[nodiscard]]
	void BVH2::IntersectRayBVHInternal(const RayPacket& p, const Frustum& f, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize], const int nodeId) const
	{
		assert(poolSize > 0);
		assert(1 <= nodeId && nodeId < poolSize);
		assert(mesh.IsValid());
		assert(mesh.vcount / 3 <= INT_MAX);

		StackNode stack[64]; // if we have more than 64 levels in a tree, we have more than 4GB of triangles and we are going to have a bad time
		int stackCurr = 0;

		StackNode root;
		root.nodeId = nodeId;
		stack[stackCurr] = root;

		// Algo Info
		int vPos;
		int tIndex;

		Ray r;
		float3 invDir;
		int isDirNeg[3];
		uint ia = 0;

		RayTriangleInterceptInfo tempHitInfo;

		int rayIndices[p.kPacketSize];
		for (int i = 0; i < p.kPacketSize; ++i)
		{
			rayIndices[i] = i;
		}

		StackNode currNode = stack[stackCurr--];

		while (true)
		{

			ia = PartRays(p, f, pool[currNode.nodeId].bounds, rayIndices, currNode.ia);

			if (!pool[currNode.nodeId].IsLeaf())
			{
				stack[++stackCurr] = StackNode(pool[currNode.nodeId].LeftChild(), ia);
				stack[++stackCurr] = StackNode(pool[currNode.nodeId].RightChild(), ia);
			}
			else
			{
				const int triMax = pool[currNode.nodeId].count + pool[currNode.nodeId].FirstPrimitive();

				for (int i = pool[currNode.nodeId].FirstPrimitive(); i < triMax; i++)
				{
					tIndex = indices[i];
					vPos = tIndex * 3;
						
					if (TestFrustumAABBTriangle(f, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2]))
					{
						for (int j = 0; j < ia; ++j)
						{
							p.GetRay(r, rayIndices[j]);

							invDir = make_float3(r.InverseDirection());

							// Trick from http://www.pbr-book.org/3ed-2018/Primitives_and_Intersection_Acceleration/Bounding_Volume_Hierarchies.html#BVHAccel::splitMethod
							isDirNeg[0] = r.direction.x > 0;
							isDirNeg[1] = r.direction.y > 0;
							isDirNeg[2] = r.direction.z > 0;
								
							if (interceptRayTriangle<backCulling>(r, mesh.vertices[vPos + 0], mesh.vertices[vPos + 1], mesh.vertices[vPos + 2], tempHitInfo))
							{
								if (tempHitInfo < hitInfo[rayIndices[j]].triIntercept)
								{
									hitInfo[rayIndices[j]].meshId = mesh.meshID;
									tempHitInfo.CopyTo(hitInfo[rayIndices[j]].triIntercept);
									hitInfo[rayIndices[j]].triId = tIndex;
								}
							}
						}
					}
				}
			}
			if (stackCurr < 0)
			{
				break;
			}
			currNode = stack[stackCurr--]; // Get from stack and pop
		}
	}

	template <bool backCulling>
	[[nodiscard]]
	bool BVH2::DepthRayBVHInternal(const Ray& r, const int meshId, const int triId, const float tD, const int nodeId) const
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

#ifdef MEASURE_BVH
		int nodeIntersections = 0;
		int aabbIntersections = 0;
		int triangleIntersections = 0;
#endif

		while (stackCurr >= 0)
		{
			assert(stackCurr < 64);
			const BVHNode& node = pool[stack[stackCurr--]];

#ifdef MEASURE_BVH
			++nodeIntersections;
			++aabbIntersections;
#endif

			if (!TestAABBIntersection(r, node.bounds, invDir))
			{
				continue;
			}

			if (node.IsLeaf())
			{
#ifdef MEASURE_BVH
				++aabbIntersections;
#endif
				if (!TestAABBIntersectionBounds(r, node.bounds, invDir, -kEps, tD))
				{
					// No intersection
					// Not sure why it is faster when repeating the test
					continue;
				}

				assert(mesh.vcount % 3 == 0); // No weird meshes
				const int triMax = node.count + node.FirstPrimitive();
				tempHitInfo.Reset();

				for (int i = node.FirstPrimitive(); i < triMax; i++)
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
							printf("BVH2 Depth nodes=%d/%d aabbIntersects=%d triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif
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

#ifdef MEASURE_BVH
		printf("BVH2 Depth nodes=%d/%d aabbIntersects=%d triangleIntersects=%d/%d\n", nodeIntersections, poolPtr, aabbIntersections, triangleIntersections, mesh.vcount / 3);
#endif
		return false;
	}


	template <bool backCulling>
	[[nodiscard]]
	bool interceptRayScene(const Ray& r, const vector<BVH2>& meshes, RayMeshInterceptInfo& hitInfo)
	{
		RayMeshInterceptInfo tempInfo;
		bool hit = false;
		hitInfo.Reset();
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH2& m = meshes[i];
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
	void interceptRayScene(const RayPacket& p, const vector<BVH2>& meshes, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize])
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH2& m = meshes[i];
			m.interceptRayScene<backCulling>(p, hitInfo);
		}
	}

	template <bool backCulling>
	[[nodiscard]]
	bool depthRayScene(const Ray& r, const vector<BVH2>& meshes, const int instId, const int meshId, const int triId, const float tD)
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH2& m = meshes[i];
			if (m.DepthRayBVH<backCulling>(r, meshId, triId, tD))
			{
				return true;
			}
		}

		return false;
	}
}
