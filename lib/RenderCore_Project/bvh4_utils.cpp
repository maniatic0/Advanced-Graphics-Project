#include "core_settings.h"


namespace lh2core
{

	void BVH4::ConstructBVH(BVH&& original)
	{
		indices = std::forward<unique_ptr<uint[]>>(original.indices);
		primitiveBounds = std::forward<unique_ptr<aabb[]>>(original.primitiveBounds);
		mesh = std::forward<Mesh>(original.mesh);

		poolSize = original.poolSize / 4 + 1;
		pool = std::make_unique<BVH4Node[]>(poolSize);

		root = &pool[rootIndex];
		poolPtr = rootIndex + 1;

		int childrenId[4];
		BVHNode *childrenNode[4];


		int stackPtr = 0;
		int stackPos = 0 * 3;
		uint stack[512]; // ParentnodeId + clusterPosition + oldNodeId 
		stack[stackPos + 0] = rootIndex;
		stack[stackPos + 1] = rootClusterIndex;
		stack[stackPos + 2] = original.rootIndex;

		while (stackPtr >= 0)
		{
			assert(stackPtr < 512);
			stackPos = stackPtr * 3;
			--stackPtr;

			const int parentNodeId = stack[stackPos + 0];
			const int nodeClusterId = stack[stackPos + 1];
			const int oldNodeId = stack[stackPos + 2];

			BVH4Node& node = pool[parentNodeId];

			// New Parent
			aabb& clusterBounds = node.bounds[nodeClusterId];
			BVH4NodeCluster &cluster = node.children[nodeClusterId];

			cluster.SetIndex(nodeClusterId);

			// Old Parent
			const BVHNode &oldNode = original.pool[oldNodeId];
			clusterBounds = oldNode.GetBounds();

			if (oldNode.IsLeaf())
			{
				cluster.SetFirstPrimitive(oldNode.FirstPrimitive());
				cluster.SetPrimitiveCount(oldNode.Count());

				continue;
			}
			assert(poolPtr < poolSize);

			for (int i = 0; i < static_cast<int>(NodeClusterName::Count); i++)
			{
				childrenId[i] = -1;
				childrenNode[i] = nullptr;
			}

			childrenId[(int)NodeClusterName::LeftLeft] = oldNode.LeftChild();
			childrenId[(int)NodeClusterName::RightLeft] = oldNode.RightChild();

			childrenNode[(int)NodeClusterName::LeftLeft] = &original.pool[childrenId[(int)NodeClusterName::LeftLeft]];
			childrenNode[(int)NodeClusterName::RightLeft] = &original.pool[childrenId[(int)NodeClusterName::RightLeft]];

			if (!childrenNode[(int)NodeClusterName::LeftLeft]->IsLeaf())
			{
				// Replace Left Right
				childrenId[(int)NodeClusterName::LeftRight] = childrenNode[(int)NodeClusterName::LeftLeft]->RightChild();
				childrenNode[(int)NodeClusterName::LeftRight] = &original.pool[childrenId[(int)NodeClusterName::LeftRight]];

				// Replace Left Left
				childrenId[(int)NodeClusterName::LeftLeft] = childrenNode[(int)NodeClusterName::LeftLeft]->LeftChild();
				childrenNode[(int)NodeClusterName::LeftLeft] = &original.pool[childrenId[(int)NodeClusterName::LeftLeft]];
			}


			if (!childrenNode[(int)NodeClusterName::RightLeft]->IsLeaf())
			{
				// Replace Left Right
				childrenId[(int)NodeClusterName::RightRight] = childrenNode[(int)NodeClusterName::RightLeft]->RightChild();
				childrenNode[(int)NodeClusterName::RightRight] = &original.pool[childrenId[(int)NodeClusterName::RightRight]];

				// Replace Left Left
				childrenId[(int)NodeClusterName::RightLeft] = childrenNode[(int)NodeClusterName::RightLeft]->LeftChild();
				childrenNode[(int)NodeClusterName::RightLeft] = &original.pool[childrenId[(int)NodeClusterName::RightLeft]];
			}

			// Set active children
			cluster.SetActiveChildren(childrenId);
			cluster.SetChildrenCluster(poolPtr);

			for (int i = static_cast<int>(NodeClusterName::Count) - 1; 0 <= i ; --i)
			{
				++stackPtr;
				stackPos = stackPtr * 3;

				// parentNodeId + clusterPosition + oldNodeId
				stack[stackPos + 0] = poolPtr; // Children cluster
				stack[stackPos + 1] = i;
				stack[stackPos + 2] = childrenId[i];
			}

			++poolPtr;
		}
		
	}
}