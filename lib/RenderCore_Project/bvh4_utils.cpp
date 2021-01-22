#include "core_settings.h"


namespace lh2core
{
	// Paper Functions
	uchar BVH4::orderLUT[8][136];
	uchar BVH4::compactLUT[24][16];
	uchar BVH4::bitCountLUT[16];

	uchar BVH4::orderToIndex(const uchar order) {
		uchar idx;
		for (idx = 0; idx < 24; idx++)
		{
			if (indexToOrderLUT[idx] == order)
			{
				break;
			}
		}
		return idx;
	}

	void BVH4::PrepareBVH4Tables()
	{
		// Fill bit count for 4 childs
		for (int i = 0; i < 16; i++)
		{
			bitCountLUT[i] = (i & 1) + ((i >> 1) & 1) + ((i >> 2) & 1) + ((i >> 3) & 1);
		}


		// fill compactLUT
		for (int orderIdx = 0; orderIdx < 24; orderIdx++) {
			for (int mask = 0; mask < 16; mask++) {
				const uchar order = indexToOrderLUT[orderIdx];
				int corder = 0;
				for (int n = 3; n >= 0; n--) {
					const int idx = (order >> (2 * n)) & 0x3;
					if (mask & (1 << idx)) {
						corder <<= 2;
						corder |= idx;
					}
				}
				compactLUT[orderIdx][mask] = corder;
			}
		}

		// fill orderLUT
		for (uint rs = 0; rs < 8; rs++)
		{
			const uint signs[3] = { rs & 1, (rs >> 1) & 1, (rs >> 2) };

			// balanced tree
			for (uint s0 = 0; s0 < 3; s0++) {
				for (uint s1 = 0; s1 < 3; s1++) {
					for (uint s2 = 0; s2 < 3; s2++) {
						const uint axis[2] = { s1, s2 };
						const uint node0 = (2 * signs[s0]) ^ signs[axis[signs[s0]]];
						const uint node1 = node0 ^ 1;
						const uint node2 = (2 * (signs[s0] ^ 1)) ^ signs[axis[(signs[s0] ^ 1)]];
						const uint node3 = node2 ^ 1;
						const uint mask = (node0 << 6) | (node1 << 4) | (node2 << 2) | node3;
						orderLUT[rs][s0 * 9 + s1 * 3 + s2] = orderToIndex(mask);
					}
				}
			}

			// unbalanced tree type 1
			for (uint s0 = 0; s0 < 3; s0++) {
				for (uint s1 = 0; s1 < 3; s1++) {
					for (uint s2 = 0; s2 < 3; s2++) {
						const uint node0 = 3 * signs[s0];
						const uint node1 = (signs[s0] ^ 1) + 2 * signs[s1];
						const uint node2 = (signs[s0] ^ 1) + (signs[s1] ^ 1) + signs[s2];
						const uint node3 = (signs[s0] ^ 1) + (signs[s1] ^ 1) + (signs[s2] ^ 1);

						const uint mask = (0 << shiftLUT[node0])  | (1 << shiftLUT[node1]) |
							(2 << shiftLUT[node2]) | (3 << shiftLUT[node3]);
						orderLUT[rs][s0 * 9 + s1 * 3 + s2 + 27 * 1] = orderToIndex(mask);
					}
				}
			}

			// unbalanced tree type 2
			for (uint s0 = 0; s0 < 3; s0++) {
				for (uint s1 = 0; s1 < 3; s1++) {
					for (uint s2 = 0; s2 < 3; s2++) {
						const uint node0 = signs[s0] + 2 * signs[s1];
						const uint node1 = signs[s0] + (signs[s1] ^ 1) + signs[s2];
						const uint node2 = signs[s0] + (signs[s1] ^ 1) + (signs[s2] ^ 1);
						const uint node3 = 3 * (signs[s0] ^ 1);
						const uint mask = (0 << shiftLUT[node0]) | (1 << shiftLUT[node1]) | 
							(2 << shiftLUT[node2]) | (3 << shiftLUT[node3]);
						orderLUT[rs][s0 * 9 + s1 * 3 + s2 + 27 * 2] = orderToIndex(mask);
					}
				}
			}

			// unbalanced tree type 3
			for (uint s0 = 0; s0 < 3; s0++) {
				for (uint s1 = 0; s1 < 3; s1++) {
					for (uint s2 = 0; s2 < 3; s2++) {
						const uint node0 = 3 * signs[s0];
						const uint node1 = (signs[s0] ^ 1) + signs[s1] + signs[s2];
						const uint node2 = (signs[s0] ^ 1) + signs[s1] + (signs[s2] ^ 1);
						const uint node3 = (signs[s0] ^ 1) + 2 * (signs[s1] ^ 1);
						const uint mask = (0 << shiftLUT[node0]) | (1 << shiftLUT[node1]) |
							(2 << shiftLUT[node2]) | (3 << shiftLUT[node3]);
						orderLUT[rs][s0 * 9 + s1 * 3 + s2 + 27 * 3] = orderToIndex(mask);
					}
				}
			}

			// unbalanced tree type 4
			for (uint s0 = 0; s0 < 3; s0++) {
				for (uint s1 = 0; s1 < 3; s1++) {
					for (uint s2 = 0; s2 < 3; s2++) {
						const uint node0 = signs[s0] + signs[s1] + signs[s2];
						const uint node1 = signs[s0] + signs[s1] + (signs[s2] ^ 1);
						const uint node2 = signs[s0] + 2 * (signs[s1] ^ 1);
						const uint node3 = 3 * (signs[s0] ^ 1);
						const uint mask = (0 << shiftLUT[node0]) | (1 << shiftLUT[node1]) |
							(2 << shiftLUT[node2]) | (3 << shiftLUT[node3]);
						orderLUT[rs][s0 * 9 + s1 * 3 + s2 + 27 * 4] = orderToIndex(mask);
					}
				}
			}
		}
	}

	// End of paper functions

	void BVH4::ConstructBVH(BVH2&& original)
	{
		assert(original.indices != nullptr);
		assert(original.primitiveBounds != nullptr);
		indices = std::forward< unique_ptr<uint[]> >(original.indices);
		primitiveBounds = std::forward<unique_ptr<aabb[]>>(original.primitiveBounds);
		mesh = std::forward<Mesh>(original.mesh);

		poolSize = original.poolSize / 4 + 1;
		pool = std::make_unique<BVH4Node[]>(poolSize);

		root = &pool[rootIndex];
		poolPtr = rootIndex + 1;

		int childrenId[4];
		BVHNode *childrenNode[4];
		int axis1, axis2, axis3;


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
			axis1 = clusterBounds.LongestAxis();

			if (oldNode.IsLeaf())
			{
				cluster.SetFirstPrimitive(oldNode.FirstPrimitive());
				cluster.SetPrimitiveCount(oldNode.Count());

				continue;
			}
			assert((int)poolPtr < poolSize);

			for (int i = 0; i < static_cast<int>(NodeClusterName::Count); i++)
			{
				childrenId[i] = -1;
				childrenNode[i] = nullptr;
			}

			
			childrenId[(int)NodeClusterName::LeftLeft] = oldNode.LeftChild();
			childrenId[(int)NodeClusterName::RightLeft] = oldNode.RightChild();

			childrenNode[(int)NodeClusterName::LeftLeft] = &original.pool[childrenId[(int)NodeClusterName::LeftLeft]];
			childrenNode[(int)NodeClusterName::RightLeft] = &original.pool[childrenId[(int)NodeClusterName::RightLeft]];

			axis2 = childrenNode[(int)NodeClusterName::LeftLeft]->GetBounds().LongestAxis();
			axis3 = childrenNode[(int)NodeClusterName::RightLeft]->GetBounds().LongestAxis();

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
			cluster.SetPerm(axis1, axis2, axis3, 0); // We are building from a BVH2 so this is a balanced split 

			for (int i = static_cast<int>(NodeClusterName::Count) - 1; 0 <= i ; --i)
			{
				if (childrenId[i] == -1) {
					continue;
				}
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