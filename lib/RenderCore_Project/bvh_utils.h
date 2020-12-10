namespace lh2core
{
	struct BVHNode
	{
	public:
		// constructor / destructor
		BVHNode() : leftFirst(0), count(-1) {};

		aabb bounds;
		int leftFirst;
		int count;

		inline bool IsLeaf() { return leftFirst >= 0; }
		inline bool HasChildren() { return leftFirst < 0; }
		inline bool LeftChild() { assert(!IsLeaf());  return -leftFirst; }
		inline bool RightChild() { assert(!IsLeaf());  return LeftChild() + 1; }

		inline bool FirstPrimitive() { assert(IsLeaf()); return leftFirst; }

		//bool IntersectRay(const Ray& r, float& t);
	};

	class BVH
	{
	public:
		uint* indices;
		BVHNode *pool;
		BVHNode *root;
		uint poolPtr;
		Mesh mesh;

		// Note: This is going to steal the pointers
		void ConstructBVH(Mesh&& mesh);
		void CalculateBounds(BVHNode* node);
		void Subdivide(BVHNode* node);
		int Partition(BVHNode* node);
	};

	

}
