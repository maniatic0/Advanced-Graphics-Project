namespace lh2core
{
	struct BVHNode
	{
	public:
		// constructor / destructor
		BVHNode() = default;

		float3 lb; // Minimum (bottom left) coordinate of the bounding box
		float3 rt; // Maxmimum (top right) coordinate of the bounding box
		bool isLeaf;
		BVHNode* left;
		BVHNode* right;
		int first, count;

		bool IntersectRay(const Ray& r, float& t);
	};

	class BVH
	{
	public:
		uint* indices;
		BVHNode* pool;
		BVHNode root;
		uint poolPtr;

		void ConstructBVH(vector<Mesh> meshes);
		void Subdivide(BVHNode* node);
		void Partition(BVHNode* node) {};
	};

	

}
