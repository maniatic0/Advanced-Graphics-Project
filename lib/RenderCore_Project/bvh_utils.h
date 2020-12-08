namespace lh2core
{
	struct BVHNode
	{ 
	public:
		// No copy by accident
		BVHNode(const BVHNode&) = delete;
		BVHNode& operator=(const BVHNode&) = delete;

		float3 lb; // Minimum (bottom left) coordinate of the bounding box
		float3 rt; // Maxmimum (top right) coordinate of the bounding box
		bool isLeaf; 
		int left, right; 
		int first, count; 

	};

	bool IntersectRayBVHNode(const Ray& r, const BVHNode& node);

}
