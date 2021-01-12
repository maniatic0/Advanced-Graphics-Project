#pragma once



namespace lh2core
{
	enum class BVH_Type : uchar {
		BVH2 = 0,
		BVH4,
		Count
	};


	class BVH {
	private:
		BVH_Type type;


		std::variant<unique_ptr<BVH2>, unique_ptr<BVH4>> bvhPtr;

	public:

		inline BVH() : type(BVH_Type::Count), bvhPtr() { };

		inline BVH(const BVH&) = delete;
		BVH& operator=(const BVH&) = delete;

		BVH(BVH&& other) noexcept = default;
		BVH& operator=(BVH&& other) noexcept = default;

		inline void ConstructBVH(BVH_Type newType, Mesh&& newMesh)
		{
			assert(BVH_Type::BVH2 <= newType && newType < BVH_Type::Count);
			switch (type)
			{
			case lh2core::BVH_Type::BVH2:
			{
				unique_ptr<BVH2>& bvh2 = get<unique_ptr<BVH2>>(bvhPtr);
				bvh2 = nullptr;
			}
			break;
			case lh2core::BVH_Type::BVH4:
			{
				unique_ptr<BVH4>& bvh4 = get<unique_ptr<BVH4>>(bvhPtr);
				bvh4 = nullptr;
			}
			break;
			case lh2core::BVH_Type::Count:
				break;
			default:
				break;
			}

			type = newType;
			switch (type)
			{
			case lh2core::BVH_Type::BVH2:
			{
				Timer timer;
				timer.reset();
				bvhPtr.emplace<unique_ptr<BVH2>>(make_unique<BVH2>());
				unique_ptr<BVH2>& bvh = get<unique_ptr<BVH2>>(bvhPtr);
				bvh->ConstructBVH(std::forward<Mesh>(newMesh));
				printf("\tbuilt BVH2 in %5.3fs\n", timer.elapsed());
			}
			break;
			case lh2core::BVH_Type::BVH4:
			{
				BVH2 temp;
				Timer timer;
				timer.reset();
				
				temp.ConstructBVH(std::forward<Mesh>(newMesh));
				printf("\tbuilt temp BVH2 in %5.3fs\n", timer.elapsed());

				timer.reset();
				bvhPtr.emplace<unique_ptr<BVH4>>(make_unique<BVH4>());
				unique_ptr<BVH4>& bvh = get<unique_ptr<BVH4>>(bvhPtr);
				bvh->ConstructBVH(std::forward<BVH2>(temp));
				printf("\tbuilt BVH4 in %5.3fs\n", timer.elapsed());
			}
			break;
			default:
				assert(false);
				break;
			}
		}

		inline const Mesh& GetMesh() const {
			return std::visit([](auto&& ptr) -> const Mesh& { assert(ptr != nullptr);  return ptr->GetMesh(); }, bvhPtr);
		}

		inline Mesh& GetMesh() {
			return std::visit([](auto&& ptr) -> Mesh& { assert(ptr != nullptr);  return ptr->GetMesh(); }, bvhPtr);
		}

		inline aabb GetBounds() const
		{
			return std::visit([](auto&& ptr) -> aabb { assert(ptr != nullptr);  return ptr->GetBounds(); }, bvhPtr);
		}

		template<bool backCulling>
		[[nodiscard]]
		inline bool IntersectRayBVH(const Ray& r, RayMeshInterceptInfo& hitInfo) const
		{
			return std::visit([&r, &hitInfo](auto&& ptr) -> bool { assert(ptr != nullptr);  return ptr->IntersectRayBVH<backCulling>(r, hitInfo); }, bvhPtr);
		}

		template<bool backCulling>
		inline void IntersectRayBVH(const RayPacket& p, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize]) const
		{
			std::visit([&p, &hitInfo](auto&& ptr) -> void { assert(ptr != nullptr); ptr->IntersectRayBVH<backCulling>(p, hitInfo); }, bvhPtr);
		}

		template <bool backCulling>
		[[nodiscard]]
		inline bool DepthRayBVH(const Ray& r, const int meshId, const int triId, const float tD) const
		{
			return std::visit([&r, meshId, triId, tD](auto&& ptr) -> bool { assert(ptr != nullptr);  return ptr->DepthRayBVH<backCulling>(r, meshId, triId, tD); }, bvhPtr);
		}
	};

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
	void interceptRayScene(const RayPacket& p, const vector<BVH>& meshes, RayMeshInterceptInfo hitInfo[RayPacket::kPacketSize])
	{
		for (size_t i = 0; i < meshes.size(); i++)
		{
			const BVH& m = meshes[i];
			m.IntersectRayBVH<backCulling>(p, hitInfo);
		}
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