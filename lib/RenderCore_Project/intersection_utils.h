#pragma once


namespace lh2core
{

//  +-----------------------------------------------------------------------------+
//  |  Mesh                                                                       |
//  |  Minimalistic mesh storage.                                           LH2'19|
//  +-----------------------------------------------------------------------------+
class Mesh
{
public:
	unique_ptr<float4[]> vertices = nullptr;							// vertex data received via SetGeometry
	unique_ptr<CoreTri[]> triangles = nullptr;							// 'fat' triangle data
	int vcount = 0;									// vertex count
	int meshID = 0;									// mesh id

	// No accidental copy
	Mesh(const Mesh&) = delete;
	Mesh& operator=(const Mesh&) = delete;

	inline Mesh() noexcept  : vertices(nullptr), triangles(nullptr), vcount(-1), meshID(-1) {};

	inline Mesh(Mesh&& other) noexcept = default;
	inline Mesh& operator=(Mesh&& other) noexcept = default;

	inline bool IsValid() const { return vcount > 0 && vcount % 3 == 0 && meshID >= 0 && vertices.get() != nullptr && triangles.get() != nullptr; }
};

//  +-----------------------------------------------------------------------------+
//  |  Ray                                                                        |
//  |  Minimalistic ray storage.											  2020|
//  +-----------------------------------------------------------------------------+
struct ALIGN(16) Ray {
public:
	float4 origin;
	float4 direction;

	/// <summary>
	/// Copy Ray
	/// </summary>
	/// <returns>New Copied Ray</returns>
	inline Ray Copy() const
	{
		return Ray(origin, direction);
	}

	inline Ray() : origin (make_float4(0)), direction(make_float4(1, 0, 0, 0)) { }

	inline Ray(const float4 &ori, const float4 &dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = ori;
		direction = dir;
	}

	inline Ray(const float3& ori, const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		origin = make_float4(ori);
		direction = make_float4(dir);
	}

	inline void SetOrigin(const float4& ori)
	{
		origin = ori;
	}
	inline void SetOrigin(const float3& ori)
	{
		origin = make_float4(ori);
	}

	inline void SetDirection(const float4& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		direction = dir;
	}
	inline void SetDirection(const float3& dir)
	{
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		direction = make_float4(dir);
	}

	/// <summary>
	/// Evaluate the ray for certain t
	/// </summary>
	/// <param name="t">Parameter</param>
	/// <returns>Position</returns>
	inline float4 Evaluate(const float t) const
	{
		return origin + (t * direction);
	}
	
	inline float4 InverseDirection() const {
		return 1.0f / direction;
	}
};

struct Frustum {
public:
	/// <summary>
	/// Number of Planes that form the Frustum
	/// </summary>
	static constexpr int kNumberOfPlanes = 4;

	/// <summary>
	///	Normals that describe the four planes of the frustum
	/// First 3 elements describe the normal vector, 4th element is b_i
	/// </summary>
	float4 normals[kNumberOfPlanes];
};

struct ALIGN(16) RayPacket {
public:
	/// <summary>
	/// Number of Rays in Packet
	/// </summary>
	static constexpr uint kPacketSize = 64;

	/// <summary>
	/// Rays
	/// </summary>
	Ray rays[kPacketSize];

	/// <summary>
	/// Max number of active rays
	/// </summary>
	uint maxActive;

	/// <summary>
	/// Copy Ray
	/// </summary>
	/// <returns>New Copied Ray</returns>
	inline RayPacket Copy() const
	{
		RayPacket temp(maxActive);
		memcpy(temp.rays, rays, kPacketSize * sizeof(Ray));
		return temp;
	}

	inline RayPacket(uint maxRayActive)
	{
		assert(0 <= maxRayActive && maxRayActive <= kPacketSize);
		maxActive = maxRayActive;
	}

	inline void SetOrigin(const float4 &ori, int index)
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		rays[index].SetOrigin(ori);
	}
	inline void SetOrigin(const float3 & ori, int index)
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		rays[index].SetOrigin(ori);
	}

	inline void SetDirection(const float4 &dir, int index)
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		rays[index].SetDirection(dir);
	}
	inline void SetDirection(const float3 &dir, int index)
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		assert(almost_equal(1, length(dir))); // "Unormalized Vector"
		rays[index].SetDirection(dir);
	}

	inline void SetRay(const Ray &r, int index)
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		rays[index] = r;
	}

	inline void GetRay(Ray& r, int index) const
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		r = rays[index];
	}

	inline void GetRays(Ray rays[kPacketSize]) const {
		for (uint i = 0; i < maxActive; i++)
		{
			GetRay(rays[i], i);
		}
	}

	inline void GetSigns(uchar signs[kPacketSize]) const {
		for (uint i = 0; i < maxActive; i++)
		{
			const float4& dir = rays[i].direction;
			signs[i] = ((dir.x >= 0) << 0) | ((dir.y >= 0) << 1) | ((dir.z >= 0) << 2);
		}
	}

	/// <summary>
	/// Evaluate the ray for certain t
	/// </summary>
	/// <param name="t">Parameter</param>
	/// <param name="index">Ray Index</param>
	/// <returns>Position</returns>
	inline float4 Evaluate(const float t, int index) const
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		return rays[index].Evaluate(t);
	}

	inline float4 InverseDirection(int index) const
	{
		assert(0 <= (uint)index && (uint)index <= maxActive);
		return  rays[index].InverseDirection();
	}

	inline void InverseDirection(float4 invDir[kPacketSize]) const {
		for (uint i = 0; i < maxActive; i++)
		{
			invDir[i] = InverseDirection(i);
		}
	}

	inline void InverseDirection(float3 invDir[kPacketSize]) const {
		for (uint i = 0; i < maxActive; i++)
		{
			invDir[i] = make_float3(InverseDirection(i));
		}
	}

	inline Frustum CreateFrustum(uint corners[4]) {
		Frustum f;

		for (int i = 0; i < Frustum::kNumberOfPlanes; ++i)
		{
			assert(0 <= corners[i] && corners[i] <= maxActive);
			assert(0 <= corners[(i + 1) % 4] && corners[(i + 1) % 4] <= maxActive);
			const float4& di = rays[corners[i]].direction;
			const float4& di2 = rays[corners[(i + 1) % 4]].direction;

			f.normals[i] = cross(di, di2); // calculate n_i
			f.normals[i].w = -dot(f.normals[i], rays[corners[i]].origin); // Calculate b_i and save it at the end of the frustum normal
		}

		return f;
	}
};

inline bool TestAABBIntersection(const Ray &r, const aabb& box, const float3 invDir)
{
	// From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
#if 1
	__m128 ori = _mm_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0);
	__m128 dirInv = _mm_setr_ps(invDir.x, invDir.y, invDir.z, 0);

	__m128 t0 = _mm_mul_ps(_mm_sub_ps(box.bmin4, ori), dirInv);
	__m128 t1 = _mm_mul_ps(_mm_sub_ps(box.bmax4, ori), dirInv);

	__m128 tmin = _mm_min_ps(t0, t1);
	__m128 tmax = _mm_max_ps(t0, t1);

	return max_component3(tmin) <= min_component3(tmax);
#else
	float3 ori = make_float3(r.origin);

	float3 t0 = (box.bmin3 - ori) * invDir;
	float3 t1 = (box.bmax3 - ori) * invDir;

	float3 tmin = fminf(t0, t1);
	float3 tmax = fmaxf(t0, t1);

	return max(tmin.x, max(tmin.y, tmin.z)) <= min(tmax.x, min(tmax.y, tmax.z));
#endif
}

inline float TestAABBIntersectionDistance(const Ray& r, const aabb& box, const float3 invDir)
{
	// From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
#if 1
	__m128 ori = _mm_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0);
	__m128 dirInv = _mm_setr_ps(invDir.x, invDir.y, invDir.z, 0);

	__m128 t0 = _mm_mul_ps(_mm_sub_ps(box.bmin4, ori), dirInv);
	__m128 t1 = _mm_mul_ps(_mm_sub_ps(box.bmax4, ori), dirInv);

	__m128 tmin = _mm_min_ps(t0, t1);
	__m128 tmax = _mm_max_ps(t0, t1);

	float high = min_component3(tmax);
	float low = max_component3(tmin);

	if (low <= high)
	{
		return low;
	}

	return -1.0;
#else
	float3 ori = make_float3(r.origin);

	float3 t0 = (box.bmin3 - ori) * invDir;
	float3 t1 = (box.bmax3 - ori) * invDir;

	float3 tmin = fminf(t0, t1);
	float3 tmax = fmaxf(t0, t1);

	float high = min(tmax.x, min(tmax.y, tmax.z));
	float low = max(tmin.x, max(tmin.y, tmin.z));

	if (low <= high)
	{
		return low;
	}

	return -1.0;
#endif
}

inline bool TestAABBIntersectionBounds(const Ray& r, const aabb& box, const float3 invDir, const float minT, const float maxT)
{
	// From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525
	// With Modifications from https://psgraphics.blogspot.com/2016/02/new-simple-ray-box-test-from-andrew.html
	__m128 ori = _mm_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0);
	__m128 dirInv = _mm_setr_ps(invDir.x, invDir.y, invDir.z, 0);

	__m128 t0 = _mm_mul_ps(_mm_sub_ps(box.bmin4, ori), dirInv);
	__m128 t1 = _mm_mul_ps(_mm_sub_ps(box.bmax4, ori), dirInv);

	__m128 tmin = _mm_max_ps(_mm_min_ps(t0, t1), _mm_set_ps1(minT));
	__m128 tmax = _mm_min_ps(_mm_max_ps(t0, t1), _mm_set_ps1(maxT));

	__m128 comp = _mm_cmple_ps(tmin, tmax);

	bool result = (_mm_movemask_ps(comp) & 0x7) == 0x7;
	return result;
}

inline uchar TestAABB4Intersection(const Ray& r, const aabb boxes[4], const float3 invDir)
{
	// Modified From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525

	uchar res = 0;

	const __m256  ori = _mm256_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0, r.origin.x, r.origin.y, r.origin.z, 0);
	const __m256  dirInv = _mm256_setr_ps(invDir.x, invDir.y, invDir.z, 0, invDir.x, invDir.y, invDir.z, 0);

	for (int i = 0; i < 2; i++)
	{
		const int baseIndex0 = 2 * i + 0;
		const int baseIndex1 = baseIndex0 + 1;

		const __m256 t0 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmin4, boxes[baseIndex1].bmin4), ori), dirInv);
		const __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmax4, boxes[baseIndex1].bmax4), ori), dirInv);

		__m256 tmin = _mm256_min_ps(t0, t1);
		__m256 tmax = _mm256_max_ps(t0, t1);

		// Modified from https://stackoverflow.com/a/17639457
		// Horizontal max. Note that the last component is trash
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(2, 1, 0, 2)));
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(1, 0, 2, 2)));

		// Horizontal min. Note that the last component is trash
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(2, 1, 0, 2)));
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(1, 0, 2, 2)));

		const __m256 comp = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
		const int mask = _mm256_movemask_ps(comp);

		const bool r0 = (mask & 0x7) == 0x7;
		const bool r1 = (mask & 0x70) == 0x70;

		// assert(r0 == TestAABBIntersection(r, boxes[2 * i + 0], invDir));
		// assert(r1 == TestAABBIntersection(r, boxes[2 * i + 1], invDir));

		res |= (r0 << (2 * i + 0)) | (r1 << (2 * i + 1));
	}
	
	return res;
}

inline uchar TestAABB4IntersectionDistance(const Ray& r, const aabb boxes[4], const float3 invDir, float distances[4])
{
	// Modified From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525

	uchar res = 0;

	const __m256  ori = _mm256_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0, r.origin.x, r.origin.y, r.origin.z, 0);
	const __m256  dirInv = _mm256_setr_ps(invDir.x, invDir.y, invDir.z, 0, invDir.x, invDir.y, invDir.z, 0);

	for (int i = 0; i < 2; i++)
	{
		const int baseIndex0 = 2 * i + 0;
		const int baseIndex1 = baseIndex0 + 1;
		
		const __m256 t0 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmin4, boxes[baseIndex1].bmin4), ori), dirInv);
		const __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmax4, boxes[baseIndex1].bmax4), ori), dirInv);

		__m256 tmin = _mm256_min_ps(t0, t1);
		__m256 tmax = _mm256_max_ps(t0, t1);

		// Modified from https://stackoverflow.com/a/17639457
		// Horizontal max. Note that the last component is trash
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(2, 1, 0, 2)));
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(1, 0, 2, 2)));

		// Horizontal min. Note that the last component is trash
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(2, 1, 0, 2)));
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(1, 0, 2, 2)));

		const __m256 comp = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
		const int mask = _mm256_movemask_ps(comp);

		const bool r0 = (mask & 0x7) == 0x7;
		const bool r1 = (mask & 0x70) == 0x70;

		// assert(r0 == TestAABBIntersection(r, boxes[2 * i + 0], invDir));
		// assert(r1 == TestAABBIntersection(r, boxes[2 * i + 1], invDir));

		res |= (r0 << (2 * i + 0)) | (r1 << (2 * i + 1));

		// Note that they might contain trash, check mask
		distances[baseIndex0] = max(_mm_cvtss_f32(_mm256_extractf128_ps(tmin, 0)), 0.0f); // Max, if negative you are inside
		distances[baseIndex1] = max(_mm_cvtss_f32(_mm256_extractf128_ps(tmin, 1)), 0.0f);
		assert((r0 && distances[baseIndex0] >= 0) || !r0);
		assert((r1 && distances[baseIndex1] >= 0) || !r1);
	}

	return res;
}

inline uchar TestAABB4IntersectionBounds(const Ray& r, const aabb boxes[4], const float3 invDir, const float minT, const float maxT)
{
	// Modified From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525

	uchar res = 0;

	const __m256 ori = _mm256_setr_ps(r.origin.x, r.origin.y, r.origin.z, 0, r.origin.x, r.origin.y, r.origin.z, 0);
	const __m256  dirInv = _mm256_setr_ps(invDir.x, invDir.y, invDir.z, 0, invDir.x, invDir.y, invDir.z, 0);

	const __m256  maxT_ = _mm256_set1_ps(maxT);
	const __m256  minT_ = _mm256_set1_ps(minT);

	for (int i = 0; i < 2; i++)
	{
		const int baseIndex0 = 2 * i + 0;
		const int baseIndex1 = baseIndex0 + 1;

		const __m256 t0 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmin4, boxes[baseIndex1].bmin4), ori), dirInv);
		const __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(_mm256_setr_m128(boxes[baseIndex0].bmax4, boxes[baseIndex1].bmax4), ori), dirInv);

		__m256 tmin = _mm256_max_ps(_mm256_min_ps(t0, t1), minT_);
		__m256 tmax = _mm256_min_ps(_mm256_max_ps(t0, t1), maxT_);

		// Compare
		__m256 comp = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);

		const int mask = _mm256_movemask_ps(comp);

		const bool r0 = (mask & 0x7) == 0x7;
		const bool r1 = (mask & 0x70) == 0x70;

		// assert(r0 == TestAABBIntersectionBounds(r, boxes[2 * i + 0], invDir, minT, maxT));
		// assert(r1 == TestAABBIntersectionBounds(r, boxes[2 * i + 1], invDir, minT, maxT));

		res |= (r0 << (2 * i + 0)) | (r1 << (2 * i + 1));
	}

	return res;
}

inline uchar Test4AABBIntersection(const aabb& box, const Ray rays[4], const float3 invDir[4])
{
	// Modified From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525

	uchar res = 0;

	const __m256 boxMin = _mm256_setr_m128(box.bmin4, box.bmin4);
	const __m256 boxMax = _mm256_setr_m128(box.bmax4, box.bmax4);

	for (int i = 0; i < 2; i++)
	{
		const int baseIndex0 = 2 * i + 0;
		const int baseIndex1 = baseIndex0 + 1;

		const Ray& ra0 = rays[baseIndex0];
		const Ray& ra1 = rays[baseIndex1];

		const float3& inv0 = invDir[baseIndex0];
		const float3& inv1 = invDir[baseIndex1];

		const __m256 ori = _mm256_setr_ps(ra0.origin.x, ra0.origin.y, ra0.origin.z, 0, ra1.origin.x, ra1.origin.y, ra1.origin.z, 0);
		const __m256 dirInv = _mm256_setr_ps(inv0.x, inv0.y, inv0.z, 0, inv1.x, inv1.y, inv1.z, 0);

		const __m256 t0 = _mm256_mul_ps(_mm256_sub_ps(boxMin, ori), dirInv);
		const __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(boxMax, ori), dirInv);

		__m256 tmin = _mm256_min_ps(t0, t1);
		__m256 tmax = _mm256_max_ps(t0, t1);

		// Modified from https://stackoverflow.com/a/17639457
		// Horizontal max. Note that the last component is trash
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(2, 1, 0, 2)));
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(1, 0, 2, 2)));

		// Horizontal min. Note that the last component is trash
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(2, 1, 0, 2)));
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(1, 0, 2, 2)));

		const __m256 comp = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
		const int mask = _mm256_movemask_ps(comp);

		const bool r0 = (mask & 0x7) == 0x7;
		const bool r1 = (mask & 0x70) == 0x70;

		assert(r0 == TestAABBIntersection(ra0, box, inv0));
		assert(r1 == TestAABBIntersection(ra1, box, inv1));

		res |= (r0 << (2 * i + 0)) | (r1 << (2 * i + 1));
	}

	return res;
}

inline uchar Test4AABBIntersectionDistance(const aabb& box, const Ray rays[4], const float3 invDir[4], float distances[4])
{
	// Modified From https://medium.com/@bromanz/another-view-on-the-classic-ray-aabb-intersection-algorithm-for-bvh-traversal-41125138b525

	uchar res = 0;

	const __m256 boxMin = _mm256_setr_m128(box.bmin4, box.bmin4);
	const __m256 boxMax = _mm256_setr_m128(box.bmax4, box.bmax4);

	for (int i = 0; i < 2; i++)
	{
		const int baseIndex0 = 2 * i + 0;
		const int baseIndex1 = baseIndex0 + 1;

		const Ray& ra0 = rays[baseIndex0];
		const Ray& ra1 = rays[baseIndex1];

		const float3& inv0 = invDir[baseIndex0];
		const float3& inv1 = invDir[baseIndex1];

		const __m256 ori = _mm256_setr_ps(ra0.origin.x, ra0.origin.y, ra0.origin.z, 0, ra1.origin.x, ra1.origin.y, ra1.origin.z, 0);
		const __m256 dirInv = _mm256_setr_ps(inv0.x, inv0.y, inv0.z, 0, inv1.x, inv1.y, inv1.z, 0);

		const __m256 t0 = _mm256_mul_ps(_mm256_sub_ps(boxMin, ori), dirInv);
		const __m256 t1 = _mm256_mul_ps(_mm256_sub_ps(boxMax, ori), dirInv);

		__m256 tmin = _mm256_min_ps(t0, t1);
		__m256 tmax = _mm256_max_ps(t0, t1);

		// Modified from https://stackoverflow.com/a/17639457
		// Horizontal max. Note that the last component is trash
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(2, 1, 0, 2)));
		tmin = _mm256_max_ps(tmin, _mm256_shuffle_ps(tmin, tmin, _MM_SHUFFLE(1, 0, 2, 2)));

		// Horizontal min. Note that the last component is trash
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(2, 1, 0, 2)));
		tmax = _mm256_min_ps(tmax, _mm256_shuffle_ps(tmax, tmax, _MM_SHUFFLE(1, 0, 2, 2)));

		const __m256 comp = _mm256_cmp_ps(tmin, tmax, _CMP_LE_OQ);
		const int mask = _mm256_movemask_ps(comp);

		const bool r0 = (mask & 0x7) == 0x7;
		const bool r1 = (mask & 0x70) == 0x70;

		assert(r0 == TestAABBIntersection(ra0, box, inv0));
		assert(r1 == TestAABBIntersection(ra1, box, inv1));

		res |= (r0 << (2 * i + 0)) | (r1 << (2 * i + 1));

		// Note that they might contain trash, check mask
		distances[baseIndex0] = max(_mm_cvtss_f32(_mm256_extractf128_ps(tmin, 0)), 0.0f); // Max, if negative you are inside
		distances[baseIndex1] = max(_mm_cvtss_f32(_mm256_extractf128_ps(tmin, 1)), 0.0f);
		assert((r0 && distances[baseIndex0] >= 0) || !r0);
		assert((r1 && distances[baseIndex1] >= 0) || !r1);
	}

	return res;
}


/// <summary>
/// Ray Triangle Interception Information
/// </summary>
struct RayTriangleInterceptInfo
{
public:
	/// <summary>
	/// Ray t Value
	/// </summary>
	float t;

	/// <summary>
	/// Barycentric u coordinate
	/// </summary>
	float u;

	/// <summary>
	/// Barycentric v coordinate
	/// </summary>
	float v;

	/// <summary>
	/// If the interception was back facing
	/// </summary>
	bool backFacing;

	inline RayTriangleInterceptInfo()
	{
		Reset();
	}

	inline void Reset()
	{
		t = std::numeric_limits<float>::infinity();
		u = 0;
		v = 0;
		backFacing = false;
	}

	inline float GetWCoord() const
	{
		assert(!isinf(t)); // Bad Interception
		return 1 - u - v;
	}

	// No copy by accident
	RayTriangleInterceptInfo(const RayTriangleInterceptInfo&) = delete;
	RayTriangleInterceptInfo& operator=(const RayTriangleInterceptInfo&) = delete;

	inline void CopyTo(RayTriangleInterceptInfo& info) const
	{
		info.t = t;
		info.u = u;
		info.v = v;
		info.backFacing = backFacing;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is closer than other to ray origin</returns>
	inline bool operator<(const RayTriangleInterceptInfo& other) const
	{
		return t < other.t;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is farther than other to ray origin</returns>
	inline bool operator>(const RayTriangleInterceptInfo& other) const
	{
		return !(*this < other);
	}

};

/// <summary>
/// Ray Mesh Interception Info
/// </summary>
struct RayMeshInterceptInfo
{
public:
	/// <summary>
	/// Triangle Interception Info
	/// </summary>
	RayTriangleInterceptInfo triIntercept;

	/// <summary>
	/// Mesh Id
	/// </summary>
	int meshId; 
	
	/// <summary>
	/// Triangle Id
	/// </summary>
	int triId;


	inline RayMeshInterceptInfo()
	{
		Reset();
	}

	inline void Reset()
	{
		triIntercept.Reset();
		meshId = -1;
		triId = -1;
	}

	operator bool() const
	{
		return meshId != -1;
	}

	// No copy by accident
	RayMeshInterceptInfo(const RayMeshInterceptInfo&) = delete;
	RayMeshInterceptInfo& operator=(const RayMeshInterceptInfo&) = delete;

	inline void CopyTo(RayMeshInterceptInfo& info) const
	{
		triIntercept.CopyTo(info.triIntercept);
		info.meshId = meshId;
		info.triId = triId;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is closer than other to ray origin</returns>
	inline bool operator<(const RayMeshInterceptInfo& other) const
	{
		return triIntercept < other.triIntercept;
	}

	/// <summary>
	/// Comparison of t Value
	/// </summary>
	/// <param name="other">Other to Compare</param>
	/// <returns>If this is farther than other to ray origin</returns>
	inline bool operator>(const RayMeshInterceptInfo& other) const
	{
		return !(*this < other);
	}
};


/// <summary>
/// Test for Ray Triangle Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="v0">Vertex 0</param>
/// <param name="v1">Vertex 1</param>
/// <param name="v2">Vertex 2</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayTriangle(const Ray& r, const float4& v0, const float4& v1, const float4& v2, RayTriangleInterceptInfo& hitInfo);

/// <summary>
/// Test for Ray Mesh Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="m">Mesh</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayMesh(const Ray& r, const Mesh& m, RayMeshInterceptInfo& hitInfo);


/// <summary>
/// Test for Ray and a Scene (list of meshes) Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="meshes">Meshes</param>
/// <param name="hitInfo">Output info</param>
/// <returns>If there was intersection</returns>
template <bool backCulling>
[[nodiscard]]
bool interceptRayScene(const Ray& r, const vector<Mesh>& meshes, RayMeshInterceptInfo& hitInfo);

/// <summary>
/// Test depth for Ray Triangle Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="v0">Vertex 0</param>
/// <param name="v1">Vertex 1</param>
/// <param name="v2">Vertex 2</param>
/// <param name="tD">Depth Squared to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayTriangle(const Ray& r, const float4& v0, const float4& v1, const float4& v2, const float tD);

/// <summary>
/// Test depth for Ray Mesh Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="m">Mesh</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth Squared to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayMesh(const Ray& r, const Mesh& m, const int meshId, const int triId, const float tD);


/// <summary>
/// Test for Ray and a Sccene (list of meshes) Interception
/// </summary>
/// <param name="backCulling">If back faced triangles</param>
/// <param name="r">Ray</param>
/// <param name="meshes">Meshes</param>
/// /// <param name="meshId">Instance Id (TODO)</param>
/// <param name="meshId">Mesh Id</param>
/// <param name="triId">Triangle Id</param>
/// <param name="tD">Depth to beat</param>
/// <returns>If there was intersection and it is closer than the depth</returns>
template <bool backCulling>
[[nodiscard]]
bool depthRayScene(const Ray& r, const vector<Mesh>& meshes, const int instId, const int meshId, const int triId, const float tD);

constexpr float kFrustumCullingTestEps = 0.1f;

inline bool TestFrustumAABBIntersection(const Frustum& f, const aabb& box) {
#define USE_FRUSTUM_SSE
#ifndef USE_FRUSTUM_SSE // Normal Code

	for (size_t i = 0; i < Frustum::kNumberOfPlanes; i++)
	{
		float4 nearestCorner;
		nearestCorner.w = 1;

		if (f.normals[i].x > 0)
		{
			nearestCorner.x = box.bmin3.x;
		}
		else
		{
			nearestCorner.x = box.bmax3.x;
		}
		if (f.normals[i].y > 0)
		{
			nearestCorner.y = box.bmin3.y;
		}
		else
		{
			nearestCorner.y = box.bmax3.y;
		}

		if (f.normals[i].z > 0)
		{
			nearestCorner.z = box.bmin3.z;
		}
		else
		{
			nearestCorner.z = box.bmax3.z;
		}

		const bool test = dot(f.normals[i], nearestCorner) > kFrustumCullingTestEps;
		if (test) return false;
	}
	return true;
	
#else
#ifndef NDEBUG
	bool properResult = true;
	{
		const float4 c000 = make_float4(box.bmin3.x, box.bmin3.y, box.bmin3.z, 1);
		const float4 c001 = make_float4(box.bmin3.x, box.bmin3.y, box.bmax3.z, 1);
		const float4 c010 = make_float4(box.bmin3.x, box.bmax3.y, box.bmin3.z, 1);
		const float4 c011 = make_float4(box.bmin3.x, box.bmax3.y, box.bmax3.z, 1);
		const float4 c100 = make_float4(box.bmax3.x, box.bmin3.y, box.bmin3.z, 1);
		const float4 c101 = make_float4(box.bmax3.x, box.bmin3.y, box.bmax3.z, 1);
		const float4 c110 = make_float4(box.bmax3.x, box.bmax3.y, box.bmin3.z, 1);
		const float4 c111 = make_float4(box.bmax3.x, box.bmax3.y, box.bmax3.z, 1);

		for (size_t i = 0; i < Frustum::kNumberOfPlanes; i++)
		{
			const bool test0 = dot(f.normals[i], c000) > kFrustumCullingTestEps;
			const bool test1 = dot(f.normals[i], c001) > kFrustumCullingTestEps;
			const bool test2 = dot(f.normals[i], c010) > kFrustumCullingTestEps;
			const bool test3 = dot(f.normals[i], c011) > kFrustumCullingTestEps;
			const bool test4 = dot(f.normals[i], c100) > kFrustumCullingTestEps;
			const bool test5 = dot(f.normals[i], c101) > kFrustumCullingTestEps;
			const bool test6 = dot(f.normals[i], c110) > kFrustumCullingTestEps;
			const bool test7 = dot(f.normals[i], c111) > kFrustumCullingTestEps;

			if (test0 && test1 && test2 && test3 && test4 && test5 && test6 && test7)
			{
				properResult = false;
				break;
			}
		}
	}
#endif

	const __m128  frustumCullingTestEps = _mm_set_ps1(kFrustumCullingTestEps);
	const __m128  zero = _mm_set_ps1(0);

	__m128 res;
	__m128 selector;
	__m128 corner;

	// Dot Product registers
	__m128 mulRes, shufReg, sumsReg;

	const __m128 mini = _mm_setr_ps(box.bmin3.x, box.bmin3.y, box.bmin3.z, 1);
	const __m128 maxi = _mm_setr_ps(box.bmax3.x, box.bmax3.y, box.bmax3.z, 1);

	for (size_t i = 0; i < Frustum::kNumberOfPlanes; i++)
	{
		const __m128 normal = _mm_setr_ps(f.normals[i].x, f.normals[i].y, f.normals[i].z, f.normals[i].w);

		selector = _mm_cmpgt_ps(normal, zero);
		corner = _mm_blendv_ps(maxi, mini, selector);

		{
			// Dot Product
			// From https://stackoverflow.com/a/42924346
			mulRes = _mm_mul_ps(corner, normal);

			// Calculates the sum of SSE Register - https://stackoverflow.com/a/35270026/195787
			shufReg = _mm_movehdup_ps(mulRes);        // Broadcast elements 3,1 to 2,0
			sumsReg = _mm_add_ps(mulRes, shufReg);
			shufReg = _mm_movehl_ps(shufReg, sumsReg); // High Half -> Low Half
			sumsReg = _mm_add_ss(sumsReg, shufReg); // Result in the lower part of the SSE Register
		}
		res = _mm_cmpgt_ps(sumsReg, frustumCullingTestEps);

		if ((_mm_movemask_ps(res) & 1) != 0)
		{
			assert(!properResult);
			return false;
		}
	}
	assert(properResult);
	return true;
#endif
	
}

inline bool TestFrustumTriangle(const Frustum& f, const float4& v0, const float4& v1, const float4& v2) {
//#define FrustumTriangleSSI
#ifndef FrustumTriangleSSI

	float4 v02, v12, v22;
	v02 = v0;
	v12 = v1;
	v22 = v2;
	v02.w = 1;
	v12.w = 1;
	v22.w = 1;

	for (size_t i = 0; i < Frustum::kNumberOfPlanes; i++)
	{
		
		const bool test1 = dot(f.normals[i], v02) > kFrustumCullingTestEps;
		const bool test2 = dot(f.normals[i], v12) > kFrustumCullingTestEps;
		const bool test3 = dot(f.normals[i], v22) > kFrustumCullingTestEps;

		if (test1 && test2 && test3)
		{
			return false;
		}
	}
	return true;

#else

	const __m128  frustumCullingTestEps = _mm_set_ps1(kFrustumCullingTestEps);

	const __m128 v00 = _mm_setr_ps(v0.x, v0.y, v0.z, 1);
	const __m128 v01 = _mm_setr_ps(v1.x, v1.y, v1.z, 1);
	const __m128 v02 = _mm_setr_ps(v2.x, v2.y, v2.z, 1);

	// Dot Product registers
	__m128 mulRes, shufReg, sumsReg;

	__m128 res;

	for (size_t i = 0; i < Frustum::kNumberOfPlanes; i++)
	{
		const __m128 normal = _mm_setr_ps(f.normals[i].x, f.normals[i].y, f.normals[i].z, f.normals[i].w);

		{
			// Dot Product
			// From https://stackoverflow.com/a/42924346
			mulRes = _mm_mul_ps(v00, normal);

			// Calculates the sum of SSE Register - https://stackoverflow.com/a/35270026/195787
			shufReg = _mm_movehdup_ps(mulRes);        // Broadcast elements 3,1 to 2,0
			sumsReg = _mm_add_ps(mulRes, shufReg);
			shufReg = _mm_movehl_ps(shufReg, sumsReg); // High Half -> Low Half
			sumsReg = _mm_add_ss(sumsReg, shufReg); // Result in the lower part of the SSE Register
		}
		__m128 res = _mm_cmpgt_ps(sumsReg, frustumCullingTestEps);

		{
			// Dot Product
			// From https://stackoverflow.com/a/42924346
			mulRes = _mm_mul_ps(v01, normal);

			// Calculates the sum of SSE Register - https://stackoverflow.com/a/35270026/195787
			shufReg = _mm_movehdup_ps(mulRes);        // Broadcast elements 3,1 to 2,0
			sumsReg = _mm_add_ps(mulRes, shufReg);
			shufReg = _mm_movehl_ps(shufReg, sumsReg); // High Half -> Low Half
			sumsReg = _mm_add_ss(sumsReg, shufReg); // Result in the lower part of the SSE Register
		}
		res = _mm_and_ps(res, _mm_cmpgt_ps(sumsReg, frustumCullingTestEps));

		{
			// Dot Product
			// From https://stackoverflow.com/a/42924346
			mulRes = _mm_mul_ps(v02, normal);

			// Calculates the sum of SSE Register - https://stackoverflow.com/a/35270026/195787
			shufReg = _mm_movehdup_ps(mulRes);        // Broadcast elements 3,1 to 2,0
			sumsReg = _mm_add_ps(mulRes, shufReg);
			shufReg = _mm_movehl_ps(shufReg, sumsReg); // High Half -> Low Half
			sumsReg = _mm_add_ss(sumsReg, shufReg); // Result in the lower part of the SSE Register
		}
		res = _mm_and_ps(res, _mm_cmpgt_ps(sumsReg, frustumCullingTestEps));

		if ((_mm_movemask_ps(res) & 1) != 0)
		{
			return false;
		}
	}
	return true;

#endif
}

}

#include "intersection_utils.impl.h"