namespace lh2core
{

template <bool backCulling>
[[nodiscard]]
bool interceptRayTriangle(
	const Ray& r, 
	const float4& v0, const float4& v1, const float4& v2, 
	RayTriangleInterceptInfo& hitInfo)
{
	/// Modified from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
#define USE_TRIANGLE_INTERCEPT_SSE
#ifndef USE_TRIANGLE_INTERCEPT_SSE
	float4 v0v1 = v1 - v0;
	float4 v0v2 = v2 - v0;
	float4 pvec = cross(r.direction, v0v2);
	float det = dot(v0v1, pvec);

	// Cool C++17 trick
	if constexpr (backCulling)
	{
		if (det < kEps) // Back culling triangles
		{
			// No intersection
			return false;
		}
	}
	else
	{
		if (fabs(det) < kEps) // Only cull parallel rays (No backculling)
		{
			// No intersection
			return false;
		}
	}

	
	hitInfo.backFacing = det < 0;

	float invDet = 1 / det;

	float4 tvec = r.origin - v0;
	hitInfo.u = dot(tvec, pvec) * invDet;
	if (hitInfo.u < 0 || hitInfo.u > 1)
	{
		return false;
	}

	float4 qvec = cross(tvec, v0v1);
	hitInfo.v = dot(r.direction, qvec) * invDet;
	if (hitInfo.v < 0 || hitInfo.u + hitInfo.v > 1) 
	{
		return false;
	}

	hitInfo.t = dot(v0v2, qvec) * invDet;

	if (hitInfo.t < kEps)
	{
		return false;
	}

	return true;
#else
	const __m128 v00 = _mm_setr_ps(v0.x, v0.y, v0.z, 1);
	const __m128 v10 = _mm_setr_ps(v1.x, v1.y, v1.z, 1);
	const __m128 v20 = _mm_setr_ps(v2.x, v2.y, v2.z, 1);

	const __m128 dir = _mm_setr_ps(r.direction.x, r.direction.y, r.direction.z, 1);
	const __m128 ori = _mm_setr_ps(r.origin.x, r.origin.y, r.origin.z, 1);

	const __m128 v0v1 = _mm_sub_ps(v10, v00);
	const __m128 v0v2 = _mm_sub_ps(v20, v00);
	const __m128 pvec = cross_3shuffles(dir, v0v2);
	const float det = _mm_cvtss_f32(_mm_dp_ps(v0v1, pvec, 0xFF));

	if constexpr (backCulling)
	{
		if (det < kEps) // Back culling triangles
		{
			// No intersection
			return false;
		}
	}
	else
	{
		if (fabs(det) < kEps) // Only cull parallel rays (No backculling)
		{
			// No intersection
			return false;
		}
	}


	hitInfo.backFacing = det < 0;

	const float invDet = 1 / det;

	const __m128 tvec = _mm_sub_ps(ori, v00);
	hitInfo.u = _mm_cvtss_f32(_mm_dp_ps(tvec, pvec, 0xFF)) * invDet;
	if (hitInfo.u < 0 || hitInfo.u > 1)
	{
		return false;
	}

	const __m128 qvec = cross_3shuffles(tvec, v0v1);
	hitInfo.v = _mm_cvtss_f32(_mm_dp_ps(dir, qvec, 0xFF)) * invDet;
	if (hitInfo.v < 0 || hitInfo.u + hitInfo.v > 1)
	{
		return false;
	}

	hitInfo.t = _mm_cvtss_f32(_mm_dp_ps(v0v2, qvec, 0xFF)) * invDet;

	if (hitInfo.t < kEps)
	{
		return false;
	}

	return true;
#endif
}

template <bool backCulling>
[[nodiscard]]
bool interceptRayMesh(const Ray& r, const Mesh& m, RayMeshInterceptInfo& hitInfo)
{
	assert(m.vcount % 3 == 0); // No weird meshes
	RayTriangleInterceptInfo tempHitInfo;
	bool hit = false;
	int triCount = m.vcount / 3;
	int vPos;

	for (int i = 0; i < triCount; i++)
	{
		vPos = i * 3;
		if (interceptRayTriangle<backCulling>(r, m.vertices[vPos + 0], m.vertices[vPos + 1], m.vertices[vPos + 2], tempHitInfo))
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


	hitInfo.meshId = m.meshID;
	return true;
}

template <bool backCulling>
[[nodiscard]]
bool interceptRayScene(const Ray& r, const vector<Mesh>& meshes, RayMeshInterceptInfo& hitInfo)
{
	RayMeshInterceptInfo tempInfo;
	bool hit = false;
	hitInfo.Reset();
	for (size_t i = 0; i < meshes.size(); i++)
	{
		const Mesh& m = meshes[i];
		if (interceptRayMesh<backCulling>(r, m, tempInfo))
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

/// Depth testing below

template <bool backCulling>
[[nodiscard]]
bool depthRayTriangle(
	const Ray& r, 
	const float4& v0, const float4& v1, const float4& v2, 
	const float tD)
{
	/// Modified from https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle/moller-trumbore-ray-triangle-intersection
#define USE_TRIANGLE_DEPTH_SSE
#ifndef USE_TRIANGLE_DEPTH_SSE
	float4 v0v1 = v1 - v0;
	float4 v0v2 = v2 - v0;
	float4 pvec = cross(r.direction, v0v2);
	float det = dot(v0v1, pvec);

	// Cool C++17 trick
	if constexpr (backCulling)
	{
		if (det < kEps) // Back culling triangles
		{
			// No intersection
			return false;
		}
	}
	else
	{
		if (fabs(det) < kEps) // Only cull parallel rays (No backculling)
		{
			// No intersection
			return false;
		}
	}

	float invDet = 1 / det;

	float4 tvec = r.origin - v0;
	float u = dot(tvec, pvec) * invDet;
	if (u < 0 || u > 1)
	{
		return false;
	}

	float4 qvec = cross(tvec, v0v1);
	float v = dot(r.direction, qvec) * invDet;
	if (v < 0 || u + v > 1)
	{
		return false;
	}

	float t = dot(v0v2, qvec) * invDet;

	if (t < kEps)
	{
		return false;
	}

	// New triangle closer
	return t - tD < kEps;
#else
	const __m128 v00 = _mm_setr_ps(v0.x, v0.y, v0.z, 1);
	const __m128 v10 = _mm_setr_ps(v1.x, v1.y, v1.z, 1);
	const __m128 v20 = _mm_setr_ps(v2.x, v2.y, v2.z, 1);

	const __m128 dir = _mm_setr_ps(r.direction.x, r.direction.y, r.direction.z, 1);
	const __m128 ori = _mm_setr_ps(r.origin.x, r.origin.y, r.origin.z, 1);

	const __m128 v0v1 = _mm_sub_ps(v10, v00);
	const __m128 v0v2 = _mm_sub_ps(v20, v00);
	const __m128 pvec = cross_3shuffles(dir, v0v2);
	const float det = _mm_cvtss_f32(_mm_dp_ps(v0v1, pvec, 0xFF));

	if constexpr (backCulling)
	{
		if (det < kEps) // Back culling triangles
		{
			// No intersection
			return false;
		}
	}
	else
	{
		if (fabs(det) < kEps) // Only cull parallel rays (No backculling)
		{
			// No intersection
			return false;
		}
	}

	const float invDet = 1 / det;

	const __m128 tvec = _mm_sub_ps(ori, v00);
	const float u = _mm_cvtss_f32(_mm_dp_ps(tvec, pvec, 0xFF)) * invDet;
	if (u < 0 || u > 1)
	{
		return false;
	}

	const __m128 qvec = cross_3shuffles(tvec, v0v1);
	const float v = _mm_cvtss_f32(_mm_dp_ps(dir, qvec, 0xFF)) * invDet;
	if (v < 0 || u + v > 1)
	{
		return false;
	}

	const float t = _mm_cvtss_f32(_mm_dp_ps(v0v2, qvec, 0xFF)) * invDet;

	if (t < kEps)
	{
		return false;
	}

	// New triangle closer
	return t - tD < kEps;
#endif
}

template <bool backCulling>
[[nodiscard]]
bool depthRayMesh(const Ray& r, const Mesh& m, const int meshId, const int triId, const float tD)
{
	assert(m.vcount % 3 == 0); // No weird meshes
	const bool sameMesh = m.meshID == meshId;
	int triCount = m.vcount / 3;
	int vPos;

	for (int i = 0; i < triCount; i++)
	{
		vPos = i * 3;
		if (depthRayTriangle<backCulling>(r, m.vertices[vPos + 0], m.vertices[vPos + 1], m.vertices[vPos + 2], tD))
		{
			if (!sameMesh || (sameMesh && i != triId))
			{
				return true;
			}
		}
	}

	return false;
}

template <bool backCulling>
[[nodiscard]]
bool depthRayScene(const Ray& r, const vector<Mesh>& meshes, const int instId, const int meshId, const int triId, const float tD)
{
	for (size_t i = 0; i < meshes.size(); i++)
	{
		const Mesh& m = meshes[i];
		if (depthRayMesh<backCulling>(r, m, meshId, triId, tD))
		{
			return true;
		}
	}

	return false;
}

}