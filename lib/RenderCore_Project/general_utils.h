#pragma once

namespace lh2core
{

	constexpr float kSqrt2 = 1.41421356237f;

	/// <summary>
	/// Approximately the same
	/// </summary>
	/// <param name="eps">Epsilon</param>
	/// <param name="target">Target Val</param>
	/// <param name="value">Val</param>
	/// <returns>If they are almost the same</returns>
	inline bool approximately(float eps, float target, float value)
	{
		return fabs(target - value) < eps;
	}

	/// <summary>
	/// General epsilon to use
	/// </summary>
	constexpr float kEps = 0.00001;

#define almost_equal(target, value) approximately(kEps, (target), (value))

	/// <summary>
	/// 3D Cross product using float 4
	/// </summary>
	/// <param name="a"></param>
	/// <param name="b"></param>
	/// <returns></returns>
	inline float4 cross(float4 a, float4 b) { return make_float4(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x, 0); }

	inline float4 uchar4ToFloat4(const uchar4 v)
	{
		constexpr float invVal = 1.0f / 255.0f;
		return make_float4((float)v.x * invVal, (float)v.y * invVal, (float)v.z * invVal, (float)v.w * invVal);
	}

	inline uint float4ToUint(const float4& color)
	{
		float4 tempColor = color;
		tempColor *= 255.0f;
		tempColor = clamp(tempColor, 0, 255);
		// AABBGGRR
		return
			((((uint)tempColor.w) << 24) & 0xFF000000)
			| ((((uint)tempColor.z) << 16) & 0xFF0000)
			| ((((uint)tempColor.y) << 8) & 0xFF00)
			| (((uint)tempColor.x) & 0xFF);
	}

	inline float4 bilinearInterpolation(
		const float tx,
		const float ty,
		const float4& c00,
		const float4& c10,
		const float4& c01,
		const float4& c11)
	{
		// From https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/interpolation/bilinear-filtering
		return (1.0f - tx) * (1.0f - ty) * c00 +
			tx * (1.0f - ty) * c10 +
			(1.0f - tx) * ty * c01 +
			tx * ty * c11;
	}

	template<bool useClamp>
	inline float4 textureFetch(const uchar4* texels, const int width, const int height, float u, float v)
	{
		if constexpr (useClamp)
		{
			u = clamp(u, 0.0f, 1.0f);
			v = clamp(v, 0.0f, 1.0f);
		}
		else
		{
			u = fmodf(u, 1.0f);
			if (u < 0.0f)
			{
				u += 1.0f;
			}

			v = fmodf(v, 1.0f);
			if (v < 0.0f)
			{
				v += 1.0f;
			}
		}

		float su = u * (float)(width - 1);
		float sv = v * (float)(height - 1);

		float tu, tv;

		float bu, bv;

		bu = (int)su;
		bv = (int)sv;


		tu = su - bu;
		tv = sv - bv;

		int uLo, uHi, vLo, vHi;

		if constexpr (useClamp)
		{
			uLo = clamp((int)bu, 0, width - 1);
			uHi = clamp(uLo + 1, 0, width - 1);

			vLo = clamp((int)bv, 0, height - 1);
			vHi = clamp(vLo + 1, 0, height - 1);
		}
		else
		{
			uLo = (int)fmodf(bu, (float)(width - 1));
			uHi = (uLo + 1) % width;

			vLo = (int)fmodf(bv, (float)(height - 1));
			vHi = (vLo + 1) % height;
		}

		return bilinearInterpolation(tu, tv, uchar4ToFloat4(texels[uLo + vLo * width]), uchar4ToFloat4(texels[uLo + vHi * width]), uchar4ToFloat4(texels[uHi + vLo * width]), uchar4ToFloat4(texels[uHi + vHi * width]));
	}

	template<bool useClamp>
	inline float4 textureFetch(const float4* texels, const int width, const int height, float u, float v)
	{
		if constexpr (useClamp)
		{
			u = clamp(u, 0.0f, 1.0f);
			v = clamp(v, 0.0f, 1.0f);
		}
		else
		{
			u = fmodf(u, 1.0f);
			if (u < 0.0f)
			{
				u += 1.0f;
			}

			v = fmodf(v, 1.0f);
			if (v < 0.0f)
			{
				v += 1.0f;
			}
		}

		float su = u * (float)(width - 1);
		float sv = v * (float)(height - 1);

		float tu, tv;

		float bu, bv;

		bu = (int)su;
		bv = (int)sv;


		tu = su - bu;
		tv = sv - bv;

		int uLo, uHi, vLo, vHi;

		if constexpr (useClamp)
		{
			uLo = clamp((int)bu, 0, width - 1);
			uHi = clamp(uLo + 1, 0, width - 1);

			vLo = clamp((int)bv, 0, height - 1);
			vHi = clamp(vLo + 1, 0, height - 1);
		}
		else
		{
			uLo = (int)fmod(bu, (float)(width - 1));
			uHi = (uLo + 1) % width;

			vLo = (int)fmod(bv, (float)(height - 1));
			vHi = (vLo + 1) % height;
		}

		return bilinearInterpolation(tu, tv, texels[uLo + vLo * width], texels[uLo + vHi * width], texels[uHi + vLo * width], texels[uHi + vHi * width]);
	}


	inline void getLightComponents(const CoreMaterial& mat, float& diffuse, float& reflection, float& refraction)
	{
		reflection = mat.reflection.value;
		refraction = mat.refraction.value;
		diffuse = 1.0f - reflection - refraction;
		assert(diffuse >= 0 && reflection >= 0 && refraction >= 0);
		assert(diffuse <= 1 && reflection <= 1 && refraction <= 1);
	}

	inline mat4 CreateTBNMatrix(const float3& T, const float3& B, const float3& N)
	{
		assert(almost_equal(1, length(T))); // "Unormalized Vector"
		assert(almost_equal(1, length(B))); // "Unormalized Vector"
		assert(almost_equal(1, length(N))); // "Unormalized Vector"
		mat4 TBN;

		TBN[0] = T.x;
		TBN[4] = T.y;
		TBN[8] = T.z;

		TBN[1] = B.x;
		TBN[5] = B.y;
		TBN[9] = B.z;

		TBN[2] = N.x;
		TBN[6] = N.y;
		TBN[10] = N.z;
		return TBN;
	}

	inline float GetRandomFloat(float a, float b) {
		float random = ((float)rand()) / (float)RAND_MAX;
		float diff = b - a;
		float r = random * diff;
		return a + r;
	}

	bool refract(const float3& I, const float3& N, const float ior, float n1, float3& T);
	float fresnel(const float3& I, const float3& N, const float ior, float n1);


	inline float3 randomPointTriangle(const float3& A, const float3& B, const float3& C)
	{
		// From https://math.stackexchange.com/questions/1785136/generating-randomly-distributed-points-inside-a-given-triangle
		const float r1 = GetRandomFloat(0, 1);
		const float r2 = sqrtf(GetRandomFloat(0, 1));
		return (1.0f - r1) * A + r1 * ((1.0f - r2) * B + r2 * C);
	}

	inline float4 ACESFilm(float4 x)
	{
		// From https://knarkowicz.wordpress.com/2016/01/06/aces-filmic-tone-mapping-curve/
		constexpr float a = 2.51f;
		constexpr float b = 0.03f;
		constexpr float c = 2.43f;
		constexpr float d = 0.59f;
		constexpr float e = 0.14f;
		return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0f, 1.0f);
	}

	inline float4 LessThan(float4 f, float value)
	{
		// From https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
		return make_float4(
			(f.x < value) ? 1.0f : 0.0f,
			(f.y < value) ? 1.0f : 0.0f,
			(f.z < value) ? 1.0f : 0.0f,
			(f.w < value) ? 1.0f : 0.0f);
	}

	inline float4 pow4(float4 f, float4 power)
	{
		return make_float4(
			pow(f.x, power.x),
			pow(f.y, power.y),
			pow(f.z, power.z),
			pow(f.z, power.z)
		);
	}

	inline float4 lerp(float4 a, float4 b, float4 t) { return a + t * (b - a); }

	inline float4 LinearToSRGB(float4 rgb)
	{
		// From https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
		rgb = clamp(rgb, 0.0f, 1.0f);

		return lerp(
			pow4(rgb, make_float4(1.0f / 2.4f)) * 1.055f - 0.055f,
			rgb * 12.92f,
			LessThan(rgb, 0.0031308f)
		);
	}


	inline float4 SRGBToLinear(float4 rgb)
	{
		// From https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
		rgb = clamp(rgb, 0.0f, 1.0f);

		return lerp(
			pow4(((rgb + 0.055f) / 1.055f), make_float4(2.4f)),
			rgb / 12.92f,
			LessThan(rgb, 0.04045f)
		);
	}

	inline float max_component(__m128 x)
	{
		// from https://stackoverflow.com/a/46126018
		__m128 max1 = _mm_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 max2 = _mm_max_ps(x, max1);
		__m128 max3 = _mm_shuffle_ps(max2, max2, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 max4 = _mm_max_ps(max2, max3);
		float result = _mm_cvtss_f32(max4);
		return result;
	}

	inline float min_component(__m128 x)
	{
		// Modified from https://stackoverflow.com/a/46126018
		__m128 min1 = _mm_shuffle_ps(x, x, _MM_SHUFFLE(0, 0, 3, 2));
		__m128 min2 = _mm_min_ps(x, min1);
		__m128 min3 = _mm_shuffle_ps(min2, min2, _MM_SHUFFLE(0, 0, 0, 1));
		__m128 min4 = _mm_min_ps(min2, min3);
		float result = _mm_cvtss_f32(min4);
		return result;
	}
}