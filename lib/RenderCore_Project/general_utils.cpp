#include "core_settings.h"

namespace lh2core
{

	bool refract(const float3& I, const float3& N, const float ior, float n1, float3& T)
	{
		// Based on https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
		float cosi = clamp(dot(I, N), -1.0f, 1.0f);
		float n2 = ior;
		float flipN = 1.0f;
		if (cosi < 0)
		{
			cosi = -cosi;
		}
		else
		{
			std::swap(n1, n2);
			flipN = -1.0f;
		}
		float eta = n1 / n2;
		float k = 1 - eta * eta * (1 - cosi * cosi);
		if (k < kEps)
		{
			T = make_float3(0);
			return false;
		}
		else
		{
			T = (I * eta) + (N * flipN) * (eta * cosi - sqrtf(k));
			return true;
		}
	}

	float fresnel(const float3& I, const float3& N, const float ior, float n1)
	{
		// https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
		float cosi = clamp(dot(I, N), -1.0f, 1.0f);
		float etai = n1, etat = ior;
		if (cosi > 0) { std::swap(etai, etat); }
		// Compute sini using Snell's law
		float sint = etai / etat * sqrtf(fmax(0.f, 1 - cosi * cosi));

		if (sint >= 1 + kEps) {
			// Total internal reflection
			return 1;
		}
		else {
			sint = clamp(sint, -1.0f, 1.0f);
			float cost = sqrtf(std::max(0.f, 1 - sint * sint));
			cosi = fabs(cosi);
			float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
			float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
			return (Rs * Rs + Rp * Rp) / 2.0f;
		}
		// As a consequence of the conservation of energy, transmittance is given by:
		// kt = 1 - kr;
	}
}