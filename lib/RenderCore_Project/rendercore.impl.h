#pragma once

namespace lh2core
{
	template <bool backCulling>
	float3 RenderCore::Illuminate(const float3& p, const float3& N, int instanceId, int meshId, int triId) const
	{
		float3 intensity = make_float3(0);
		for (int i = 0; i < scene.directionalLights.size(); i++)
		{
			if constexpr (backCulling)
			{
				// Note that we don't care about back facing directional lights. Glass doesn't care about diffuse
				intensity += clamp(dot(N, (-1.0f) * scene.directionalLights[i].direction), 0.0f, 1.0f) * scene.directionalLights[i].radiance;
			}
			else
			{
				// We are inside glass, no back culling
				intensity += clamp(fabs(dot(N, -1.0f * scene.directionalLights[i].direction)), 0.0f, 1.0f) * scene.directionalLights[i].radiance;
			}
		}

		float3 pToLight;
		float3 lightDir;
		float distSqr;
		float dist;
		float invDistSqr;
		float dotVal;
		for (int i = 0; i < scene.pointLights.size(); i++)
		{
			const CorePointLight& light = scene.pointLights[i];
			pToLight = light.position - p;

			distSqr = sqrlength(pToLight);

			if (distSqr < kEps)
			{
				// We are so close that we are at the same position
				intensity += light.radiance;
				continue;
			}

			invDistSqr = 1.0f / distSqr;
			if (invDistSqr < kEps)
			{
				// We are so far away it doesn't matter
				continue;
			}

			dist = sqrt(distSqr);
			lightDir = pToLight / dist;

			if constexpr (backCulling)
			{
				dotVal = dot(N, lightDir);
			}
			else
			{
				dotVal = fabs(dot(N, lightDir));
			}

			if (dotVal < kEps)
			{
				// Backculling
				// The light doesn't matter
				// Note that for glass the dot product uses the absolute value function
				continue;
			}

			// Occlusion Tests
			// Note this works only single threaded
			lightRay.SetOrigin(light.position);
			lightRay.SetDirection(lightDir * -1.0f);
			if (TestDepthScene<backCulling>(lightRay, instanceId, meshId, triId, dist))
			{
				// Occluded
				continue;
			}

			intensity += clamp(dotVal, 0.0f, 1.0f) * light.radiance * invDistSqr;
		}

		float3 lightToP;
		float cosFalloff;
		for (int i = 0; i < scene.spotLights.size(); i++)
		{
			const CoreSpotLight& light = scene.spotLights[i];
			pToLight = light.position - p;

			distSqr = sqrlength(pToLight);

			if (distSqr < kEps)
			{
				// We are so close that we are at the same position
				intensity += light.radiance;
				continue;
			}

			invDistSqr = 1.0f / distSqr;
			if (invDistSqr < kEps)
			{
				// We are so far away it doesn't matter
				continue;
			}

			dist = sqrt(distSqr);
			lightDir = pToLight / dist;

			if constexpr (backCulling)
			{
				dotVal = dot(N, lightDir);
			}
			else
			{
				dotVal = fabs(dot(N, lightDir));
			}

			if (dotVal < kEps)
			{
				// Backculling
				// The light doesn't matter
				// Note that for glass the dot product uses the absolute value function
				continue;
			}

			lightToP = lightDir * -1.0f;
			float dotLCone = dot(light.direction, lightToP);
			cosFalloff = clamp((dotLCone - light.cosOuter) / (light.cosInner - light.cosOuter), 0.0f, 1.0f);
			if (cosFalloff < kEps)
			{
				// Not enough power
				continue;
			}

			// Occlusion Tests (do we have to test back facin triangles?)
			// Note this works only single threaded
			lightRay.SetOrigin(light.position);
			lightRay.SetDirection(lightToP);
			if (TestDepthScene<backCulling>(lightRay, instanceId, meshId, triId, dist))
			{
				// Occluded
				continue;
			}

			intensity += clamp(dotVal, 0.0f, 1.0f) * light.radiance * invDistSqr * cosFalloff;
		}

		return intensity;
	}

	template <bool backCulling>
	float4 RenderCore::Trace(Ray& r, const float3& intensity, int matId, int currentDepth) const
	{
		if (intensity.x  + intensity.y + intensity.z < kEps)
		{
			// No intensity left
			return make_float4(0);
		}

		if (currentDepth > maximumDepth)
		{
			return make_float4(0);
		}

		// This works only single threaded
		if (!IntersectScene<backCulling>(r, hitInfo))
		{
			return make_float4(0);
		}

		// New info
		const float t = hitInfo.triIntercept.t;
		const float3 I = make_float3(r.Evaluate(t));
		const float3 D = make_float3(r.direction);

		const CoreTri& triangle = meshes[hitInfo.meshId].triangles[hitInfo.triId];
		const CoreMaterial& material = scene.matList[triangle.material];

		const float2 uv = 
			make_float2(triangle.u0, triangle.v0) * hitInfo.triIntercept.u 
			+ make_float2(triangle.u1, triangle.v1) * hitInfo.triIntercept.v 
			+ make_float2(triangle.u2, triangle.v2) * hitInfo.triIntercept.GetWCoord();

		// Note that this could be a texture map of normals in material
		const float3& N
			= normalize(triangle.vN0 * hitInfo.triIntercept.u + triangle.vN1 * hitInfo.triIntercept.v + triangle.vN2 * hitInfo.triIntercept.GetWCoord())
			* (hitInfo.triIntercept.backFacing ? -1.0f : 1.0f);

		float3 color = make_float3(0);

		// Prev material
		float n1;
		float3 absorption;
		float3 intensityNew;
		if (matId == -1)
		{
			n1 = scene.air.ior.value;
			absorption = scene.air.absorption.value;
		}
		else
		{
			const CoreMaterial& glassMaterial = scene.matList[matId];
			assert(glassMaterial.pbrtMaterialType == MaterialType::PBRT_GLASS);
			n1 = glassMaterial.ior.value;
			absorption = glassMaterial.absorption.value;
		}

		intensityNew = intensity * make_float3(exp(-absorption.x * t), exp(-absorption.y * t), exp(-absorption.z * t));


		switch (material.pbrtMaterialType)
		{
		case MaterialType::PBRT_GLASS:
		{
			// Pure Glass
			const float ior = material.ior.value;
			const float reflection = Fresnel(D, N, ior, n1);
			const float refraction = 1.0f - reflection;

			if (reflection > kEps)
			{
				r.SetOrigin(I);
				r.SetDirection(reflect(D, N));

				color += make_float3(Trace<backCulling>(r, intensityNew, matId, currentDepth + 1)) * reflection;
			}

			if (refraction > kEps)
			{
				float3 T;
				if (Refract(D, N, ior, n1, T))
				{
					r.SetOrigin(I);
					r.SetDirection(normalize(T));

					if (matId != -1 && triangle.material == matId)
					{
						// From glass to air
						color += make_float3(Trace<true>(r, intensityNew, -1, currentDepth + 1)) * refraction;
					}
					else
					{
						// Air to glass
						assert(matId == -1);
						color += make_float3(Trace<false>(r, intensityNew, triangle.material, currentDepth + 1)) * refraction;
					}
					
				}
			}

		}
		break;
		default:
		{
			// Non-pure glass (these can be fake values)
			float diffuse, reflection, refraction;
			getLightComponents(material, diffuse, reflection, refraction);

			if (diffuse > kEps)
			{
				color += diffuse * Illuminate<backCulling>(I, N, -1, hitInfo.meshId, hitInfo.triId);
			}

			if (reflection > kEps)
			{
				r.SetOrigin(I);
				r.SetDirection(reflect(D, N));

				color += make_float3(Trace<backCulling>(r, intensityNew, matId, currentDepth + 1)) * reflection;
			}

			if (refraction > kEps)
			{
				const float ior = material.ior.value;
				float3 T;
				if (Refract(D, N, ior, n1, T))
				{
					r.SetOrigin(I);
					r.SetDirection(normalize(T));
					if (matId != -1 && triangle.material == matId)
					{
						// From glass to air
						color += make_float3(Trace<true>(r, intensityNew, -1, currentDepth + 1)) * refraction;
					}
					else
					{
						// Air to glass
						assert(matId == -1);
						color += make_float3(Trace<false>(r, intensityNew, triangle.material, currentDepth + 1)) * refraction;
					}
				}
			}
		}
		break;
		}

		// Note that N should could be flipped if this is glass
		return make_float4(intensityNew * color, 1.0f) * LoadMaterialFloat4(material.color, uv);
	}

} // lh2core