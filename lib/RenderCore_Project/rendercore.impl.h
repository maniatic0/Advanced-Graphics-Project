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
	float3 RenderCore::DirectLighting(const float3& p, const float3& N, int instanceId, int meshId, int triID) const
	{
		float3 position;
		float3 intensity = make_float3(0);
		float3 lightToP;
		float3 lightDir;
		float distSqr;
		float dist;
		float invDistSqr;
		float cos_i;
		float cos_o;
		for (int i = 0; i < scene.areaLights.size(); i++)
		{
			const CoreLightTri& light = scene.areaLights[i];
			position = randomPointTriangle(light.vertex0, light.vertex1, light.vertex2);
			lightToP = position - p;

			distSqr = sqrlength(lightToP);

			if (distSqr < kEps)
			{
				// We are so close that we are at the same position
				continue;
			}

			invDistSqr = 1.0f / distSqr;
			if (invDistSqr < kEps)
			{
				// We are so far away it doesn't matter
				continue;
			}

			dist = sqrt(distSqr);
			lightDir = lightToP / dist;

			if constexpr (backCulling)
			{
				cos_i = dot(N, -lightDir);
			}
			else
			{
				cos_i = fabs(dot(N, -lightDir));
			}

			cos_o = dot(lightDir, light.N);

			if (cos_i < kEps || cos_o < kEps)
			{
				// Backculling
				// The light doesn't matter
				// Note that for glass the dot product uses the absolute value function
				continue;
			}

			// Occlusion Tests
			// Note this works only single threaded
			lightRay.SetOrigin(position);
			lightRay.SetDirection(lightDir);
			if (TestDepthScene<backCulling>(lightRay, instanceId, meshId, triID, dist))
			{
				// Occluded
				continue;
			}

			intensity +=  clamp(cos_i, 0.0f, 1.0f) * light.radiance * clamp(cos_o, 0.0f, 1.0f) * light.area * invDistSqr;
		}
		// We don't have N or light count because we sample once for each light
		return intensity;
	}

	template <bool backCulling>
	float4 RenderCore::Trace(Ray& r, const float3& intensity, int matId, int currentDepth) const
	{
		if (intensity.x + intensity.y + intensity.z < kEps)
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
			const float reflection = fresnel(D, N, ior, n1);
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
				if (refract(D, N, ior, n1, T))
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
				if (refract(D, N, ior, n1, T))
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

		return make_float4(intensityNew * color, 1.0f) * LoadMaterialFloat4(material.color, uv);
	}

	template <bool backCulling>
	float4 RenderCore::Sample(Ray& r, const float3& intensity, int matId, int currentDepth) const
	{
		if (intensity.x + intensity.y + intensity.z < kEps)
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

		const float3& N
			= normalize(triangle.vN0 * hitInfo.triIntercept.u + triangle.vN1 * hitInfo.triIntercept.v + triangle.vN2 * hitInfo.triIntercept.GetWCoord())
			* (hitInfo.triIntercept.backFacing ? -1.0f : 1.0f);

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

		if (triangle.ltriIdx != -1)
		{
			// Area light
			const CoreLightTri& light = scene.areaLights[triangle.ltriIdx];
			if (material.color.textureID != -1)
			{
				return make_float4(intensityNew * light.radiance, 1.0f) * LoadMaterialFloat4(material.color, uv);
			}
			return make_float4(intensityNew * light.radiance, 1.0f);
		}

		switch (material.pbrtMaterialType)
		{
		case MaterialType::PBRT_GLASS:
		{
			// Pure Glass
			const float ior = material.ior.value;
			const float reflection = fresnel(D, N, ior, n1);
			const float refraction = 1.0f - reflection;

			const float path = GetRandomFloat(0.0f, 1.0f);

			const float p0 = refraction <= reflection ? refraction : reflection;
			const MaterialType t0 = refraction <= reflection ? MaterialType::PBRT_GLASS : MaterialType::PBRT_MIRROR;
			const float p1 = refraction <= reflection ? reflection : refraction;
			const MaterialType t1 = refraction <= reflection ? MaterialType::PBRT_MIRROR : MaterialType::PBRT_GLASS;

			const MaterialType selectedType = path < p0 ? t0 : t1;

			switch (selectedType)
			{
			case MaterialType::PBRT_GLASS:
			{
				// Refraction ray
				float3 T;
				if (refract(D, N, ior, n1, T))
				{
					r.SetOrigin(I);
					r.SetDirection(normalize(T));

					if (matId != -1 && triangle.material == matId)
					{
						// From glass to air
						return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<true>(r, intensityNew, -1, currentDepth + 1);
					}
					else
					{
						// Air to glass
						assert(matId == -1);
						return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<false>(r, intensityNew, triangle.material, currentDepth + 1);
					}
				}
				else
				{
					assert(false); // What
					return make_float4(0);
				}
			}
			break;
			case MaterialType::PBRT_MIRROR:
			{
				// Reflection Ray
				r.SetOrigin(I);
				r.SetDirection(reflect(D, N));
				return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<backCulling>(r, intensityNew, matId, currentDepth + 1);
			}
			break;
			default:
				assert(false);
				break;
			}
			assert(false);
		}
		break;
		case MaterialType::PBRT_MIRROR:
		{
			// Pure Mirror
			r.SetOrigin(I);
			r.SetDirection(reflect(D, N));
			return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<backCulling>(r, intensityNew, matId, currentDepth + 1);
		}
		break;
		case MaterialType::PBRT_MATTE:
		{
			const float3 R = DiffuseReflection(N, triangle);
			const float cosR = dot(N, R);
			assert(cosR >= -10.0f * kEps);
			r.SetOrigin(I);
			r.SetDirection(R);

			const float3 directLight = directIndirectMix > kEps ? DirectLighting<backCulling>(I, N, -1, hitInfo.meshId, hitInfo.triId) * INVPI : make_float3(0);

			if (cosR < kEps)
			{
				// No indirect light from here. Only direct
				return make_float4(intensityNew * directLight * directIndirectMix, 1.0f)  * LoadMaterialFloat4(material.color, uv);
			}

			const float4 indirectLight = (1.0f - directIndirectMix) > kEps ? 2.0f * cosR * Sample<backCulling>(r, intensityNew, matId, currentDepth + 1) : make_float4(4); // We omit PI from indirect because it is cancelled

			// Direct and Indirect Lighting
			return
				make_float4(intensityNew, 1.0f) *  LoadMaterialFloat4(material.color, uv) 
					* lerp(make_float4(directLight, 1.0f), indirectLight, directIndirectMix);
		}
		break;
		default:
		{
			// Non-pure glass (these can be fake values)
			float diffuse, reflection, refraction;
			getLightComponents(material, diffuse, reflection, refraction);

			const float path = GetRandomFloat(0.0f, 1.0f);

			float p0 = diffuse;
			MaterialType t0 = MaterialType::PBRT_MATTE;

			float p1 = reflection;
			MaterialType t1 = MaterialType::PBRT_MIRROR;

			float p2 = refraction;
			MaterialType t2 = MaterialType::PBRT_GLASS;

			// Sorting
			if (p0 > p1)
			{
				std::swap(p0, p1);
				std::swap(t0, t1);
			}

			if (p0 > p2)
			{
				std::swap(p0, p2);
				std::swap(t0, t2);
			}

			//Now the smallest element is the 1st one. Just check the 2nd and 3rd

			if (p1 > p2)
			{
				std::swap(p1, p2);
				std::swap(t1, t2);
			}

			MaterialType selectedType;

			if (path < p0)
			{
				selectedType = t0;
			}
			else if (path < p1)
			{
				selectedType = t1;
			}
			else
			{
				selectedType = t2;
			}

			switch (selectedType)
			{
			case MaterialType::PBRT_MATTE:
			{
				const float3 R = DiffuseReflection(N, triangle);
				const float cosR = dot(N, R);
				assert(cosR >= -10.0f * kEps);
				r.SetOrigin(I);
				r.SetDirection(R);

				const float3 directLight = directIndirectMix > kEps ? DirectLighting<backCulling>(I, N, -1, hitInfo.meshId, hitInfo.triId) * INVPI : make_float3(0);

				if (cosR < kEps)
				{
					// No indirect light from here. Only direct
					return make_float4(intensityNew * directLight * directIndirectMix, 1.0f) * LoadMaterialFloat4(material.color, uv);
				}

				const float4 indirectLight = (1.0f - directIndirectMix) > kEps ? 2.0f * cosR * Sample<backCulling>(r, intensityNew, matId, currentDepth + 1) : make_float4(0); // We omit PI from indirect because it is cancelled

				// Direct and Indirect Lighting
				return
					make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv)
					* lerp(make_float4(directLight, 1.0f), indirectLight, directIndirectMix);
			}
			break;
			case MaterialType::PBRT_GLASS:
			{
				// Refraction ray
				const float ior = material.ior.value;
				float3 T;
				if (refract(D, N, ior, n1, T))
				{
					r.SetOrigin(I);
					r.SetDirection(normalize(T));

					if (matId != -1 && triangle.material == matId)
					{
						// From glass to air
						return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<true>(r, intensityNew, -1, currentDepth + 1);
					}
					else
					{
						// Air to glass
						assert(matId == -1);
						return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<false>(r, intensityNew, triangle.material, currentDepth + 1);
					}
				}
				else
				{
					// Below TIR
					return make_float4(0);
				}
			}
			break;
			case MaterialType::PBRT_MIRROR:
			{
				// Reflection Ray
				r.SetOrigin(I);
				r.SetDirection(reflect(D, N));
				return make_float4(intensityNew, 1.0f) * LoadMaterialFloat4(material.color, uv) * Sample<backCulling>(r, intensityNew, matId, currentDepth + 1);
			}
			break;
			default:
				assert(false);
				break;
			}
			assert(false);
		}
		break;
		}
		assert(false);
		return make_float4(0);
	}
} // lh2core