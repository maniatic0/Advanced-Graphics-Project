/* rendercore.cpp - Copyright 2019/2020 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#include "core_settings.h"

using namespace lh2core;

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Init                                                           |
//  |  Initialization.                                                      LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Init()
{
	// initialize
	maximumDepth = 10;

	aaLevel = 1;
	assert(0 < aaLevel && aaLevel <= pixelOffsetsSize);
	invAaLevel = 1.0f / (float)(aaLevel);

	distortionType = DistortionType::None;

	gamma = 2.2f;

	useChromaticAberration = false;
	aberrationUOffset = make_float3(0);
	aberrationVOffset = make_float3(0);
	aberrationRadialK = make_float3(0);
	chromaticAberrationScale = 0.0075f;

	useVignetting = false;
	sigma = 500;
	kernel = nullptr;


	renderType = RenderType::Whitted;

	historyMix = 0.8f;
	offsetIter = 0;

	indirectDirectMix = 1.0f;

	useToneMapping = false;

	exposure = 0.5f;

	BVH4::PrepareBVH4Tables();

	bvhType = BVH_Type::BVH4;
}

void RenderCore::Setting(const char* name, float value)
{
	if (!strcmp(name, "gamma"))
	{
		gamma = value;
	}
	else if (!strcmp(name, "exposure"))
	{
		exposure = value;
	}
	else if (!strcmp(name, "max_depth"))
	{
		maximumDepth = (int)fabs(value);
	}
	else if (!strcmp(name, "render_type"))
	{
		renderType = (RenderType)clamp((int)value, 0, (int)RenderType::Count - 1);
	}
	else if (!strcmp(name, "bvh_type"))
	{
		bvhType = (BVH_Type)clamp((int)value, 0, (int)BVH_Type::Count - 1);
	}
	else if (!strcmp(name, "indirect_direct_mix"))
	{
		indirectDirectMix = clamp(value, 0.0f, 1.0f);
	}
	else if (!strcmp(name, "tone_mapping_enabled"))
	{
		useToneMapping = (bool)value;
	}
	else if (!strcmp(name, "aa_mix"))
	{
		historyMix = clamp(value, 0.0f, 1.0f);
	}
	else if (!strcmp(name, "aa_level"))
	{
		aaLevel = (int)clamp(value, 1.0f, (float)pixelOffsetsSize);
		invAaLevel = 1.0f / (float)(aaLevel);
	}
	else if (!strcmp(name, "distortion_type"))
	{
		distortionType = (DistortionType)clamp((int)value, 0, (int)DistortionType::Count - 1);
	}
	else if (!strcmp(name, "vignetting_enabled"))
	{
		useVignetting = (bool)value;
	}
	else if (!strcmp(name, "sigma"))
	{
		sigma = value;
		if (kernel != nullptr)
		{
			CreateGaussianKernel(screen->width, screen->height);
		}
	}
	else if (!strcmp(name, "chromatic_aberration_enabled"))
	{
		useChromaticAberration = (bool)value;
	}
	else if (!strcmp(name, "chromatic_aberration_scale"))
	{
		chromaticAberrationScale = value;
	}
	else if (!strcmp(name, "chromatic_u_offset_r"))
	{
		aberrationUOffset.x = value;
	}
	else if (!strcmp(name, "chromatic_u_offset_g"))
	{
		aberrationUOffset.y = value;
	}
	else if (!strcmp(name, "chromatic_u_offset_b"))
	{
		aberrationUOffset.z = value;
	}
	else if (!strcmp(name, "chromatic_v_offset_r"))
	{
		aberrationVOffset.x = value;
	}
	else if (!strcmp(name, "chromatic_v_offset_g"))
	{
		aberrationVOffset.y = value;
	}
	else if (!strcmp(name, "chromatic_v_offset_b"))
	{
		aberrationVOffset.z = value;
	}
	else if (!strcmp(name, "chromatic_radial_k_r"))
	{
		aberrationRadialK.x = value;
	}
	else if (!strcmp(name, "chromatic_radial_k_g"))
	{
		aberrationRadialK.y = value;
	}
	else if (!strcmp(name, "chromatic_radial_k_b"))
	{
		aberrationRadialK.z = value;
	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget(GLTexture* target, const uint)
{
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	delete fscreen;
	delete kernel;
	delete accumulationBuffer;
	screen = new Bitmap(target->width, target->height);
	fscreen = new float4[static_cast<size_t>(target->width) * static_cast<size_t>(target->height)];
	accumulationBuffer = new float4[static_cast<size_t>(target->width) * static_cast<size_t>(target->height)];
	for (size_t i = 0; i < static_cast<size_t>(target->width) * static_cast<size_t>(target->height); i++)
	{
		accumulationBuffer[i] = make_float4(0);
	}

	// dynamically allocate kernel
	kernel = new float[static_cast<size_t>(target->width) * static_cast<size_t>(target->height)];
	CreateGaussianKernel(target->width, target->height);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry(const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData)
{
	assert(triangleCount * 3 == vertexCount);
	Timer timer;
	timer.reset();
	Mesh newMesh;
	// copy the supplied vertices; we cannot assume that the render system does not modify
	// the original data after we leave this function.
	newMesh.meshID = static_cast<int>(scene.meshBVH.size()); // TODO: Change this to keep the meshIdx
	newMesh.vertices = make_unique<float4[]>(vertexCount);
	newMesh.vcount = vertexCount;
	memcpy(newMesh.vertices.get(), vertexData, vertexCount * sizeof(float4));
	// copy the supplied 'fat triangles'
	newMesh.triangles = make_unique<CoreTri[]>(vertexCount / 3);
	memcpy(newMesh.triangles.get(), triangleData, (vertexCount / 3) * sizeof(CoreTri));
	printf("copied data for BVH from mesh-%d (triangle count %d) in %5.3fs\n", meshIdx, triangleCount, timer.elapsed());

	BVH& bvh = scene.meshBVH.emplace_back();
	bvh.ConstructBVH(bvhType, std::forward<Mesh>(newMesh));

#ifdef MEASURE_BVH
	aabb bounds = bvh.GetBounds();
	Ray r(bounds.bmax3, normalize(make_float3(bounds.Center(0), bounds.Center(1), bounds.Center(2)) - bounds.bmax3));

	RayMeshInterceptInfo hitInfo;
	hitInfo.Reset();
	bvh.IntersectRayBVH<true>(r, hitInfo);
	bvh.DepthRayBVH<true>(r, -1, -1, hitInfo.triIntercept.t);
#endif

}

void RenderCore::SetTextures(const CoreTexDesc* tex, const int textureCount)
{
	scene.texList.resize(textureCount);
	// copy the supplied array of materials
	for (int i = 0; i < textureCount; i++)
	{
		scene.texList[i] = tex[i];
		switch (tex[i].storage)
		{
		case ARGB128:
		{
			scene.texList[i].fdata = new float4[scene.texList[i].pixelCount];
			memcpy(scene.texList[i].fdata, tex[i].fdata, tex[i].pixelCount * sizeof(float4));
		}
		break;
		case ARGB32:
		{
			scene.texList[i].idata = new uchar4[scene.texList[i].pixelCount];
			memcpy(scene.texList[i].idata, tex[i].idata, tex[i].pixelCount * sizeof(uchar4));
		}
		break;
		default:
			assert(false);
			break;
		}

	}
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid& view, const Convergence converge, bool async)
{
	// render
	//screen->Clear(); // TODO: un comment when we have achieved useful times
	Timer timer;
	timer.reset();

	Ray ray;
	float3 intensity = make_float3(1);
	float3 dir;

	float v, u;
	const float scrWidth = (float)screen->width;
	const float scrHeight = (float)screen->height;
	const float invHeight = 1.0f / scrHeight;
	const float invWidth = 1.0f / scrWidth;
	float pixelOffsetU = 0.5f;
	float pixelOffsetV = 0.5f;
	uint aaOffset = 0;

	uint base, base2;

	// Clear screen
	for (uint y = 0; y < screen->height; y++)
	{
		base = y * screen->width;
		for (uint x = 0; x < screen->width; x++)
		{
			base2 = x + base;
			fscreen[base2] = make_float4(0);
		}
	}

	// AA
	for (size_t i = 0; i < aaLevel; i++)
	{
		// AA offsets
		aaOffset = ((i + offsetIter) % pixelOffsetsSize) * 2;
		pixelOffsetU = pixelOffSets[aaOffset + 0];
		pixelOffsetV = pixelOffSets[aaOffset + 1];

		// Fill Screen
		for (uint y = 0; y < screen->height; y++)
		{
			base = y * screen->width;
			for (uint x = 0; x < screen->width; x++)
			{
				base2 = x + base;

				u = 0;
				v = 0;

				// Distortion Effects
				switch (distortionType)
				{
				case DistortionType::None:
				{
					v = ((float)y + 0.5f + pixelOffsetU) * invHeight;
					u = ((float)x + 0.5f + pixelOffsetV) * invWidth;
				}
				break;
				case DistortionType::Barrel:
				{

					v = ((float)y + 0.5f + pixelOffsetU) * invHeight;
					u = ((float)x + 0.5f + pixelOffsetV) * invWidth;

					// From https://www.geeks3d.com/20140213/glsl-shader-library-fish-eye-and-dome-and-barrel-distortion-post-processing-filters/2/
					float us = 2.0f * u - 1.0f;
					float vs = 2.0f * v - 1.0f;
					const float d = us * us + vs * vs;
					if (d < 1.0)
					{
						// Only apply near the center (0.5, 0.5)
						const float theta = atan2f(vs, us);
						const float radius = pow(sqrtf(d), view.distortion);
						us = radius * cosf(theta);
						vs = radius * sinf(theta);
						u = 0.5f * (us + 1.0f);
						v = 0.5f * (vs + 1.0f);
					}
				}
				break;
				case DistortionType::FishEye:
				{
					v = ((float)y + 0.5f + pixelOffsetU) * invHeight;
					u = ((float)x + 0.5f + pixelOffsetV) * invWidth;

					// From https://www.geeks3d.com/20140213/glsl-shader-library-fish-eye-and-dome-and-barrel-distortion-post-processing-filters/
					const float aperture = 180.0f * view.aperture;
					const float apertureHalf = 0.5f * aperture * (PI / 180.0f);
					const float maxFactor = sin(apertureHalf);


					float us = 2.0f * u - 1.0f;
					float vs = 2.0f * v - 1.0f;
					const float d = sqrtf(us * us + vs * vs);
					if (d < 1.0f)
					{
						const float d2 = d * maxFactor;
						const float z = sqrtf(1.0f - d * d);
						const float r = atan2(d2, z) / PI;
						const float phi = atan2(vs, us);

						u = r * cosf(phi) + 0.5f;
						v = r * sinf(phi) + 0.5f;
					}
				}
				break;
				case DistortionType::BarrelSpecial:
				{
					if (view.distortion == 0)
					{
						v = ((float)y + 0.5f + pixelOffsetU) * invHeight;
						u = ((float)x + 0.5f + pixelOffsetV) * invWidth;
					}
					else
					{
						// Barrel Distortion centered at 0.5, 0.5
						const float tx = (x + pixelOffsetU) * invWidth - 0.5f;
						const float ty = (y + pixelOffsetV) * invHeight - 0.5f;
						const float rr = tx * tx + ty * ty;
						const float rq = sqrtf(rr) * (1.0f + view.distortion * rr + view.distortion * rr * rr);
						const float theta = atan2f(tx, ty);
						const float bx = (sinf(theta) * rq + 0.5f) * scrWidth;
						const float by = (cosf(theta) * rq + 0.5f) * scrHeight;
						u = (bx + 0.5f) * invWidth;
						v = (by + 0.5f) * invHeight;
					}
				}
				break;
				case DistortionType::Count:
				default:
					assert(false); // What are you doing here
					break;
				}



				dir = normalize(view.p1 + u * (view.p2 - view.p1) + v * (view.p3 - view.p1) - view.pos);

				ray.SetOrigin(view.pos);
				ray.SetDirection(dir);

				switch (renderType)
				{
				case RenderCore::RenderType::Whitted:
					fscreen[base2] += Trace<true>(ray, intensity, -1, 0) * invAaLevel;
					break;
				case RenderCore::RenderType::PathTracer:
					fscreen[base2] += Sample<true>(ray, intensity, -1, 0) * invAaLevel;
					break;
				default:
					assert(false); // What are you doing here
					break;
				}
			}
		}
	}

	// HDR to 255 colors
	float4 tempColor;
	const float gammaCorrection = 1.0f / gamma;

	// History mix and render if possible
	for (uint y = 0; y < screen->height; y++)
	{
		base = y * screen->width;
		for (uint x = 0; x < screen->width; x++)
		{
			base2 = x + base;
			accumulationBuffer[base2] = lerp(fscreen[base2], accumulationBuffer[base2], historyMix);
			
			tempColor = accumulationBuffer[base2];

			//if (!useChromaticAberration)
			{
				// We are ready to render to screen if no more to do. In case we have more to do, we just overwrite
				// Order in part from https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
				if (useToneMapping)
				{
					tempColor = ACESFilm(tempColor);
					tempColor.w = 1.0f;
				}

				tempColor.x = pow(tempColor.x, gammaCorrection);
				tempColor.y = pow(tempColor.y, gammaCorrection);
				tempColor.z = pow(tempColor.z, gammaCorrection);

				if (useVignetting)
				{
					tempColor *= kernel[base2];
				}

				screen->Plot(x, y, float4ToUint(LinearToSRGB(tempColor)));
			}
		}
	}

	if (useChromaticAberration)
	{
		// Chromatic aberration modified from https://www.shadertoy.com/view/wl2SDt
		for (uint y = 0; y < screen->height; y++)
		{
			base = y * screen->width;
			const float pv = ((float)y + 0.5f) * invHeight;

			for (uint x = 0; x < screen->width; x++)
			{
				base2 = x + base;

				const float pu = ((float)x + 0.5f) * invWidth;

				// AA Aberration
				tempColor = make_float4(0);
				for (int i = 0; i < aaLevel; i++)
				{
					// AA offsets
					aaOffset = i * 2;
					pixelOffsetU = pixelOffSets[aaOffset + 0];
					pixelOffsetV = pixelOffSets[aaOffset + 1];

					const float u = pu + pixelOffsetU * invWidth;
					const float v = pv + pixelOffsetV * invHeight;

					const float cvR = v + aberrationVOffset.x - 0.5f;
					const float cvG = v + aberrationVOffset.y - 0.5f;
					const float cvB = v + aberrationVOffset.z - 0.5f;

					const float cuR = u + aberrationUOffset.x - 0.5f;
					const float cuG = u + aberrationUOffset.y - 0.5f;
					const float cuB = u + aberrationUOffset.z - 0.5f;

					const float uR = u + aberrationRadialK.x * cuR * chromaticAberrationScale;
					const float uG = u + aberrationRadialK.y * cuG * chromaticAberrationScale;
					const float uB = u + aberrationRadialK.z * cuB * chromaticAberrationScale;

					const float vR = v + aberrationRadialK.x * cvR * chromaticAberrationScale;
					const float vG = v + aberrationRadialK.y * cvG * chromaticAberrationScale;
					const float vB = v + aberrationRadialK.z * cvB * chromaticAberrationScale;

					tempColor.w += textureFetch<true>(accumulationBuffer, screen->width, screen->height, u, v).w;
					tempColor.x += textureFetch<true>(accumulationBuffer, screen->width, screen->height, uR, vR).x;
					tempColor.y += textureFetch<true>(accumulationBuffer, screen->width, screen->height, uG, vG).y;
					tempColor.z += textureFetch<true>(accumulationBuffer, screen->width, screen->height, uB, vB).z;

				}

				tempColor *= invAaLevel;

				// Order in part from https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
				tempColor *= exposure;
				if (useToneMapping)
				{
					tempColor = ACESFilm(tempColor);
					tempColor.w = 1.0f;
				}

				tempColor.x = pow(tempColor.x, gammaCorrection);
				tempColor.y = pow(tempColor.y, gammaCorrection);
				tempColor.z = pow(tempColor.z, gammaCorrection);

				if (useVignetting)
				{
					tempColor *= kernel[base2];
				}

				screen->Plot(x, y, float4ToUint(LinearToSRGB(tempColor)));
			}
		}
	}

	elapsedTime += timer.elapsed();

	offsetIter = (offsetIter + 1) % pixelOffsetsSize;

	printf("rendered the complete screen in %5.3fs\n", elapsedTime);
	elapsedTime = 0.f;

	// copy pixel buffer to OpenGL render target texture
	glBindTexture(GL_TEXTURE_2D, targetTextureID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::GetCoreStats                                                   |
//  |  Get a copy of the counters.                                          LH2'19|
//  +-----------------------------------------------------------------------------+
CoreStats RenderCore::GetCoreStats() const
{
	return coreStats;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Shutdown                                                       |
//  |  Free all resources.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Shutdown()
{
	for (size_t i = 0; i < scene.texList.size(); i++)
	{
		switch (scene.texList[i].storage)
		{
		case ARGB32:
			if (scene.texList[i].idata != nullptr)
			{
				delete scene.texList[i].idata;
			}
			break;
		case ARGB128:
			if (scene.texList[i].fdata != nullptr)
			{
				delete scene.texList[i].fdata;
			}
			break;
		default:
			break;
		}

	}

	if (screen == nullptr) return; // Nothing to release
	delete screen;
	delete fscreen;
	delete kernel;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetMaterials                                                   |
//  |  Set the material data.                                               LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetMaterials(CoreMaterial* mat, const int materialCount)
{
	scene.matList.resize(materialCount);
	// copy the supplied array of materials
	for (int i = 0; i < materialCount; i++)
	{
		scene.matList[i] = mat[i];
	}

	// Air
	scene.air.ior.value = 1.0f;
	scene.air.absorption.value = make_float3(0);
	scene.air.pbrtMaterialType = MaterialType::PBRT_GLASS;
}

void RenderCore::SetLights(const CoreLightTri* triLights, const int triLightCount,
	const CorePointLight* pointLights, const int pointLightCount,
	const CoreSpotLight* spotLights, const int spotLightCount,
	const CoreDirectionalLight* directionalLights, const int directionalLightCount)
{
	scene.areaLights.resize(triLightCount);
	for (int i = 0; i < triLightCount; i++)
	{
		scene.areaLights[i] = triLights[i];
	}

	scene.spotLights.resize(spotLightCount);
	for (int i = 0; i < spotLightCount; i++)
	{
		scene.spotLights[i] = spotLights[i];
	}

	scene.pointLights.resize(pointLightCount);
	for (int i = 0; i < pointLightCount; i++)
	{
		scene.pointLights[i] = pointLights[i];
	}

	scene.directionalLights.resize(directionalLightCount);
	for (int i = 0; i < directionalLightCount; i++)
	{
		scene.directionalLights[i] = directionalLights[i];
	}
}

void RenderCore::CreateGaussianKernel(uint width, uint height)
{
	float meani = ((float)width - 1.0f) / 2.0f;
	float meanj = ((float)height - 1.0f) / 2.0f;
	float maxValue = 0;

	// generating kernel 
	float sigmaxy = 2 * sigma * sigma;

	int base;
	for (int j = 0; j < (int)height; j++)
	{
		base = j * width;
		for (int i = 0; i < (int)width; i++)
		{
			float value = exp(-((j - meanj) * (j - meanj) + (i - meani) * (i - meani)) / (sigmaxy));
			kernel[i + base] = value;
			if (value > maxValue)
			{
				maxValue = value;
			}
		}
	}


	// normalizing the kernel 
	for (int j = 0; j < (int)height; j++)
	{
		base = j * width;
		for (int i = 0; i < (int)width; i++)
		{
			kernel[i + base] /= maxValue;
		}
	}

}

float4 RenderCore::LoadMaterialFloat4(const CoreMaterial::Vec3Value& val, const float2& uv) const
{
	if (val.textureID == -1)
	{
		// Just one value
		assert(val.value.x != 1e-32);
		return make_float4(val.value, 1.0f);
	}

	float2 newUV = uv;

	if (val.scale != 1e-32)
	{
		newUV *= val.scale;
	}

	if (val.uvscale.x != 1e-32)
	{
		assert(val.uvscale.y != 1e-32);
		newUV *= val.uvscale;
	}

	if (val.uvoffset.x != 1e-32)
	{
		assert(val.uvoffset.y != 1e-32);
		newUV += val.uvoffset;
	}

	const CoreTexDesc& tex = scene.texList[val.textureID];

	switch (tex.storage)
	{
	case ARGB32:
		return textureFetch<false>(tex.idata, tex.width, tex.height, newUV.x, newUV.y);
	case ARGB128:
		return textureFetch<false>(tex.fdata, tex.width, tex.height, newUV.x, newUV.y);
	default:
		break;
	}

	assert(false);
	return make_float4(0);
}

float3 RenderCore::DiffuseReflection(const float3& N, const CoreTri& triangle) const
{
	// Generate a random direction on a hemisphere
	// Based of https://www.scratchapixel.com/lessons/3d-basic-rendering/global-illumination-path-tracing/global-illumination-path-tracing-practical-implementation
	float3 randomDir;
	const float u = GetRandomFloat(0, 1);
	const float v = GetRandomFloat(0, 1);
	const float angle1 = 2.0f * PI * u;
	const float sinTheta = sqrtf(1.0f - v * v);
	randomDir.x = sinf(angle1) * sinTheta;
	randomDir.y = cosf(angle1) * sinTheta;
	randomDir.z = u;
	randomDir = normalize(randomDir);

	// Convert the random direction to world space
	mat4 TBN = CreateTBNMatrix(triangle.T, triangle.B, N);
	return normalize(TBN.TransformVector(randomDir));
}


float RenderCore::pixelOffSets[RenderCore::pixelOffsetsSize * 2] =
{
   0,		   0
,  0.0000000, -0.16666667
, -0.2500000,  0.16666667
,  0.2500000, -0.38888889
, -0.3750000, -0.05555556
,  0.1250000,  0.27777778
, -0.1250000, -0.27777778
,  0.3750000,  0.05555556
, -0.4375000,  0.38888889
,  0.0625000, -0.46296296
, -0.1875000, -0.12962963
,  0.3125000,  0.20370370
, -0.3125000, -0.35185185
,  0.1875000, -0.01851852
, -0.0625000,  0.31481481
,  0.4375000, -0.24074074
, -0.4687500,  0.09259259
,  0.0312500,  0.42592593
, -0.2187500, -0.42592593
,  0.2812500, -0.09259259
, -0.3437500,  0.24074074
,  0.1562500, -0.31481481
, -0.0937500,  0.01851852
,  0.4062500,  0.35185185
, -0.4062500, -0.20370370
,  0.0937500,  0.12962963
, -0.1562500,  0.46296296
,  0.3437500, -0.48765432
, -0.2812500, -0.15432099
,  0.2187500,  0.17901235
, -0.0312500, -0.37654321
,  0.4687500, -0.04320988
, -0.4843750,  0.29012346
,  0.0156250, -0.26543210
, -0.2343750,  0.06790123
,  0.2656250,  0.40123457
, -0.3593750, -0.45061728
,  0.1406250, -0.11728395
, -0.1093750,  0.21604938
,  0.3906250, -0.33950617
, -0.4218750, -0.00617284
,  0.0781250,  0.32716049
, -0.1718750, -0.22839506
,  0.3281250,  0.10493827
, -0.2968750,  0.43827160
,  0.2031250, -0.41358025
, -0.0468750, -0.08024691
,  0.4531250,  0.25308642
, -0.4531250, -0.30246914
,  0.0468750,  0.03086420
, -0.2031250,  0.36419753
,  0.2968750, -0.19135802
, -0.3281250,  0.14197531
,  0.1718750,  0.47530864
, -0.0781250, -0.47530864
,  0.4218750, -0.14197531
, -0.3906250,  0.19135802
,  0.1093750, -0.36419753
, -0.1406250, -0.03086420
,  0.3593750,  0.30246914
, -0.2656250, -0.25308642
,  0.2343750,  0.08024691
, -0.0156250,  0.41358025
,  0.4843750, -0.43827160
, -0.4921875, -0.10493827
};

// EOF