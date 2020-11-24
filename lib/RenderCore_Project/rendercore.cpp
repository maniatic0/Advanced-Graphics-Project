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
	aaLevel = 65;
	assert(0 < aaLevel && aaLevel <= pixelOffsetsSize);
	invAaLevel = 1.0f / (float)(aaLevel);

	distortionType = DistortionType::None;
}

void RenderCore::Setting(const char* name, float value)
{
	if (!strcmp(name, "aa_level"))
	{
		aaLevel = (int)clamp(value, 1.0f, (float)pixelOffsetsSize);
		invAaLevel = 1.0f / (float)(aaLevel);
	}
	else if (!strcmp(name, "distortion_type"))
	{
		distortionType = (DistortionType)(int)clamp(value, 0.0f, (float)((int)DistortionType::Count - 1));
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
	screen = new Bitmap(target->width, target->height);
	fscreen = new float4[static_cast<size_t>(target->width) * static_cast<size_t>(target->height)];
	yScanline = 0;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry(const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData)
{
	Mesh newMesh;
	// copy the supplied vertices; we cannot assume that the render system does not modify
	// the original data after we leave this function.
	newMesh.meshID = static_cast<int>(meshes.size());
	newMesh.vertices = new float4[vertexCount];
	newMesh.vcount = vertexCount;
	memcpy(newMesh.vertices, vertexData, vertexCount * sizeof(float4));
	// copy the supplied 'fat triangles'
	newMesh.triangles = new CoreTri[vertexCount / 3];
	memcpy(newMesh.triangles, triangleData, (vertexCount / 3) * sizeof(CoreTri));
	meshes.push_back(newMesh);
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render(const ViewPyramid& view, const Convergence converge, bool async)
{
	// render
	//screen->Clear(); // TODO: un comment when we have achieved useful times
	Ray ray;
	float3 intensity = make_float3(1);
	float3 dir;

	float v, u;
	float scrWidth = (float)screen->width;
	float scrHeight = (float)screen->height;
	float invHeight = 1.0f / scrHeight;
	float invWidth = 1.0f / scrWidth;
	float pixelOffsetU = 0.5f;
	float pixelOffsetV = 0.5f;
	uint aaOffset = 0;

	uint base, base2;

	base = yScanline * screen->width;
	for (uint x = 0; x < screen->width; x++)
	{
		// AA
		fscreen[x + base] = make_float4(0);
		for (int i = 0; i < aaLevel; i++)
		{
			// AA offsets
			aaOffset = i * 2;
			pixelOffsetU = pixelOffSets[aaOffset + 0];
			pixelOffsetV = pixelOffSets[aaOffset + 1];

			u = 0;
			v = 0;

			// Distortion Effects
			switch (distortionType)
			{
			case DistortionType::None:
			{
				v = ((float)yScanline + 0.5f + pixelOffsetU) * invHeight;
				u = ((float)x + 0.5f + pixelOffsetV) * invWidth;
			}
			break;
			case DistortionType::Barrel:
			{

				v = ((float)yScanline + 0.5f + pixelOffsetU) * invHeight;
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
					us = radius * cos(theta);
					vs = radius * sin(theta);
					u = 0.5f * (us + 1.0f);
					v = 0.5f * (vs + 1.0f);
				}
			}
			break;
			case DistortionType::FishEye:
			{
				v = ((float)yScanline + 0.5f + pixelOffsetU) * invHeight;
				u = ((float)x + 0.5f + pixelOffsetV) * invWidth;

				// From https://www.geeks3d.com/20140213/glsl-shader-library-fish-eye-and-dome-and-barrel-distortion-post-processing-filters/
				const float aperture = 180.0f * view.distortion;
				const float apertureHalf = 0.5f * aperture * (PI / 180.0);
				const float maxFactor = sin(apertureHalf);


				float us = 2.0f * u - 1.0f;
				float vs = 2.0f * v - 1.0f;
				const float d = sqrtf(us * us + vs * vs);
				if (d < (2.0f - maxFactor))
				{
					const float d2 = d * maxFactor;
					const float z = sqrtf(1.0f - d * d);
					const float r = atan2(d2, z) / PI;
					const float phi = atan2(vs, us);

					u = r * cos(phi) + 0.5f;
					v = r * sin(phi) + 0.5f;
				}
			}
			break;
			case DistortionType::BarrelSpecial:
			{
				if (view.distortion == 0)
				{
					v = ((float)yScanline + 0.5f + pixelOffsetU) * invHeight;
					u = ((float)x + 0.5f + pixelOffsetV) * invWidth;
				}
				else
				{
					// Barrel Distortion centered at 0.5, 0.5
					const float tx = (x + pixelOffsetU) * invWidth - 0.5f;
					const float ty = (yScanline + pixelOffsetV) * invHeight - 0.5f;
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

			fscreen[x + base] += Trace<true>(ray, intensity, -1, 0);
		}
		fscreen[x + base] *= invAaLevel;

		// UV test
		//fscreen[x + base] = make_float4(u, v, 0, 1);
		//fscreen[x + base] = make_float4(0, 0, 0, 1);
	}

	uint color;
	// HDR to 255 colors
	base = yScanline * screen->width;
	const float gammaCorrection = 1 / 2.2f;
	for (uint x = 0; x < screen->width; x++)
	{
		base2 = x + base;
		fscreen[base2].x = pow(fscreen[base2].x, gammaCorrection);
		fscreen[base2].y = pow(fscreen[base2].y, gammaCorrection);
		fscreen[base2].z = pow(fscreen[base2].z, gammaCorrection);

		fscreen[base2] *= 255.0f;
		fscreen[base2] = clamp(fscreen[base2], 0, 255);
		// AABBGGRR
		color =
			((((uint)fscreen[base2].w) << 24) & 0xFF000000)
			| ((((uint)fscreen[base2].z) << 16) & 0xFF0000)
			| ((((uint)fscreen[base2].y) << 8) & 0xFF00)
			| (((uint)fscreen[base2].x) & 0xFF);
		screen->Plot(x, yScanline, color);
	}

	yScanline = (yScanline + 1) % screen->height;

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
	delete screen;
	delete fscreen;
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

bool RenderCore::Refract(const float3& I, const float3& N, const float ior, float n1, float3& T)
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

float RenderCore::Fresnel(const float3& I, const float3& N, const float ior, float n1)
{
	// https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
	float cosi = clamp(dot(I, N), -1.0f, 1.0f);
	float etai = n1, etat = ior;
	if (cosi > 0) { std::swap(etai, etat); }
	// Compute sini using Snell's law
	float sint = etai / etat * sqrtf(fmax(0.f, 1 - cosi * cosi));
	// Total internal reflection
	if (sint >= 1 + kEps) {
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