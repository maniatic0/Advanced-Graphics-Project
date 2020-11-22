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
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetTarget                                                      |
//  |  Set the OpenGL texture that serves as the render target.             LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetTarget( GLTexture* target, const uint )
{
	// synchronize OpenGL viewport
	targetTextureID = target->ID;
	if (screen != 0 && target->width == screen->width && target->height == screen->height) return; // nothing changed
	delete screen;
	delete fscreen;
	screen = new Bitmap( target->width, target->height );
	fscreen = new float4[static_cast<size_t>(target->width) * static_cast<size_t>(target->height)];
	yScanline = 0;
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::SetGeometry                                                    |
//  |  Set the geometry data for a model.                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangleData )
{
	Mesh newMesh;
	// copy the supplied vertices; we cannot assume that the render system does not modify
	// the original data after we leave this function.
	newMesh.meshID = static_cast<int>(meshes.size());
	newMesh.vertices = new float4[vertexCount];
	newMesh.vcount = vertexCount;
	memcpy( newMesh.vertices, vertexData, vertexCount * sizeof( float4 ) );
	// copy the supplied 'fat triangles'
	newMesh.triangles = new CoreTri[vertexCount / 3];
	memcpy( newMesh.triangles, triangleData, (vertexCount / 3) * sizeof( CoreTri ) );
	meshes.push_back( newMesh );
}

//  +-----------------------------------------------------------------------------+
//  |  RenderCore::Render                                                         |
//  |  Produce one image.                                                   LH2'19|
//  +-----------------------------------------------------------------------------+
void RenderCore::Render( const ViewPyramid& view, const Convergence converge, bool async )
{
	// render
	//screen->Clear(); // TODO: un comment when we have achieved useful times
	Ray ray;
	float3 intensity = make_float3(1);

	float v, u;
	float invHeight = 1.0f / (float)screen->height;
	float invWidth = 1.0f / (float)screen->width;
	uint base, base2;

	v = ((float)yScanline + 0.5f) * invHeight;
	base = yScanline * screen->width;
	for (uint x = 0; x < screen->width; x++)
	{
		u = ((float)x + 0.5f) * invWidth;
		float3 dir = normalize(view.p1 + u * (view.p2 - view.p1) + v * (view.p3 - view.p1) - view.pos);

		ray.SetOrigin(view.pos);
		ray.SetDirection(dir);

		fscreen[x + base] = Trace<true>(ray, intensity, -1, 0);
		//fscreen[x + base] = make_float4(u, v, 0, 1);
		//fscreen[x + base] = make_float4(0, 0, 0, 1);
	}

	uint color;
	// HDR to 255 colors
	base = yScanline * screen->width;
	for (uint x = 0; x < screen->width; x++)
	{
		base2 = x + base;
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
	glBindTexture( GL_TEXTURE_2D, targetTextureID );
	glTexImage2D( GL_TEXTURE_2D, 0, GL_RGBA, screen->width, screen->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, screen->pixels );
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

bool RenderCore::Refract(const float3 &I, const float3 &N, const float ior, float n1, float3 &T)
{
	// Based on https://www.scratchapixel.com/lessons/3d-basic-rendering/introduction-to-shading/reflection-refraction-fresnel
	float cosi = clamp( dot(I, N), -1.0f, 1.0f);
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

// EOF