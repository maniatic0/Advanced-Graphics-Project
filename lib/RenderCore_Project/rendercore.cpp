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

		fscreen[x + base] = Trace(ray);
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

float4 RenderCore::Trace(Ray& r, int currentDepth) const
{
	if (currentDepth > maximumDepth)
	{
		return make_float4(1, 0, 1, 0);
	}

	// This works only single threaded
	if (!IntersectScene<true>(r, hitInfo))
	{
		return make_float4(0);
	}

	const float3 I = make_float3(r.Evaluate(hitInfo.triIntercept.t));

	const CoreTri &triangle = meshes[hitInfo.meshId].triangles[hitInfo.triId];
	const CoreMaterial &material = scene.matList[triangle.material];

	// Note that this could be a texture map of normals in material
	const float3& N 
		= normalize(triangle.vN0 * hitInfo.triIntercept.u + triangle.vN1 * hitInfo.triIntercept.v + triangle.vN2 * hitInfo.triIntercept.GetWCoord())
			* (hitInfo.triIntercept.backFacing ? -1.0f : 1.0f);

	float diffuse, reflection, refraction;
	getLightComponents(material, diffuse, reflection, refraction);

	float3 color = make_float3(0);
	if (diffuse > kEps)
	{
		color += diffuse * Illuminate(I, N, -1, hitInfo.meshId, hitInfo.triId);
	}

	if (reflection > kEps)
	{
		r.SetOrigin(I);
		float3 D = make_float3(r.direction);
		r.SetDirection(D - 2.f * (dot(D, N) * N));
		color += make_float3(Trace(r, currentDepth + 1)) * reflection;
	}

	// Note that N should could be flipped if this is glass
	return make_float4(color * material.color.value);
}


float3 RenderCore::Illuminate(const float3 &p, const float3 &N, int instanceId, int meshId, int triId) const
{
	// TODO spot lights
	// TODO Check if back culling is okay
	float3 intensity = make_float3(0);
	for (int i = 0; i < scene.directionalLights.size(); i++)
	{
		// Note that we don't care about back facing directional lights. Glass doesn't care about diffuse
		intensity += clamp(dot(N, (-1.0f) * scene.directionalLights[i].direction), 0.0f, 1.0f) * scene.directionalLights[i].radiance;
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
		dotVal = dot(N, lightDir);

		if (dotVal < kEps)
		{
			// The light doesn't matter
			// TODO check if this is okay for glass
			continue;
		}


		// Occlusion Tests (do we have to test back facin triangles?)
		// Note this works only single threaded
		lightRay.SetOrigin(light.position);
		lightRay.SetDirection(lightDir * -1.0f);
		if (TestDepthScene<true>(lightRay, instanceId, meshId, triId, dist))
		{
			// Occluded
			continue;
		}


		// Note that we don't care about back facing directional lights. Glass doesn't care about diffuse
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

		// TODO Remember backfacing surfaces 
		dist = sqrt(distSqr);
		lightDir = pToLight / dist;
		dotVal = dot(N, lightDir);
		if (dotVal < kEps)
		{
			// The light doesn't matter
			// TODO check if this is okay for glass
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
		if (TestDepthScene<true>(lightRay, instanceId, meshId, triId, dist))
		{
			// Occluded
			continue;
		}

		// Note that we don't care about back facing directional lights. Glass doesn't care about diffuse
		intensity += clamp(dotVal, 0.0f, 1.0f) * light.radiance * invDistSqr * cosFalloff;
	}

	return intensity;
}

// EOF