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
	// initialize core
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
	ray.SetOrigin(view.pos);

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


float4 RenderCore::Trace(const Ray& r) const
{
	// This works only single threaded
	if (!IntersectMeshes<true>(r, hitInfo))
	{
		return make_float4(0);
	}

	// Just for testing
	float depth = 1.0f - clamp(hitInfo.triIntercept.t / 100.0f, 0.0f, 1.0f);
	return make_float4(depth, depth, depth, 1.0f);
}

// EOF