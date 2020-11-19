/* rendercore.h - Copyright 2019/2020 Utrecht University

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

#pragma once

namespace lh2core
{

//  +-----------------------------------------------------------------------------+
//  |  RenderCore                                                                 |
//  |  Encapsulates device code.                                            LH2'19|
//  +-----------------------------------------------------------------------------+
class RenderCore : public CoreAPI_Base
{
public:
	// methods
	void Init();
	void SetTarget( GLTexture* target, const uint spp );
	void SetGeometry( const int meshIdx, const float4* vertexData, const int vertexCount, const int triangleCount, const CoreTri* triangles );
	void Render( const ViewPyramid& view, const Convergence converge, bool async );
	void SetMaterials(CoreMaterial* mat, const int materialCount);
	void SetLights(const CoreLightTri* triLights, const int triLightCount,
		const CorePointLight* pointLights, const int pointLightCount,
		const CoreSpotLight* spotLights, const int spotLightCount,
		const CoreDirectionalLight* directionalLights, const int directionalLightCount);

	void WaitForRender() { /* this core does not support asynchronous rendering yet */ }
	CoreStats GetCoreStats() const override;
	void Shutdown();

	// unimplemented for the minimal core
	inline void SetProbePos( const int2 pos ) override {}
	inline void Setting( const char* name, float value ) override {}
	inline void SetTextures( const CoreTexDesc* tex, const int textureCount ) override {}
	inline void SetSkyData( const float3* pixels, const uint width, const uint height, const mat4& worldToLight ) override {}
	inline void SetInstance( const int instanceIdx, const int modelIdx, const mat4& transform ) override {}
	inline void FinalizeInstances() override {}

	// internal methods
private:

	// data members
	Bitmap* screen = 0;								// temporary storage of RenderCore output; will be copied to render target
	float4* fscreen = 0;							// HDR temp storage
	int targetTextureID = 0;						// ID of the target OpenGL texture
	vector<Mesh> meshes;							// mesh data storage
	Scene scene;									// color and texture data storage 
	int maximumDepth = 10;

	uint yScanline;

	template <bool backCulling>
	float4 Trace(Ray &r, int currentDepth = 0) const;
	float3 Refract(const float3* I, const float3* N, const float& ior) const;

	/// <summary>
	/// Note this trick only works single threaded. We would need threadlocal stuff
	/// </summary>
	mutable RayMeshInterceptInfo hitInfo;

	/// <summary>
	/// Note this trick only works single threaded. We would need threadlocal stuff
	/// </summary>
	mutable Ray lightRay;

	template <bool backCulling>
	[[nodiscard]]
	inline bool IntersectScene(const Ray& v, RayMeshInterceptInfo& hit) const
	{
		return interceptRayScene<backCulling>(v, meshes, hit);
	}

	template <bool backCulling>
	[[nodiscard]]
	inline bool TestDepthScene(const Ray& v, const int instId, const int meshId, const int triId, const float tD) const
	{
		return depthRayScene<backCulling>(v, meshes, instId, meshId, triId, tD);
	}

	template <bool backCulling>
	float3 Illuminate(const float3& p, const float3& N, int instanceId, int meshId, int triID) const;

public:
	CoreStats coreStats;							// rendering statistics
};

} // namespace lh2core

#include "rendercore.impl.h"

// EOF