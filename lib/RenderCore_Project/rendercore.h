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
	void SetTextures(const CoreTexDesc* tex, const int textureCount);
	void Setting(const char* name, float value);

	void WaitForRender() { /* this core does not support asynchronous rendering yet */ }
	CoreStats GetCoreStats() const override;
	void Shutdown();

	// unimplemented for the minimal core
	inline void SetProbePos( const int2 pos ) override {}
	inline void SetSkyData( const float3* pixels, const uint width, const uint height, const mat4& worldToLight ) override {}
	inline void SetInstance( const int instanceIdx, const int modelIdx, const mat4& transform ) override {}
	inline void FinalizeInstances() override {}

	using Renderable = std::function<void(uint, uint, uint)>;

	// internal methods
private:

	enum class RenderType : int {
		Whitted = 0,
		PathTracer,
		Count
	};

	RenderType renderType;

	// data members
	Bitmap* screen = 0;								// temporary storage of RenderCore output; will be copied to render target
	float4* fscreen = 0;							// HDR temp storage
	int targetTextureID = 0;						// ID of the target OpenGL texture
	Scene scene;									// color and texture data storage 
	int maximumDepth = 10;

	float4* accumulationBuffer = 0;					// History storage
	float historyMix;								// History mix value

	float* kernel;
	bool useVignetting;
	float sigma;
	bool useToneMapping;

	float elapsedTime = 0;

	thread_pool pool = 4;
	vector<future<void>> futures;


	/// <summary>
	/// Scene exposure. See https://blog.demofox.org/2020/06/06/casual-shadertoy-path-tracing-2-image-improvement-and-glossy-reflections/
	/// </summary>
	float exposure;

	template <bool backCulling>
	float4 Trace(Ray &r, const float3 &intensity, int matId = -1, int currentDepth = 0) const;

	void RenderCore::CreateGaussianKernel(uint width, uint height);

	float4 LoadMaterialFloat4(const CoreMaterial::Vec3Value& val, const float2& uv) const;

	template <bool backCulling>
	float4 Sample(Ray& r, const float3& intensity, int matId = -1, int currentDepth = 0) const;
	float3 DiffuseReflection(const float3& N, const CoreTri& triangle) const;

	/// <summary>
	/// Mix of indirect and direct lighting
	/// </summary>
	float indirectDirectMix;

	/// <summary>
	/// BVH Type used
	/// </summary>
	BVH_Type bvhType;

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
		return interceptRayScene<backCulling>(v, scene.meshBVH, hit);
	}

	template <bool backCulling>
	[[nodiscard]]
	inline bool TestDepthScene(const Ray& v, const int instId, const int meshId, const int triId, const float tD) const
	{
		return depthRayScene<backCulling>(v, scene.meshBVH, instId, meshId, triId, tD);
	}

	template <bool backCulling>
	float3 Illuminate(const float3& p, const float3& N, int instanceId, int meshId, int triID) const;

	template <bool backCulling>
	float3 DirectLighting(const float3& p, const float3& N, int instanceId, int meshId, int triID) const;

	void RenderInternal(Renderable R);

	enum class DistortionType : int {
		None = 0,
		Barrel,
		BarrelSpecial,
		FishEye,
		Count
	};

	/// <summary>
	/// Current Distortion Type
	/// </summary>
	DistortionType distortionType;

	/// <summary>
	/// Anti Aliasing Level
	/// </summary>
	int aaLevel;

	/// <summary>
	/// Inverse of Anti Aliasing Level
	/// </summary>
	float invAaLevel;

	/// <summary>
	/// Current Offset used
	/// </summary>
	int offsetIter = 0;

	/// <summary>
	/// Number of Pixel Offsets for anti-aliasing
	/// </summary>
	static constexpr int pixelOffsetsSize = 65;

	/// <summary>
	/// Pixel Offsets, the first one is always the center
	/// </summary>
	static float pixelOffSets[pixelOffsetsSize * 2];

	/// <summary>
	/// For Gamma Correction
	/// </summary>
	float gamma;

	/// <summary>
	/// Gamma Correction
	/// </summary>
	float gammaCorrection;

	/// <summary>
	/// Chromatic Aberration Scaling Constant
	/// </summary>
	float chromaticAberrationScale;

	/// <summary>
	/// If we are using chromatic aberration
	/// </summary>
	bool useChromaticAberration;

	/// <summary>
	/// Chromatic Aberration Radial Constant
	/// </summary>
	float3 aberrationRadialK;

	/// <summary>
	/// Chromatic Aberration U Focal Point Offset
	/// </summary>
	float3 aberrationUOffset;

	/// <summary>
	/// Chromatic Aberration Y Focal Point Offset
	/// </summary>
	float3 aberrationVOffset;

	/// <summary>
	/// Number of Tiles on the X axis
	/// </summary>
	uint packetXTileNumber = 0;

	/// <summary>
	/// Number of Tiles on the Y axis
	/// </summary>
	uint packetYTileNumber = 0;

public:
	CoreStats coreStats;							// rendering statistics

	/// <summary>
	/// Packet Tile size on X
	/// </summary>
	constexpr static uint kPacketXTileSize = 8;

	/// <summary>
	/// Packet Tile size on Y
	/// </summary>
	constexpr static uint kPacketYTileSize = 8;
};

} // namespace lh2core

#include "rendercore.impl.h"

// EOF