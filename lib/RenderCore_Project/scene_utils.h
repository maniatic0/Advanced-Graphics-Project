#pragma once

namespace lh2core
{
	// -----------------------------------------------------------
	// Scene class
	// owner of the scene graph;
	// owner of the material and texture list
	// -----------------------------------------------------------
	class Scene {
	public:
		// constructor / destructor
		Scene() = default;
		// data members
		vector<CoreMaterial> matList;
		CoreMaterial air;

		vector<CoreTexDesc> texList;

		// Lights
		vector<CorePointLight>			pointLights;
		vector<CoreSpotLight>			spotLights;
		vector<CoreDirectionalLight>	directionalLights;
		vector<CoreLightTri>			areaLights;

		// BVH
		vector<BVH> meshBVH;

	};

}