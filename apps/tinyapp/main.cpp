/* main.cpp - Copyright 2019/2020 Utrecht University

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

#include "platform.h"
#include "rendersystem.h"

#include <bitset>

static RenderAPI* renderer = 0;
static GLTexture* renderTarget = 0;
static Shader* shader = 0;
static uint scrwidth = 0, scrheight = 0, car = 0, scrspp = 1;
static bool running = true;
static std::bitset<1024> keystates;

#include "main_tools.h"

//  +-----------------------------------------------------------------------------+
//  |  PrepareScene                                                               |
//  |  Initialize a scene.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void PrepareScene()
{
	// initialize scene
	//renderer->AddScene( "scene.gltf", "../_shareddata/pica/" );
	//renderer->SetNodeTransform( renderer->FindNode( "RootNode (gltf orientation matrix)" ), mat4::RotateX( -PI / 2 ) );
	//int lightMat = renderer->AddMaterial( make_float3( 100, 100, 80 ) );
	//int lightQuad = renderer->AddQuad( make_float3( 0, -1, 0 ), make_float3( 0, 26.0f, 0 ), 6.9f, 6.9f, lightMat );
	//renderer->AddInstance( lightQuad );
	//car = renderer->AddInstance( renderer->AddMesh( "basic_box.obj", "../_shareddata/", 10.0f ) );

	//int lightMat = renderer->AddMaterial(make_float3(1, 1, 1));
	//HostMaterial &lightMatHost = *renderer->GetMaterial(lightMat);
	//lightMatHost.reflection.value = 0.4f; // glass
	//lightMatHost.refraction.value = 0.5f; // glass
	//lightMatHost.ior.value = 1.5f; // glass
	//lightMatHost.pbrtMaterialType = MaterialType::PBRT_GLASS;
	//int lightQuad = renderer->AddQuad( make_float3( 1, 1, 0 ), make_float3( 0, 0.1, 0 ), 10.0f, 10.0f, lightMat );
	//renderer->AddInstance( lightQuad );
	//int lightQuad_2 = renderer->AddQuad(make_float3(-1, -1, 0), make_float3(0, 0, 0), 10.0f, 10.0f, lightMat);
	//renderer->AddInstance(lightQuad_2);


	//int lightMat2 = renderer->AddMaterial(make_float3(1, 0, 0));
	//HostMaterial& lightMat2Host = *renderer->GetMaterial(lightMat2);
	//lightMat2Host.reflection.value = 0.4f; // glass
	//lightMat2Host.refraction.value = 0.5f; // glass
	//lightMat2Host.ior.value = 1.5f; // glass
	//lightMat2Host.pbrtMaterialType = MaterialType::PBRT_GLASS;
	//int lightQuad2 = renderer->AddQuad(make_float3(1, 1, 0), make_float3(0, 0.1, 20), 10.0f, 10.0f, lightMat2);
	//renderer->AddInstance(lightQuad2);
	//int lightQuad2_2 = renderer->AddQuad(make_float3(-1, -1, 0), make_float3(0, 0, 20), 10.0f, 10.0f, lightMat2);
	//renderer->AddInstance(lightQuad2_2);

	//int lightMat3 = renderer->AddMaterial(make_float3(0, 1, 0));
	//HostMaterial& lightMat3Host = *renderer->GetMaterial(lightMat3);
	//lightMat3Host.reflection.value = 0.4f; // glass
	//lightMat3Host.refraction.value = 0.5f; // glass
	//lightMat3Host.ior.value = 1.5f; // glass
	//lightMat3Host.pbrtMaterialType = MaterialType::PBRT_GLASS;
	//int lightQuad3 = renderer->AddQuad(make_float3(1, 1, 0), make_float3(0, 0.1, -20), 10.0f, 10.0f, lightMat3);
	//renderer->AddInstance(lightQuad3);
	//int lightQuad3_2 = renderer->AddQuad(make_float3(-1, -1, 0), make_float3(0, 0, -20), 10.0f, 10.0f, lightMat3);
	//renderer->AddInstance(lightQuad3_2);

	//int lightMat4 = renderer->AddMaterial(make_float3(1, 0.1, 0.1));
	//HostMaterial& lightMat4Host = *renderer->GetMaterial(lightMat4);
	//lightMat4Host.reflection.value = 0.4f; // glass
	//lightMat4Host.refraction.value = 0.5f; // glass
	//lightMat4Host.ior.value = 1.5f; // glass
	//lightMat4Host.pbrtMaterialType = MaterialType::PBRT_GLASS;
	//int lightQuad4 = renderer->AddQuad(make_float3(1, 1, 0), make_float3(-5, -5.0, 5), 10.0f, 10.0f, lightMat4);
	//renderer->AddInstance(lightQuad4);
	//int lightQuad4_2 = renderer->AddQuad(make_float3(-1, -1, 0), make_float3(-5, -5.1, 5), 10.0f, 10.0f, lightMat4);
	//renderer->AddInstance(lightQuad4_2);

	//int lightMat5 = renderer->AddMaterial(make_float3(1, 1, 1));
	//HostMaterial& lightMatHost5 = *renderer->GetMaterial(lightMat5);
	//lightMatHost5.reflection.value = 0.5f; // Metal
	//lightMatHost5.pbrtMaterialType = MaterialType::PBRT_METAL;

	//int lightQuad5 = renderer->AddQuad(make_float3(1, 1, 0), make_float3(-2, -20, 0), 200.0f, 200.0f, lightMat5);
	//renderer->AddInstance(lightQuad5);

	//int lightMat5 = renderer->AddMaterial(make_float3(1, 0.1, 0.1));
	//HostMaterial& lightMatHost5 = *renderer->GetMaterial(lightMat5);
	//lightMatHost5.reflection.value = 0.0f; // Glass
	//lightMatHost5.refraction.value = 0.9f; // Glass
	//lightMatHost5.ior.value = 1.5f; // Glass
	//lightMatHost5.pbrtMaterialType = MaterialType::PBRT_GLASS;

	//int lightQuad5 = renderer->AddQuad(make_float3(1, 1, 0), make_float3(0, 30, 0), 100.0f, 100.0f, lightMat5);
	//renderer->AddInstance(lightQuad5);
	//int lightQuad5_2 = renderer->AddQuad(make_float3(-1,- 1, 0), make_float3(0, 29.5f, 0), 100.0f, 100.0f, lightMat5);
	//renderer->AddInstance(lightQuad5_2);

	//// Test 1 light
	//renderer->AddDirectionalLight(make_float3(0, -1, 0), make_float3(0.2, 0.2, 0.2));
	//renderer->AddPointLight(make_float3(-5, -5, 0), make_float3(10 * 10, 10 * 10, 10 * 10));
	////renderer->AddPointLight(make_float3(40, 40, -10), make_float3(30 * 30, 30 * 30, 30 * 30));
	
	//renderer->AddSpotLight(make_float3(20, 40, 0), normalize(make_float3(-1, -1, 0)), cos(10.0f * PI / 180.0f), cos(70.0f * PI / 180.0f), make_float3(50 * 50, 50 * 50, 50 * 50));

	//int wallMat = renderer->AddMaterial(make_float3(0.2, 0.2, 0.7));

	//int wallQuad1 = renderer->AddQuad(make_float3(1, 0, 0), make_float3(-50, 0, 0), 300.0f, 300.0f, wallMat);
	//renderer->AddInstance(wallQuad1);
	//int wallQuad2 = renderer->AddQuad(make_float3(0, 0, 1), make_float3(0, 0, -40), 200.0f, 200.0f, wallMat);
	//renderer->AddInstance(wallQuad2);
	//int wallQuad3 = renderer->AddQuad(make_float3(0, 0, -1), make_float3(0, 0, 40), 200.0f, 200.0f, wallMat);
	//renderer->AddInstance(wallQuad3);

	//int cubeMat = renderer->AddMaterial(make_float3(0.2, 0.7, 0.2));
	//HostMaterial& cubeMatHost = *renderer->GetMaterial(cubeMat);
	//cubeMatHost.reflection.value = 0.0f; // Glass
	//cubeMatHost.refraction.value = 0.9f; // Glass
	//cubeMatHost.ior.value = 1.5f; // Glass
	//cubeMatHost.pbrtMaterialType = MaterialType::PBRT_GLASS;

	//int cubeQuad1 = renderer->AddQuad(make_float3(1, 0, 0), make_float3(0, 0, 20), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad1);
	//int cubeQuad2 = renderer->AddQuad(make_float3(0, 0, -1), make_float3(0, 0, 15), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad2);
	//int cubeQuad3 = renderer->AddQuad(make_float3(0, 0, 1), make_float3(0, 0, 25), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad3);
	//int cubeQuad4 = renderer->AddQuad(make_float3(1, 0, 0), make_float3(-10, 0, 20), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad4);
	//int cubeQuad5 = renderer->AddQuad(make_float3(0, 1, 0), make_float3(0, 5, 20), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad5);
	//int cubeQuad6 = renderer->AddQuad(make_float3(0, -1, 0), make_float3(0, -5, 20), 10.0f, 10.0f, cubeMat);
	//renderer->AddInstance(cubeQuad6);

	// initialize scene
	int boxScene = renderer->AddMesh("../_shareddata/basic_box.obj", 0.2f);
	renderer->AddInstance(boxScene);
	renderer->AddPointLight(make_float3(0, 20, 0), 50 * make_float3(10, 10, 10));

	/*
	HostMaterial& cubeMatHost = *renderer->GetMaterial(2);
	cubeMatHost.ior.value = 1.5f; // Glass
	cubeMatHost.absorption.value = make_float3(0.f);
	cubeMatHost.pbrtMaterialType = MaterialType::PBRT_GLASS;*/
	
	int texId = renderer->AddTexture("../_shareddata/textures/LEGOSHLD.tga", 0);
	//int texId = renderer->AddTexture("../_shareddata/textures/checker.png", 0);

	int lightMat = renderer->AddMaterial(make_float3(1, 1, 1));
	HostMaterial &lightMatHost = *renderer->GetMaterial(lightMat);
	lightMatHost.color.textureID = texId;
	//lightMatHost.ior.value = 1.5f; // glass
	//lightMatHost.absorption.value = make_float3(0.05f);
	//lightMatHost.pbrtMaterialType = MaterialType::PBRT_GLASS;
	int lightQuad = renderer->AddQuad( make_float3( 1, -1, 0 ), make_float3( 0, 10.1, 0 ), 10.0f, 10.0f, lightMat );
	renderer->AddInstance( lightQuad );
	int lightQuad_2 = renderer->AddQuad(make_float3(-1, 1, 0), make_float3(0, 10, 0), 10.0f, 10.0f, lightMat);
	renderer->AddInstance(lightQuad_2);

	//renderer->AddPointLight(make_float3(0, 0, 0), 50 * make_float3(10, 10, 10));

}

//  +-----------------------------------------------------------------------------+
//  |  HandleInput                                                                |
//  |  Process user input.                                                  LH2'19|
//  +-----------------------------------------------------------------------------+
void HandleInput( float frameTime )
{
	// handle keyboard input
	float spd = (keystates[GLFW_KEY_LEFT_SHIFT] ? 15.0f : 5.0f) * frameTime, rot = 2.5f * frameTime;
	Camera* camera = renderer->GetCamera();
	if (keystates[GLFW_KEY_A]) camera->TranslateRelative( make_float3( -spd, 0, 0 ) );
	if (keystates[GLFW_KEY_D]) camera->TranslateRelative( make_float3( spd, 0, 0 ) );
	if (keystates[GLFW_KEY_W]) camera->TranslateRelative( make_float3( 0, 0, spd ) );
	if (keystates[GLFW_KEY_S]) camera->TranslateRelative( make_float3( 0, 0, -spd ) );
	if (keystates[GLFW_KEY_R]) camera->TranslateRelative( make_float3( 0, spd, 0 ) );
	if (keystates[GLFW_KEY_F]) camera->TranslateRelative( make_float3( 0, -spd, 0 ) );
	if (keystates[GLFW_KEY_UP]) camera->TranslateTarget( make_float3( 0, -rot, 0 ) );
	if (keystates[GLFW_KEY_DOWN]) camera->TranslateTarget( make_float3( 0, rot, 0 ) );
	if (keystates[GLFW_KEY_LEFT]) camera->TranslateTarget( make_float3( -rot, 0, 0 ) );
	if (keystates[GLFW_KEY_RIGHT]) camera->TranslateTarget( make_float3( rot, 0, 0 ) );
}

//  +-----------------------------------------------------------------------------+
//  |  main                                                                       |
//  |  Application entry point.                                             LH2'19|
//  +-----------------------------------------------------------------------------+
int main()
{
	// initialize OpenGL
	InitGLFW();

	// initialize renderer: pick one
	renderer = RenderAPI::CreateRenderAPI("RenderCore_Project");		// Project
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Optix7filter" );			// OPTIX7 core, with filtering (static scenes only for now)
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Optix7" );			// OPTIX7 core, best for RTX devices
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_OptixPrime_B" );		// OPTIX PRIME, best for pre-RTX CUDA devices
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_SoftRasterizer" );	// RASTERIZER, your only option if not on NVidia
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_Vulkan_RT" );			// Meir's Vulkan / RTX core
	// renderer = RenderAPI::CreateRenderAPI( "RenderCore_OptixPrime_BDPT" );	// Peter's OptixPrime / BDPT core

	renderer->DeserializeExtraSettings("extra_settings.xml");
	renderer->DeserializeCamera( "camera.xml" );
	// initialize scene
	PrepareScene();
	// set initial window size
	ReshapeWindowCallback( 0, SCRWIDTH, SCRHEIGHT );
	// enter main loop
	while (!glfwWindowShouldClose( window ))
	{
		// update scene
		renderer->SynchronizeSceneData();
		// render
		renderer->Render( Restart /* alternative: converge */ );
		// handle user input
		HandleInput( 0.025f );
		// minimal rigid animation example
		static float r = 0;
		//renderer->SetNodeTransform( car, mat4::RotateY( r * 2.0f ) * mat4::RotateZ( 0.2f * sinf( r * 8.0f ) ) * mat4::Translate( make_float3( 0, 5, 0 ) ) );
		r += 0.025f * 0.3f; if (r > 2 * PI) r -= 2 * PI;
		// finalize and present
		shader->Bind();
		shader->SetInputTexture( 0, "color", renderTarget );
		shader->SetInputMatrix( "view", mat4::Identity() );
		DrawQuad();
		shader->Unbind();
		// finalize
		glfwSwapBuffers( window );
		glfwPollEvents();
		if (!running) break; // esc was pressed
	}
	// clean up
	renderer->SerializeCamera( "camera.xml" );
	renderer->Shutdown();
	glfwDestroyWindow( window );
	glfwTerminate();
	return 0;
}

// EOF