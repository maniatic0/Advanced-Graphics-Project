/* core_settings.h - Copyright 2019/2020 Utrecht University

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

	   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

   The settings and classes in this file are core-specific:
   - avilable in host and device code
   - specific to this particular core.
   Global settings can be configured shared.h.
*/

#pragma once

#include "platform.h"

using namespace lighthouse2;

#include "core_api_base.h"
#include "general_utils.h"
#include "intersection_utils.h"

// #define MEASURE_BVH // To measure BVHs

#include "bvh2_utils.h"
#include "bvh4_utils.h"
#include "bvh_utils.h"
#include "threadpool.h"

#include "scene_utils.h"
#include "rendercore.h"

using namespace lh2core;

// EOF