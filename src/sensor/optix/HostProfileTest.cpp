#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/optix/ChOptixScene.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChOptixSensor.h"

#include <gtest/gtest.h>
#include <cmath>
#include <type_traits>

#if !defined(CHRONO_HAS_OPTIX) || !defined(CHRONO_HAS_SENSOR_RENDER) || !defined(CHRONO_USE_CUDA)
#error "This gate needs the actual OptiX/CUDA Sensor profile"
#endif
#if defined(CHRONO_HAS_VULKAN_RT) || defined(CHRONO_HAS_METAL_RT) || defined(USE_CUDA_NVRTC)
#error "Mixed backend or private NVRTC definition leaked into the consumer"
#endif

static_assert(OPTIX_VERSION == 90100);
static_assert(std::is_base_of_v<chrono::sensor::ChOptixSensor, chrono::sensor::ChCameraSensor>);

TEST(OptixHostProfile, OriginalSceneFogUsesTheRequestedScatteringDistance) {
    chrono::sensor::ChOptixScene scene;
    EXPECT_FLOAT_EQ(scene.GetFogScattering(), 0.f);
    scene.SetFogScatteringFromDistance(200.f);
    EXPECT_FLOAT_EQ(scene.GetFogScattering(), static_cast<float>(std::log(256.0) / 200.0));
    scene.SetFogScattering(2.f);
    EXPECT_FLOAT_EQ(scene.GetFogScattering(), 1.f);
    scene.SetFogScattering(-1.f);
    EXPECT_FLOAT_EQ(scene.GetFogScattering(), 0.f);
}

#ifdef CHRONO_FSI_SPH
static_assert(std::is_member_function_pointer_v<decltype(&chrono::sensor::ChSensorManager::AttachFsiSphSystem)>);
static_assert(std::is_member_function_pointer_v<decltype(&chrono::sensor::ChOptixScene::AddFsiSphSystem)>);

TEST(OptixHostProfile, SphEnabledSceneContainsTheRealRegistrationInterface) {
    chrono::sensor::ChOptixScene scene;
    EXPECT_TRUE(scene.GetFsiSphSources().empty());
    // No fake fluid object or graphics device is constructed by this host gate.
    // A real attachment/render test requires the separately guarded GPU runtime.
}
#endif
