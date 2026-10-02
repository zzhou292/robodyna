#include "chrono_sensor/ChConfigSensor.h"
#include "chrono_sensor/filters/ChFilterTachometerUpdate.h"
#include "chrono_sensor/sensors/ChTachometerSensor.h"
#include "chrono_sensor/utils/ChGPSUtils.h"
#include "chrono/utils/ChConstants.h"
#include "robodyna/mbd/RbBody.h"
#include "robodyna/simulation/RbSystemNSC.h"

#include <gtest/gtest.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#if !defined(CHRONO_HAS_VULKAN_RT) || !defined(CHRONO_HAS_SENSOR_RENDER) || !defined(USE_SENSOR_GLFW)
#error "This gate requires the declared Vulkan rendered-sensor and real windowing profile"
#endif
#if defined(CHRONO_HAS_OPTIX) || defined(CHRONO_HAS_METAL_RT) || defined(CHRONO_SENSOR_VULKAN_RT_GPU_ENABLED)
#error "Unexpected alternate backend or leaked private renderer definition"
#endif

TEST(SensorHost, OriginalGpsConversionRoundTripsTheDeclaredReference) {
    chrono::ChVector3d reference(-122, 37, 15);
    chrono::ChVector3d zero(0, 0, 0);
    chrono::sensor::Cartesian2GPS(zero, reference);
    EXPECT_EQ(zero, reference);
    const chrono::ChVector3d original(123, -456, 7);
    auto position = original;
    chrono::sensor::Cartesian2GPS(position, reference);
    chrono::sensor::GPS2Cartesian(position, reference);
    EXPECT_NEAR((position - original).Length(), 0, 2e-8);
}

TEST(SensorHost, TachometerReadsTheSameBodyWithoutAdvancingItsSystem) {
    using namespace chrono::sensor;
    robodyna::simulation::RbSystemNSC system;
    auto body = chrono_types::make_shared<robodyna::mbd::RbBody>();
    system.AddBody(body);
    body->SetAngVelLocal(chrono::ChVector3d(0, 0, 6 * chrono::CH_PI));
    system.SetTime(2.5);
    auto sensor = chrono_types::make_shared<ChTachometerSensor>(body, 100, chrono::ChFramed(), ChTachometerSensor::Axis::Z);
    ChFilterTachometerUpdate updater;
    std::shared_ptr<SensorBuffer> output;
    updater.Initialize(sensor, output);
    updater.Apply();
    auto rpm = std::dynamic_pointer_cast<SensorHostTachometerBuffer>(output);
    ASSERT_NE(rpm, nullptr);
    ASSERT_EQ(rpm->Width, 1);
    ASSERT_EQ(rpm->Height, 1);
    EXPECT_FLOAT_EQ(rpm->Buffer[0].rpm, 180.f);
    EXPECT_EQ(sensor->GetParent().get(), body.get());
    EXPECT_EQ(body->GetSystem(), &system);
    EXPECT_DOUBLE_EQ(system.GetTime(), 2.5);
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
}

TEST(SensorHost, PinnedWindowingLibrariesLinkWithoutOpeningAWindow) {
    int major = 0;
    int minor = 0;
    int revision = 0;
    glfwGetVersion(&major, &minor, &revision);
    EXPECT_EQ(major, 3);
    EXPECT_EQ(minor, 3);
    EXPECT_EQ(revision, 6);
    EXPECT_STREQ(reinterpret_cast<const char*>(glewGetString(GLEW_VERSION)), "2.2.0");
}
