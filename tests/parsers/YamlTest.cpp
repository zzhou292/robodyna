#include "chrono_parsers/yaml/ChParserMbsYAML.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/input_output/ChUtilsYAML.h"
#include "gtest/gtest.h"

#include <cmath>

#ifndef CHRONO_HAS_YAML
#error "YAML admission must use the coherent owning configuration"
#endif

TEST(YamlParser, CoreSettingsDefinitionsAndVectorConversionAreLinked) {
    const auto root = YAML::Load(R"(
visualization:
  render_fps: 25
  camera:
    vertical: Z
    location: [1, 2, 3]
    target: [0, 0, 0]
output:
  format: ASCII
  mode: SERIES
  fps: 40
)");
    const auto visual = chrono::ChVisualSystem::Settings::Read(root["visualization"]);
    EXPECT_DOUBLE_EQ(visual.render_fps, 25);
    EXPECT_EQ(visual.camera_vertical, chrono::CameraVerticalDir::Z);
    EXPECT_DOUBLE_EQ(visual.camera_location.z(), 3);
    const auto output = chrono::ChOutput::Settings::Read(root["output"]);
    EXPECT_EQ(output.format, chrono::ChOutput::Format::ASCII);
    EXPECT_EQ(output.mode, chrono::ChOutput::Mode::SERIES);
    EXPECT_DOUBLE_EQ(output.fps, 40);
}

TEST(YamlParser, ParsedBodyUsesOneNativeClockAndExpectedFreeFall) {
    chrono::parsers::ChParserMbsYAML parser;
    parser.LoadSimData(YAML::Load(R"(
simulation:
  gravity: [0, 0, -9.81]
  end_time: 0.1
  enforce_realtime: false
  num_threads: {chrono: 1, collision: 1, eigen: 1}
)"));
    parser.LoadSolverData(YAML::Load(R"(
contact_method: SMC
integrator: {type: Euler_implicit_linearized, time_step: 0.001}
solver: {type: MINRES, max_iterations: 100, tolerance: 1.e-12}
)"));
    parser.LoadModelData(YAML::Load(R"(
model:
  name: parser_admission
  bodies:
    - name: falling
      location: [0, 0, 2]
      mass: 2
      inertia: {moments: [1, 1, 1]}
)"));
    ASSERT_TRUE(parser.HasModelData());
    ASSERT_TRUE(parser.HasSolverData());
    auto system = parser.CreateSystem();
    ASSERT_TRUE(system);
    EXPECT_EQ(parser.Populate(*system), 0);
    const auto body = parser.FindBodyByName("falling");
    ASSERT_TRUE(body);
    EXPECT_EQ(body->GetSystem(), system.get());
    EXPECT_DOUBLE_EQ(body->GetMass(), 2);
    EXPECT_DOUBLE_EQ(parser.GetTimestep(), 0.001);
    for (int i = 0; i < 100; ++i) parser.DoStepDynamics();
    EXPECT_NEAR(system->GetChTime(), 0.1, 1e-13);
    EXPECT_NEAR(body->GetPosDt().z(), -0.981, 1e-10);
    // The retained semi-implicit Euler update uses the new velocity each step.
    EXPECT_NEAR(body->GetPos().z(), 2 - 9.81 * 0.001 * 0.001 * 100 * 101 / 2, 1e-10);
    EXPECT_TRUE(std::isfinite(body->GetPos().z()));
    EXPECT_EQ(system->GetVisualSystem(), nullptr);
}
