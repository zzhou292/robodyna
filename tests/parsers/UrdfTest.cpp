#include "chrono_parsers/urdf/ChParserURDF.h"
#include "chrono/physics/ChSystemNSC.h"
#include "gtest/gtest.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

TEST(UrdfParser, RealSdkBuildsBodiesJointAndOneNativeSystem) {
    const char* temporary = std::getenv("TEST_TMPDIR");
    ASSERT_NE(temporary, nullptr);
    const auto file = std::filesystem::path(temporary) / "robot.urdf";
    ASSERT_FALSE(std::filesystem::exists(file));
    {
        std::ofstream output(file);
        output << R"(<robot name="parser_admission">
<link name="world"/>
<link name="payload"><inertial><mass value="2"/>
  <origin xyz="0 0 0"/><inertia ixx="1" iyy="2" izz="3" ixy="0" ixz="0" iyz="0"/>
</inertial></link>
<joint name="attachment" type="fixed"><parent link="world"/><child link="payload"/>
  <origin xyz="1 2 3" rpy="0 0 0"/>
</joint></robot>)";
        ASSERT_TRUE(output.good());
    }
    chrono::parsers::ChParserURDF parser(file.string());
    EXPECT_EQ(parser.GetModelName(), "parser_admission");
    chrono::ChSystemNSC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, 0});
    parser.PopulateSystem(system);
    ASSERT_EQ(system.GetBodies().size(), 2U);
    EXPECT_EQ(&parser.GetChSystem(), &system);
    EXPECT_TRUE(parser.GetRootChBody()->IsFixed());
    const auto body = parser.GetChBody("payload");
    ASSERT_TRUE(body);
    EXPECT_DOUBLE_EQ(body->GetMass(), 2);
    EXPECT_DOUBLE_EQ(body->GetInertiaXX().y(), 2);
    EXPECT_NEAR(body->GetPos().x(), 1, 1e-14);
    EXPECT_NEAR(body->GetPos().y(), 2, 1e-14);
    EXPECT_NEAR(body->GetPos().z(), 3, 1e-14);
    ASSERT_TRUE(parser.GetChLink("attachment"));
    ASSERT_TRUE(system.DoStepDynamics(0.001));
    EXPECT_NEAR(system.GetChTime(), 0.001, 1e-15);
    EXPECT_NEAR(body->GetPos().z(), 3, 1e-12);
}
