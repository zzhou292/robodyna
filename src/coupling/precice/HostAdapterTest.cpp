// Admission of the real SDK and retained MBS adapter; no coupled trajectory claim.
#include "chrono_precice/ChPreciceAdapterMbs.h"
#include "robodyna/simulation/RbSystemSMC.h"
#include "precice/Tooling.hpp"
#include "chrono_thirdparty/rapidjson/document.h"

#include <gtest/gtest.h>
#include <cmath>
#include <dlfcn.h>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {
std::string participant_yaml;
std::string test_configuration;
std::string sdk_receipt;
}

TEST(PreciceHost, ActualSdkVersionAndConfigurationValidation) {
    EXPECT_EQ(std::string(PRECICE_VERSION), "3.0.0");
    const auto loaded = precice::getVersionInformation();
    EXPECT_NE(loaded.find("3.0.0"), std::string::npos);
    std::ifstream input(sdk_receipt);
    ASSERT_TRUE(input.good());
    const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    rapidjson::Document receipt;
    receipt.Parse(text.c_str());
    ASSERT_FALSE(receipt.HasParseError());
    ASSERT_TRUE(receipt.HasMember("libraries"));
    ASSERT_TRUE(receipt["libraries"].HasMember("libprecice.so.3"));
    const auto& expected = receipt["libraries"]["libprecice.so.3"]["resolved"];
    ASSERT_TRUE(expected.IsString());
    Dl_info owner{};
    ASSERT_NE(dladdr(reinterpret_cast<const void*>(&precice::getVersionInformation), &owner), 0);
    ASSERT_NE(owner.dli_fname, nullptr);
    EXPECT_TRUE(std::filesystem::equivalent(owner.dli_fname, expected.GetString()));
    EXPECT_NO_THROW(precice::tooling::checkConfiguration(test_configuration, "Solver1", 1));
}

TEST(PreciceHost, ExistingAdapterRetainsTheProvidedSystemOwner) {
    auto system = chrono_types::make_shared<robodyna::simulation::RbSystemSMC>();
    chrono::ch_precice::ChPreciceAdapterMbs adapter(system, 0.001);
    EXPECT_EQ(&adapter.GetSystem(), system.get());
    EXPECT_EQ(adapter.GetModelName(), "model_MBS");
    EXPECT_EQ(system->GetVisualSystem(), nullptr);
    auto body = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    body->SetMass(3);
    body->SetPos(chrono::ChVector3d(0, 0, 2));
    system->AddBody(body);
    adapter.AddCouplingBody(body, {{0, 0, 0}, {1, 0, 0}});
    EXPECT_EQ(body->GetSystem(), system.get());
    EXPECT_EQ(adapter.GetSystem().GetBodies().size(), 1);
    EXPECT_DOUBLE_EQ(system->GetChTime(), 0);
    // Advancing the public shared system is a mechanics ownership check only;
    // no preCICE participant has been registered or initialized here.
    ASSERT_TRUE(adapter.GetSystem().DoStepDynamics(0.001));
    EXPECT_DOUBLE_EQ(system->GetChTime(), 0.001);
    EXPECT_TRUE(std::isfinite(body->GetPos().z()));
    EXPECT_EQ(system->GetVisualSystem(), nullptr);
}

TEST(PreciceHost, OriginalSphereYamlCreatesTheActualModelAndInterfaces) {
    using Adapter = chrono::ch_precice::ChPreciceAdapter;
    chrono::ch_precice::ChPreciceAdapterMbs adapter(participant_yaml, false);
    auto& system = adapter.GetSystem();
    auto ball = system.SearchBody("ball");
    ASSERT_NE(ball, nullptr);
    EXPECT_EQ(system.GetBodies().size(), 1);
    EXPECT_EQ(adapter.GetModelName(), "ball");
    EXPECT_DOUBLE_EQ(ball->GetMass(), 3.62);
    EXPECT_EQ(ball->GetPos(), chrono::ChVector3d(0, 0, 0.18));
    EXPECT_EQ(ball->GetInertiaXX(), chrono::ChVector3d(0.021, 0.021, 0.021));
    EXPECT_EQ(adapter.GetCouplingMeshType("SolidMesh"), Adapter::CouplingMeshType::RIGID_BODY_REFS);
    EXPECT_EQ(adapter.GetCouplingDataType("SolidMesh", "positions"), Adapter::CouplingDataType::POSITIONS);
    EXPECT_EQ(adapter.GetCouplingDataType("SolidMesh", "forces"), Adapter::CouplingDataType::FORCES);
    EXPECT_EQ(adapter.GetCouplingDataType("SolidMesh", "torques"), Adapter::CouplingDataType::TORQUES);
    EXPECT_THROW(adapter.GetCouplingMeshType("absent"), std::out_of_range);
    EXPECT_EQ(system.GetVisualSystem(), nullptr);
    EXPECT_DOUBLE_EQ(system.GetChTime(), 0);
}

int main(int argc, char** argv) {
    if (argc < 4)
        return 2;
    participant_yaml = std::filesystem::absolute(argv[1]).string();
    test_configuration = std::filesystem::absolute(argv[2]).string();
    sdk_receipt = std::filesystem::absolute(argv[3]).string();
    for (int i = 4; i < argc; ++i)
        argv[i - 3] = argv[i];
    argc -= 3;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
