#include "chrono_parsers/ChParserPython.h"
#include "chrono/physics/ChSystemNSC.h"
#include "src/io/parsers/runtime/Resources.h"
#include "gtest/gtest.h"

#include <filesystem>
#include <string>

namespace {
std::string package_manifest;
std::string model;
}

TEST(EmbeddedPython, RetainedEngineExchangesValuesAndOwnsImportedSharedBody) {
    robodyna::parsers::ConfigureEmbeddedPackage(package_manifest);
    chrono::ChSystemNSC system;
    system.SetNumThreads(1, 1, 1);
    system.SetGravitationalAcceleration({0, 0, -9.81});
    std::shared_ptr<chrono::ChBody> body;
    {
        // Only one interpreter exists. C++ keeps ownership after Py_Finalize.
        chrono::parsers::ChPythonEngine engine;
        engine.Run("import pychrono, robodyna\nassert pychrono.ChBody is robodyna.ChBody\nvalue = 6 * 7");
        int value = 0;
        ASSERT_TRUE(engine.GetInteger("value", value));
        EXPECT_EQ(value, 42);
        engine.SetFloat("answer", 3.25);
        double answer = 0;
        ASSERT_TRUE(engine.GetFloat("answer", answer));
        EXPECT_DOUBLE_EQ(answer, 3.25);
        engine.ImportSolidWorksSystem(robodyna::parsers::ResolveRunfile(model), system);
        ASSERT_EQ(system.GetBodies().size(), 1U);
        body = system.GetBodies().front();
        ASSERT_TRUE(body);
        EXPECT_EQ(body->GetSystem(), &system);
        EXPECT_DOUBLE_EQ(body->GetMass(), 2);
        EXPECT_EQ(body->GetName(), "embedded_body");
    }
    ASSERT_TRUE(body);
    EXPECT_EQ(system.GetBodies().front().get(), body.get());
    EXPECT_DOUBLE_EQ(body->GetPos().z(), 2);
    ASSERT_TRUE(system.DoStepDynamics(0.001));
    EXPECT_NEAR(system.GetChTime(), 0.001, 1e-15);
    EXPECT_NEAR(body->GetPosDt().z(), -0.00981, 1e-10);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 3) return 2;
    package_manifest = argv[1];
    model = argv[2];
    return RUN_ALL_TESTS();
}
