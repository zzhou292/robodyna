#include "chrono_fmi/fmi2/ChFmuForgeImport.h"

#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>

namespace {
std::string archive_path;
using Variable = chrono::fmi2::FmuVariable;

TEST(FmiTemplate, ActualArchiveConstantsSurviveSteppingAndMembersRemainLive) {
    const char* temporary = std::getenv("TEST_TMPDIR");
    ASSERT_NE(temporary, nullptr);
    const auto unpack = std::filesystem::path(temporary) / "template-bindings";
    ASSERT_TRUE(std::filesystem::create_directory(unpack));
    chrono::fmi2::FmuChronoUnit fmu;
    fmu.Load(fmi2CoSimulation, archive_path, unpack.string());
    fmu.Instantiate("bounded-template", false, false);
    struct InstanceLifetime {
        chrono::fmi2::FmuChronoUnit& fmu;
        ~InstanceLifetime() {
            // FmuUnit's destructor is empty; release this instance explicitly.
            // This does not claim a general importer unload facility.
            fmu._fmi2FreeInstance(fmu.component);
            fmu.component = nullptr;
        }
    } lifetime{fmu};
    ASSERT_EQ(fmu.SetupExperiment(fmi2False, 0, 0, fmi2True, .001), fmi2OK);
    ASSERT_EQ(fmu.EnterInitializationMode(), fmi2OK);
    ASSERT_EQ(fmu.ExitInitializationMode(), fmi2OK);
    std::map<std::string, int> constants;
    for (const auto& [name, variable] : fmu.GetVariablesList()) {
        if (variable.GetType() == Variable::Type::Integer &&
            variable.GetVariability() == Variable::VariabilityType::constant) {
            int value = 0;
            ASSERT_EQ(fmu.GetVariable(name, value, Variable::Type::Integer), fmi2OK) << name;
            constants.emplace(name, value);
        }
    }
    ASSERT_FALSE(constants.empty());
    unsigned versions = 0, dimensions = 0;
    for (const auto& [name, value] : constants) {
        if (name.find("_version_") != std::string::npos) ++versions;
        if ((name.size() >= 5 && name.substr(name.size() - 5) == ".rows") ||
            (name.size() >= 8 && name.substr(name.size() - 8) == ".columns")) {
            ++dimensions;
            EXPECT_GT(value, 0) << name;
        }
    }
    EXPECT_GT(versions, 0u);
    EXPECT_GT(dimensions, 0u);
    double time = -1;
    int steps = -1;
    ASSERT_EQ(fmu.GetVariable("sys.ch_time", time, Variable::Type::Real), fmi2OK);
    ASSERT_EQ(fmu.GetVariable("sys.stepcount", steps, Variable::Type::Integer), fmi2OK);
    EXPECT_DOUBLE_EQ(time, 0);
    EXPECT_EQ(steps, 0);
    ASSERT_EQ(fmu.DoStep(0, .001, fmi2True), fmi2OK);
    ASSERT_EQ(fmu.GetVariable("sys.ch_time", time, Variable::Type::Real), fmi2OK);
    ASSERT_EQ(fmu.GetVariable("sys.stepcount", steps, Variable::Type::Integer), fmi2OK);
    EXPECT_NEAR(time, .001, 1e-14);
    EXPECT_GT(steps, 0);
    for (const auto& [name, before] : constants) {
        int value = -1;
        ASSERT_EQ(fmu.GetVariable(name, value, Variable::Type::Integer), fmi2OK) << name;
        EXPECT_EQ(value, before) << name;
    }
    EXPECT_EQ(fmu._fmi2Terminate(fmu.component), fmi2OK);
}
}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    archive_path = argv[1];
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
