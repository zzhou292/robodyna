#include "chrono_dem/ChConfigDem.h"
#include "chrono_dem/physics/ChSystemDem.h"
#include "chrono_dem/utils/ChDemJsonParser.h"

#include <gtest/gtest.h>

#include <string>
#include <type_traits>

#ifndef CHRONO_USE_CUDA
#error "The retained DEM target must select its actual CUDA runtime backend"
#endif
#ifdef CHRONO_USE_HIP
#error "This first native DEM profile is CUDA, not a mixed HIP/CUDA build"
#endif

static_assert(std::is_same_v<gpuError, cudaError_t>);
static_assert(std::is_same_v<gpuStream, cudaStream_t>);

namespace {
std::string configuration_file;

TEST(DemHostContract, PublicHostTypesUseTheSelectedCudaBackend) {
    // Host-only function object from the original public header: no device
    // allocation, system constructor, CUDA API call or kernel launch occurs.
    const auto position = chrono::dem::GranPosFunction_default(.25f);
    EXPECT_EQ(position.x, 0);
    EXPECT_EQ(position.y, 0);
    EXPECT_EQ(position.z, 0);
}

TEST(DemHostContract, OriginalBallConfigurationIsAdmittedWithoutUnitConversion) {
    chrono::dem::ChDemSimulationParameters parameters{};
    ASSERT_TRUE(chrono::dem::ParseJSON(configuration_file, parameters, false));
    EXPECT_FLOAT_EQ(parameters.sphere_radius, 1.0f);
    EXPECT_FLOAT_EQ(parameters.sphere_density, 1.0f);
    EXPECT_FLOAT_EQ(parameters.box_X, 300.0f);
    EXPECT_FLOAT_EQ(parameters.box_Y, 300.0f);
    EXPECT_FLOAT_EQ(parameters.box_Z, 200.0f);
    EXPECT_FLOAT_EQ(parameters.step_size, 5e-5f);
    EXPECT_FLOAT_EQ(parameters.time_end, 2.0f);
    EXPECT_FLOAT_EQ(parameters.grav_Z, -980.0f);
    EXPECT_EQ(parameters.psi_T, 32u);
    EXPECT_EQ(parameters.psi_L, 16u);
    EXPECT_EQ(parameters.output_dir, "ballCosim");
    EXPECT_EQ(parameters.write_mode, chrono::dem::CHDEM_OUTPUT_MODE::CSV);
}
}  // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    if (argc != 2)
        return 2;
    configuration_file = argv[1];
    return RUN_ALL_TESTS();
}
