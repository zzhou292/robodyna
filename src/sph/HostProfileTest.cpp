// Check incoming compiler definitions before CCCL headers select their defaults.
#if defined(THRUST_DEVICE_SYSTEM) || defined(THRUST_HOST_SYSTEM) || defined(CH_API_COMPILE_FSI)
#error "Implementation-only definitions must not leak into ordinary SPH consumers"
#endif

#include "chrono_fsi/ChFsiDefinitions.h"
#include "chrono_fsi/sph/ChFsiDataTypesSPH.h"

#include <gtest/gtest.h>

#include <limits>
#include <type_traits>

#ifndef CHRONO_USE_CUDA
#error "SPH public host headers must select the same CUDA runtime as their implementation"
#endif
#ifdef CHRONO_USE_HIP
#error "This profile is CUDA, not a mixed HIP/CUDA build"
#endif
#ifdef CHRONO_SPH_USE_DOUBLE
#error "The initial SPH profile must retain the CMake-default float precision"
#endif
#ifdef CHRONO_HAS_SPLASHSURF
#error "No Splashsurf executable has been admitted in the initial SPH profile"
#endif

static_assert(std::is_same_v<chrono::fsi::sph::Real, float>);
static_assert(std::is_same_v<gpuStream, cudaStream_t>);
static_assert(sizeof(chrono::fsi::sph::Real3) == 3 * sizeof(float));
static_assert(sizeof(chrono::fsi::sph::Real4) == 4 * sizeof(float));

TEST(SphHostProfile, RetainedSinglePrecisionBoundsAreAvailableWithoutConstructingFluid) {
    const chrono::fsi::sph::RealAABB bounds;
    EXPECT_EQ(bounds.min.x, std::numeric_limits<float>::max());
    EXPECT_EQ(bounds.min.y, std::numeric_limits<float>::max());
    EXPECT_EQ(bounds.min.z, std::numeric_limits<float>::max());
    EXPECT_EQ(bounds.max.x, -std::numeric_limits<float>::max());
    EXPECT_EQ(bounds.max.y, -std::numeric_limits<float>::max());
    EXPECT_EQ(bounds.max.z, -std::numeric_limits<float>::max());
}

TEST(SphHostProfile, GenericBodyStateRetainsItsMechanicalDoublePrecisionContract) {
    chrono::fsi::FsiBodyState state;
    state.pos = chrono::ChVector3d(1.25, -2.5, 3.75);
    EXPECT_DOUBLE_EQ(state.pos.x(), 1.25);
    EXPECT_DOUBLE_EQ(state.pos.y(), -2.5);
    EXPECT_DOUBLE_EQ(state.pos.z(), 3.75);
    EXPECT_DOUBLE_EQ(state.rot.e0(), 1);
    EXPECT_DOUBLE_EQ(state.rot.e1(), 0);
    EXPECT_DOUBLE_EQ(state.rot.e2(), 0);
    EXPECT_DOUBLE_EQ(state.rot.e3(), 0);
}
