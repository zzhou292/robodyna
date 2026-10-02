#include "chrono_vehicle/ChConfigVehicle.h"
#include "chrono_vehicle/terrain/CRMTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/test_rig/ChWheelTestRig.h"

#include <gtest/gtest.h>
#include <type_traits>

#if !defined(CHRONO_CRM) || !defined(CHRONO_FSI_SPH)
#error "Vehicle CRM and shared SPH capability must be enabled together"
#endif
#ifdef CHRONO_HAS_SCM_GPU
#error "This CRM admission must preserve the separately qualified CPU SCM implementation"
#endif

static_assert(std::is_base_of_v<chrono::vehicle::ChTerrain, chrono::vehicle::CRMTerrain>);
static_assert(std::is_base_of_v<chrono::fsi::sph::ChFsiProblemCartesian, chrono::vehicle::CRMTerrain>);
static_assert(std::is_member_function_pointer_v<decltype(&chrono::vehicle::ChWheelTestRigBase::WheelAssembly::AddFSIBodies)>);

TEST(VehicleCrmProfile, OriginalWheelRigParameterConstructorLinksUnderTheSameProfile) {
    // This out-of-line original constructor is compiled by the existing wheel
    // rig owner. It stores parameters only; no terrain/fluid/device is created.
    chrono::vehicle::ChWheelTestRigBase::TerrainParamsCRM params;
    EXPECT_EQ(params.sph_params.integration_scheme, chrono::fsi::sph::IntegrationScheme::RK2);
    EXPECT_EQ(params.sph_params.viscosity_method, chrono::fsi::sph::ViscosityMethod::ARTIFICIAL_BILATERAL);
    EXPECT_EQ(params.sph_params.boundary_method, chrono::fsi::sph::BoundaryMethod::ADAMI);
}
