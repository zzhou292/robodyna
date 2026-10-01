#include <gtest/gtest.h>
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/utils/ChConstants.h"
#include "chrono_vehicle/terrain/SCMTerrain.h"

#include <link.h>
#include <string>

#ifdef CHRONO_HAS_SCM_GPU
#error "This retained SCM profile qualifies the original CPU/Bullet path"
#endif

TEST(RetainedScm, MissingCollisionSystemIsRejected) {
    chrono::ChSystemSMC system;
    EXPECT_THROW(chrono::vehicle::SCMTerrain terrain(&system, false), std::runtime_error);
}

TEST(RetainedScm, FlatYUpTerrainRetainsGridCoordinatesAndExplicitHeights) {
    chrono::vehicle::ChWorldFrame::SetYUP();
    chrono::ChSystemSMC system;
    system.SetNumThreads(1, 1, 1);
    system.SetCollisionSystem(std::make_shared<chrono::ChCollisionSystemBullet>());
    chrono::vehicle::SCMTerrain terrain(&system, false);
    terrain.SetReferenceFrame(chrono::ChCoordsys<>(chrono::VNULL, chrono::QuatFromAngleX(-chrono::CH_PI_2)));
    terrain.Initialize(2, 6, .04);
    EXPECT_NEAR(terrain.GetInitHeight({0, 1, 0}), 0, 1e-14);
    EXPECT_TRUE(terrain.GetModifiedNodes(true).empty());
    terrain.SetModifiedNodes({{chrono::ChVector2i(0, 0), -.02}});
    EXPECT_NEAR(terrain.GetHeight({0, 1, 0}), -.02, 1e-14);
    ASSERT_EQ(terrain.GetModifiedNodes(true).size(), 1u);
    EXPECT_EQ(terrain.GetNumRaycastGpuSteps(), 0);
    EXPECT_EQ(terrain.GetNumContactForceGpuSteps(), 0);
}

TEST(RetainedScm, HeadlessTerrainDoesNotLoadGraphicsOrHistoricalChronoLibraries) {
    std::string unexpected;
    dl_iterate_phdr([](dl_phdr_info* info, std::size_t, void* data) {
        const std::string path = info->dlpi_name ? info->dlpi_name : "";
        for (const auto* name : {"libvsg", "libvulkan", "libChrono_"}) {
            if (path.find(name) != std::string::npos)
                *static_cast<std::string*>(data) = path;
        }
        return 0;
    }, &unexpected);
    EXPECT_TRUE(unexpected.empty()) << unexpected;
}
