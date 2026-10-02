#include "chrono_peridynamics/ChPeridynamics.h"
#include "chrono_peridynamics/ChMatterPeriSprings.h"

#include <gtest/gtest.h>

TEST(PeridynamicsAdmission, SamplingPreservesTotalMassAndPlacement) {
    auto matter = std::make_shared<chrono::peridynamics::ChMatterPeriSprings>();
    chrono::peridynamics::ChPeridynamics domain;
    domain.AddMatter(matter);
    std::vector<chrono::ChVector3d> points{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    domain.Fill(matter, points, 1, 6, 3, 1.6, .3, chrono::ChCoordsys<>(chrono::ChVector3d(2, 3, 4)));
    ASSERT_EQ(domain.GetNodes().size(), 3u);
    double mass = 0;
    double volume = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        const auto& node = domain.GetNodes()[i];
        mass += node->GetMass();
        volume += node->volume;
        EXPECT_NEAR((node->GetPos() - points[i] - chrono::ChVector3d(2, 3, 4)).Length(), 0, 1e-14);
        EXPECT_NEAR((node->GetPos() - node->GetX0()).Length(), 0, 1e-14);
    }
    EXPECT_DOUBLE_EQ(mass, 6);
    EXPECT_DOUBLE_EQ(volume, 3);
}
