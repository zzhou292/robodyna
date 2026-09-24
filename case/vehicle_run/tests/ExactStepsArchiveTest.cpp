#include "../Config.h"
#include "output/full_shell/FullShellVisualizationPlan.h"
#include <gtest/gtest.h>
namespace crash::cases::vehicle_run::test {
TEST(VehicleRunExactStepsArchive, ActualPlannerSavesFiftyOneStatesThroughEpochSixThousand) {
    for (double dt : {2e-7, 2.25e-7}) {
        Config config;
        config.exact_steps = 6000;
        config.samples = 51;
        config.fixed_dt_s = dt;
        const auto horizon = Plan(config);
        output::full_shell::PlanRequest request;
        request.nodes = 4;
        request.parents = 1;
        request.frames = config.samples;
        request.intervals = horizon.intervals;
        request.fixed_dt = horizon.fixed_dt_s;
        request.requested_duration = horizon.requested_duration_s;
        request.static_files = {{"manifest.json", 1}, {"frame-index.json", 1},
                                {"configuration.json", 1}};
        const auto archive = output::full_shell::PlanArchive(request);
        ASSERT_EQ(archive.frame_epochs.size(), 51u);
        EXPECT_EQ(archive.frame_capacity, 52u); // One extra genuine prefix state is reserved.
        for (std::size_t i = 0; i < archive.frame_epochs.size(); ++i)
            EXPECT_EQ(archive.frame_epochs[i], 120 * i);
        EXPECT_EQ(archive.frame_epochs.front(), 0u);
        EXPECT_EQ(archive.frame_epochs.back(), 6000u);
    }
}
} // namespace crash::cases::vehicle_run::test
