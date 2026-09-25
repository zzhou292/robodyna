#include "viewer/physical_run/Options.h"
#include "chrono/ReplayVisuals.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
#include <vector>

namespace crash::viewer::physical_run::test {
namespace {
Options ParseArgs(std::initializer_list<const char*> args) {
    std::vector<const char*> values(args);
    return Parse(static_cast<int>(values.size()), const_cast<char**>(values.data()));
}
void SameCamera(const visual::ReplayCamera& a, const visual::ReplayCamera& b) {
    EXPECT_EQ(std::memcmp(a.position.data(), b.position.data(), sizeof(a.position)), 0);
    EXPECT_EQ(std::memcmp(a.target.data(), b.target.data(), sizeof(a.target)), 0);
    EXPECT_EQ(a.vertical, b.vertical);
    EXPECT_EQ(a.view, b.view);
    EXPECT_EQ(a.vertical_fov_degrees, b.vertical_fov_degrees);
}
} // namespace

TEST(PhysicalPartPalette, StrictUnsignedSeedDefaultAndFullWidthValueReachScene) {
    EXPECT_EQ(ParseArgs({"viewer","run"}).scene.part_palette_seed,1u);
    EXPECT_EQ(ParseArgs({"viewer","run","--part-palette-seed","0"}).scene.part_palette_seed,0u);
    EXPECT_EQ(ParseArgs({"viewer","run","--part-palette-seed","2"}).scene.part_palette_seed,2u);
    EXPECT_EQ(ParseArgs({"viewer","run","--part-palette-seed","18446744073709551615"}).scene.part_palette_seed,UINT64_MAX);
    for(const char* invalid:{"","-1","+2"," 2","2 ","2.0","2x","18446744073709551616"})
        EXPECT_THROW(ParseArgs({"viewer","run","--part-palette-seed",invalid}),std::exception);
    EXPECT_THROW(ParseArgs({"viewer","run","--part-palette-seed","2","--part-palette-seed","3"}),std::exception);
    EXPECT_THROW(ParseArgs({"viewer","run","--part-palette-seed","2","--color","plastic-strain"}),std::exception);
}

TEST(PhysicalFixedCamera, PairedSiCoordinatesAndExistingUpAxesReachCameraUnchanged) {
    const auto options = ParseArgs({"viewer", "run", "--camera-eye", "-1.25,2e-1,0.8",
        "--camera-target", "0.5,-0.0,0.4"});
    ASSERT_TRUE(options.scene.fixed_camera);
    visual::ReplayCamera camera;
    ASSERT_TRUE(visual::MakeFixedCamera(*options.scene.fixed_camera, camera));
    const std::array<double, 3> eye{-1.25, .2, .8}, target{.5, -0.0, .4};
    EXPECT_EQ(std::memcmp(camera.position.data(), eye.data(), sizeof(eye)), 0);
    EXPECT_EQ(std::memcmp(camera.target.data(), target.data(), sizeof(target)), 0);
    EXPECT_EQ(camera.vertical, visual::ReplayVertical::Z);
    EXPECT_EQ(camera.vertical_fov_degrees, 40);
    EXPECT_STREQ(visual::ReplayViewName(camera.view), "explicit-fixed");
    const auto y_up = ParseArgs({"viewer", "run", "--camera-up", "y", "--camera-target", "0,0,1",
        "--camera-eye", "0,0,3"});
    ASSERT_TRUE(visual::MakeFixedCamera(*y_up.scene.fixed_camera, camera));
    EXPECT_EQ(camera.vertical, visual::ReplayVertical::Y);
    EXPECT_EQ(camera.position, (std::array<double, 3>{0, 0, 3}));
}

TEST(PhysicalFixedCamera, IncompleteConflictingAndDuplicateOptionsReject) {
    for (const char* name : {"--camera-eye", "--camera-target"})
        EXPECT_THROW(ParseArgs({"viewer", "run", name, "1,2,3"}), std::exception);
    EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-up", "z"}), std::exception);
    EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", "1,2,3", "--camera-target", "0,0,0",
        "--view", "incident-side"}), std::exception);
    for (const char* name : {"--camera-eye", "--camera-target", "--camera-up"}) {
        const char* value = std::string(name) == "--camera-up" ? "z" : "1,2,3";
        EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", "1,2,3", "--camera-target", "0,0,0",
            "--camera-up", "z", name, value}), std::exception);
    }
    EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", "1,2,3", "--camera-target", "0,0,0",
        "--camera-up", "x"}), std::exception);
    EXPECT_THROW(ParseArgs({"viewer", "run", "--view", "explicit-fixed"}), std::exception);
}

TEST(PhysicalFixedCamera, StrictTriplesFiniteValuesAndRepresentableLookBasis) {
    for (const char* bad : {"", "1,2", "1,2,3,4", "1,,3", "1,2,", "1,2,3m", "nan,2,3",
            "1,inf,3", "1,2,-inf", "1e309,2,3", "1e-200,0,0", "1e200,0,0", "0,0,0", "0,0,3"}) {
        SCOPED_TRACE(bad);
        EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", bad, "--camera-target", "0,0,0"}),
            std::exception);
    }
    EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", "1,2,3", "--camera-target", "1,2,3"}),
        std::exception);
    EXPECT_THROW(ParseArgs({"viewer", "run", "--camera-eye", "1,2,3", "--camera-target", "1,2,nan"}),
        std::exception);
}

TEST(PhysicalFixedCamera, InvalidProgrammaticInputStagesCompleteOutputAndPermitsRetry) {
    visual::ReplayCamera camera;
    const visual::FixedCameraInput good{{-2, -1, .5}, {0, 0, .25}};
    ASSERT_TRUE(visual::MakeFixedCamera(good, camera));
    const auto accepted = camera;
    for (unsigned fault = 0; fault < 8; ++fault) {
        auto bad = good;
        switch (fault) {
            case 0: bad.target = bad.eye; break;
            case 1: bad.eye[2] = std::numeric_limits<double>::infinity(); break;
            case 2: bad.target[2] = std::numeric_limits<double>::quiet_NaN(); break;
            case 3: bad.vertical = static_cast<visual::ReplayVertical>(99); break;
            case 4: bad.eye = {0, 0, 1}; bad.target = {0, 0, 0}; break;
            case 5: bad.eye = {0, 1, 0}; bad.target = {0, 0, 0}; bad.vertical = visual::ReplayVertical::Y; break;
            case 6: bad.eye = {1e308, 0, 0}; bad.target = {-1e308, 0, 0}; break;
            case 7: bad.eye = {1e-200, 0, 1}; bad.target = {0, 0, 0}; break;
        }
        EXPECT_FALSE(visual::MakeFixedCamera(bad, camera));
        SameCamera(camera, accepted);
    }
    ASSERT_TRUE(visual::MakeFixedCamera(good, camera));
    SameCamera(camera, accepted);
}

TEST(PhysicalFixedCamera, DefaultAndNamedBoundsChoicesRetainExactCameraValues) {
    const auto defaults = ParseArgs({"viewer", "run"});
    const auto named = ParseArgs({"viewer", "run", "--view", "incident-side"});
    const auto wall = ParseArgs({"viewer", "run", "--view", "wall-side"});
    EXPECT_FALSE(defaults.scene.fixed_camera);
    EXPECT_FALSE(named.scene.fixed_camera);
    EXPECT_FALSE(wall.scene.fixed_camera);
    visual::ReplayCamera camera, explicit_default;
    ASSERT_TRUE(visual::MakeBoundsCamera({-2, -1, 0}, {3, 2, 2}, {-1, -1, .45}, .85,
        visual::ReplayVertical::Z, defaults.scene.view, camera));
    ASSERT_TRUE(visual::MakeBoundsCamera({-2, -1, 0}, {3, 2, 2}, {-1, -1, .45}, .85,
        visual::ReplayVertical::Z, named.scene.view, explicit_default));
    SameCamera(camera, explicit_default);
    EXPECT_STREQ(visual::ReplayViewName(camera.view), "incident-side");
    EXPECT_STREQ(visual::ReplayViewName(wall.scene.view), "wall-side");
}
} // namespace crash::viewer::physical_run::test
