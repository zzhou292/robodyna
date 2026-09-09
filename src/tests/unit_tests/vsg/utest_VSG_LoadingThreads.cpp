// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
// Copyright (c) 2026 projectchrono.org
// Use of this source code is governed by the BSD-style license in LICENSE.
// =============================================================================

#include <gtest/gtest.h>
#include <memory>
#include <stdexcept>

#include "chrono_vsg/ChVisualSystemVSG.h"

using chrono::vsg3d::ChVisualSystemVSG;
using chrono::vsg3d::ChVisualSystemVSGPlugin;

namespace {
class StopBeforeWindow : public std::runtime_error {
  public:
    StopBeforeWindow() : std::runtime_error("intentional pre-window callback failure") {}
};
class FailingInitialization : public ChVisualSystemVSGPlugin {
  public:
    int calls = 0;
    bool count_was_locked = false;
  protected:
    void OnInitialize() override {
        ++calls;
        try {
            GetVisualSystemVSG().SetLoadingThreadCount(3);
        } catch (const std::logic_error&) {
            count_was_locked = true;
        }
        throw StopBeforeWindow();  // Runs before window/device or worker creation.
    }
};
}  // namespace

TEST(VSGLoadingThreads, DefaultAndValidPreInitializeConfiguration) {
    ChVisualSystemVSG visual;
    EXPECT_FALSE(visual.IsInitialized());
    EXPECT_EQ(visual.GetLoadingThreadCount(), 16);
    for (int count : {1, 64, 2}) {
        EXPECT_NO_THROW(visual.SetLoadingThreadCount(count));
        EXPECT_EQ(visual.GetLoadingThreadCount(), count);
    }
    EXPECT_FALSE(visual.IsInitialized());
}

TEST(VSGLoadingThreads, InvalidCountsPreserveConfiguredValue) {
    ChVisualSystemVSG visual;
    visual.SetLoadingThreadCount(1);
    for (int count : {-1, 0, 65, 2147483647}) {
        EXPECT_THROW(visual.SetLoadingThreadCount(count), std::invalid_argument);
        EXPECT_EQ(visual.GetLoadingThreadCount(), 1);
    }
    EXPECT_NO_THROW(visual.SetLoadingThreadCount(2));
    EXPECT_EQ(visual.GetLoadingThreadCount(), 2);
}

TEST(VSGLoadingThreads, FirstAttemptLocksBeforeCallbacksAndRetainsLockAfterFailure) {
    ChVisualSystemVSG visual;
    visual.SetLoadingThreadCount(1);
    auto callback = std::make_shared<FailingInitialization>();
    visual.AttachPlugin(callback);
    EXPECT_THROW(visual.Initialize(), StopBeforeWindow);
    EXPECT_EQ(callback->calls, 1);
    EXPECT_TRUE(callback->count_was_locked);
    EXPECT_FALSE(visual.IsInitialized());
    EXPECT_EQ(visual.GetLoadingThreadCount(), 1);
    EXPECT_THROW(visual.SetLoadingThreadCount(2), std::logic_error);
    EXPECT_EQ(visual.GetLoadingThreadCount(), 1);
    // Existing Initialize retry behavior remains intact; configuration remains
    // locked even when the preceding attempt never created a window.
    EXPECT_THROW(visual.Initialize(), StopBeforeWindow);
    EXPECT_EQ(callback->calls, 2);
    EXPECT_EQ(visual.GetLoadingThreadCount(), 1);
}
