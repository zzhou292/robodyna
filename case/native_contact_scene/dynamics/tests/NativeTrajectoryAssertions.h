#pragma once
#include "NativeSceneAccess.h"
#include "lib_utest/qualification/radioss_type25_runtime/Reference.h"
#include <gtest/gtest.h>
#include <cmath>
#include <vector>
namespace crash::cases::native_scene::trajectory_test {
namespace reference=native_runtime_test;
inline void Number(double actual,double wanted,double absolute,double relative=2e-8) {
    ASSERT_TRUE(std::isfinite(actual));
        ASSERT_TRUE(std::isfinite(wanted));
    EXPECT_NEAR(actual,wanted,absolute+relative*std::abs(wanted));
}
inline void Motion(output::physical_frames::NativeAcceptedState state,const reference::ExpectedFrame& wanted) {
    ASSERT_TRUE(state.available);
        ASSERT_EQ(state.nodes,18u);
        ASSERT_EQ(state.stamp.epoch,wanted.epoch);
    Number(state.stamp.time,wanted.time,1e-16,0);
    for(unsigned i=0;i<54;++i){SCOPED_TRACE(i);Number(state.position_xyz[i]/.001,wanted.position[i],2e-8);Number(state.velocity_xyz[i]/.001,wanted.velocity[i],2e-5);}
}
inline void History(const native::NativeContactRow& actual,const native::NativeContactRow& wanted) {
    for(unsigned i=0;i<4;++i)ASSERT_EQ(actual.irtlm[i],wanted.irtlm[i]);
    const auto& a=actual.history.normal;const auto& b=wanted.history.normal;
    Number(a.previous_penetration,b.previous_penetration,2e-8);Number(a.staged_penetration,b.staged_penetration,2e-8);
    Number(a.previous_stiffness,b.previous_stiffness,2e-3);Number(a.staged_stiffness,b.staged_stiffness,2e-3);
    Number(a.damping_half_force,b.damping_half_force,2e-5);
    const double x[]{actual.history.previous_force.x,actual.history.previous_force.y,actual.history.previous_force.z,
        actual.history.staged_force.x,actual.history.staged_force.y,actual.history.staged_force.z};
    const double y[]{wanted.history.previous_force.x,wanted.history.previous_force.y,wanted.history.previous_force.z,
        wanted.history.staged_force.x,wanted.history.staged_force.y,wanted.history.staged_force.z};
    for(unsigned i=0;i<6;++i)Number(x[i],y[i],2e-5);
    Number(actual.penetration_auxiliary,wanted.penetration_auxiliary,2e-8);Number(actual.penetration_offset,wanted.penetration_offset,2e-8);
    for(unsigned i=0;i<2;++i){if(std::abs(wanted.selection_metric[i])==1e20)EXPECT_EQ(actual.selection_metric[i],wanted.selection_metric[i]);
        else Number(actual.selection_metric[i],wanted.selection_metric[i],2e-8);}
}
inline void Packets(const native::runtime_qualification::Observation& actual,const reference::ExpectedFrame& wanted) {
    ASSERT_EQ(actual.force_base_epoch,wanted.epoch);std::vector<int> secondary,main;
    for(const auto& item:actual.occurrences)if(item.secondary>0){secondary.push_back(item.secondary);main.push_back(item.selected.local_main);}
    ASSERT_EQ(actual.cohort_ends.size(),wanted.geometry_packets.size());std::size_t first=0;
    for(std::size_t i=0;i<actual.cohort_ends.size();++i){const auto last=actual.cohort_ends[i];
        ASSERT_GE(last,first);
        ASSERT_LE(last,secondary.size());
        EXPECT_EQ(std::vector<int>(secondary.begin()+first,secondary.begin()+last),wanted.geometry_packets[i].secondary);
        EXPECT_EQ(std::vector<int>(main.begin()+first,main.begin()+last),wanted.geometry_packets[i].main);first=last;}
    EXPECT_EQ(first,secondary.size());
}
}
