#pragma once
#include "output/physical_frames/NativeAcceptedFrames.h"
#include <gtest/gtest.h>
#include <array>
#include <vector>
namespace crash::cases::native_scene::capture_test {
inline std::vector<double> Copy(const output::physical_frames::NativeAcceptedState& s) {
    std::vector<double> out;out.reserve(21*s.nodes);
    const auto append=[&](const double* values,std::size_t n){out.insert(out.end(),values,values+n);};
    append(s.position_xyz,3*s.nodes);append(s.velocity_xyz,3*s.nodes);append(s.orientation_wxyz,4*s.nodes);
    append(s.spin_xyz,3*s.nodes);append(s.reaction_force_xyz,3*s.nodes);append(s.reaction_couple_xyz,3*s.nodes);
    append(s.mass_kg,s.nodes);append(s.inertia_kg_m2,s.nodes);return out;
}
inline void Same(const std::vector<double>& expected,const output::physical_frames::NativeAcceptedState& actual) {
    const auto copied=Copy(actual);ASSERT_EQ(expected.size(),copied.size());
    for(std::size_t i=0;i<expected.size();++i)EXPECT_EQ(output::Bits(expected[i]),output::Bits(copied[i]));
}
inline std::array<double,15> HistoryValues(const native::NativeContactRow& row) {
    const auto& h=row.history;const auto& n=h.normal;
    return {n.previous_penetration,n.previous_stiffness,n.staged_penetration,n.staged_stiffness,n.damping_half_force,
        h.previous_force.x,h.previous_force.y,h.previous_force.z,h.staged_force.x,h.staged_force.y,h.staged_force.z,
        row.penetration_auxiliary,row.penetration_offset,row.selection_metric[0],row.selection_metric[1]};
}
inline void SameHistory(const native::NativeGeometryHistory& a,const native::NativeGeometryHistory& b) {
    EXPECT_EQ(a.secondary_source_id,b.secondary_source_id);EXPECT_EQ(a.generation,b.generation);
    for(unsigned i=0;i<4;++i)EXPECT_EQ(a.row.irtlm[i],b.row.irtlm[i]);
    const auto x=HistoryValues(a.row),y=HistoryValues(b.row);for(unsigned i=0;i<15;++i)EXPECT_EQ(output::Bits(x[i]),output::Bits(y[i]));
}

}
