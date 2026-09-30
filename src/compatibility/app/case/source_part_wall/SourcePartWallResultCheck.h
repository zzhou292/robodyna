#pragma once
#include "SourcePartWallContact.h"
#include "output/ArtifactIO.h"
#include <gtest/gtest.h>

namespace crash::cases::source_part_wall::check {
template<class T> void SameValue(const T& a,const T& b) {EXPECT_EQ(a,b);}
inline void SameValue(double a,double b) {EXPECT_EQ(output::Bits(a),output::Bits(b));}
inline void SameValue(contact::Vec3 a,contact::Vec3 b) {SameValue(a.x,b.x);SameValue(a.y,b.y);SameValue(a.z,b.z);}
inline void SameValue(contact::Q4CertifiedIntegral a,contact::Q4CertifiedIntegral b) {
    SameValue(a.value,b.value);SameValue(a.lower,b.lower);SameValue(a.upper,b.upper);SameValue(a.error,b.error);
}
// Every scientific/identity field is compared explicitly, avoiding padding.
// Only retry attempt identifiers may change, including nested point rows.
inline void SameScientificResult(const contact::NodalWallDeviceResults& clean,const contact::NodalWallDeviceResults& retry) {
    {
        const auto& a=clean.diagnostics;const auto& b=retry.diagnostics;
#define SAME(field) SameValue(a.field,b.field)
        SAME(owner_id);SAME(configuration_id);SAME(qualification_id);SAME(wall_binding_id);SAME(base_epoch);
        SAME(phase);SAME(scheme);SAME(velocity_phase);SAME(time);SAME(velocity_time);SAME(base_time);SAME(base_velocity_time);SAME(kick_dt);
        SAME(resultant);SAME(potential);SAME(wall_reaction);SAME(wall_moment);SAME(surface_power);SAME(maximum_penetration);
        SAME(stiffness_rate_bound);SAME(base_potential);SAME(base_potential_error);SAME(potential_increment);
        SAME(kick_work);SAME(kick_work_roundoff);SAME(drift_work);SAME(drift_work_roundoff);
        SAME(conservative_defect);SAME(work_uncertainty);SAME(quadratic_work_upper);
        SAME(wall_kick_impulse);SAME(wall_kick_impulse_error);SAME(wall_kick_moment);SAME(wall_kick_moment_error);
        SAME(node_count);SAME(parent_count);SAME(valid);
        EXPECT_GT(b.attempt,a.attempt);
#undef SAME
    }
    for(unsigned p=0;p<contact::MaxNodalWallDeviceParents;++p) {
        SCOPED_TRACE(p);const auto& a=clean.parents[p];const auto& b=retry.parents[p];
        for(unsigned n=0;n<4;++n)SameValue(a.force[n],b.force[n]);
        SameValue(a.resultant,b.resultant);SameValue(a.potential,b.potential);SameValue(a.parent_element_id,b.parent_element_id);
        SameValue(a.feature_id,b.feature_id);SameValue(a.parent_face_id,b.parent_face_id);
        SameValue(a.arity,b.arity);SameValue(a.family,b.family);SameValue(a.valid,b.valid);
    }
    for(unsigned n=0;n<contact::MaxNodalWallDeviceNodes;++n) {
        SCOPED_TRACE(n);const auto& a=clean.nodes[n];const auto& b=retry.nodes[n];
#define SAME(field) SameValue(a.field,b.field)
        SAME(force);SAME(potential);SAME(stiffness);SAME(force_world);SAME(wall_point);SAME(wall_reaction);SAME(wall_moment);
        SAME(surface_power);SAME(local_velocity_first_timestep);SAME(base_epoch);SAME(node);SAME(fixed);SAME(touching_or_penetrating);SAME(valid);
        SAME(row.count);SAME(row.base_epoch);SAME(row.valid);
        for(unsigned i=0;i<tl::fea::stability::MaxNodes;++i) {
            SameValue(a.row.nodes[i],b.row.nodes[i]);SameValue(a.row.stiffness[i],b.row.stiffness[i]);SameValue(a.row.damping[i],b.row.damping[i]);
        }
        SameValue(clean.wall_face[n],retry.wall_face[n]);
        if(a.attempt) {EXPECT_EQ(a.attempt,clean.diagnostics.attempt);EXPECT_EQ(b.attempt,retry.diagnostics.attempt);}
        else EXPECT_EQ(b.attempt,0u);
        if(a.row.attempt) {EXPECT_EQ(a.row.attempt,clean.diagnostics.attempt);EXPECT_EQ(b.row.attempt,retry.diagnostics.attempt);}
        else EXPECT_EQ(b.row.attempt,0u);
#undef SAME
    }
}
} // namespace crash::cases::source_part_wall::check
