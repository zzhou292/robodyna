#pragma once
#include "lib_src/constraints/NodalRigidGroupState.h"

namespace crash::output::assembly {
// Inactive attachment scope is represented ONLY by an all-zero descriptor.
// Otherwise require the complete immutable source model association; a partial
// or differently instanced owner cannot borrow this assembly's provenance.
inline bool MatchesAssemblyRigidScope(tl::fea::NodalRigidGroupInfo actual,
                                      tl::fea::NodalRigidGroupInfo expected) noexcept {
    if(!actual.source_instance_id&&!actual.group_count&&!actual.member_count)return true;
    return actual.source_instance_id&&actual.group_count&&actual.member_count&&
        tl::fea::SameRigidGroupInfo(actual,expected);
}
} // namespace crash::output::assembly
