#pragma once
#include "SourceAssemblyObservation.h"
#include "lib_src/elements/qeph/QephForceData.h"
#include <array>

namespace crash::cases::source_assembly_observation {
inline constexpr std::size_t MaxSpinParents=8;
struct QephSpinParent {
    std::uint64_t source_parent=0,source_part=0,source_material=0,source_section=0,source_curve=0;
    std::size_t source_index=0,family_index=0,local_node=0;
    std::array<std::uint64_t,4> source_nodes{};
    std::array<tl::math::Vec3,4> position{},velocity{},omega{};
    fe::qeph::ForceTrial force;
    fe::ShellBatchSectionState section;
    bool has_native_kinematics=false;
    tl::math::Vec3 native_normal;
    double normal_spin=0,tangent_spin=0,normal_internal_couple=0;
    // Powers use the actual carried midpoint spin with this retained force
    // packet. They are phase-labelled diagnostics, not collocated work.
    double normal_internal_power=0,tangent_internal_power=0;
};
struct QephSpinObservation {
    fe::NodalStamp base,enclosing; // enclosing is supplied only by the sole successful commit.
    std::uint64_t attempt=0,source_instance=0,source_node=0;
    std::size_t global_node=SIZE_MAX,parent_count=0;
    bool completed=false;
    fe::ShellBindingMass native;
    tl::math::Vec3 position,velocity,omega,applied_force,applied_couple;
    std::array<double,4> orientation{};
    tl::math::Vec3 negative_parent_couple_sum,assembly_couple_residual;
    std::array<QephSpinParent,MaxSpinParents> parents,candidate_parents;

};
struct QephSpinInput {
    const SourceAssemblyBindings* bindings=nullptr;
    std::uint64_t source_node=0;
    std::uint64_t configuration_id=0,qualification_id=0;
    fe::NodalStamp base;
    fe::NodalPreparedView prepared;
    fe::HostNodalKinematicsView before,after;
    const fe::qeph::BatchDiagnostics* parent_diagnostics=nullptr;
    const fe::qeph::BatchDiagnostics* candidate_diagnostics=nullptr;
    const fe::qeph::ForceTrial* parents=nullptr;
    const fe::ShellBatchSectionState* sections=nullptr;
    const fe::qeph::ForceTrial* candidate_parents=nullptr;
    const fe::ShellBatchSectionState* candidate_sections=nullptr;
    std::size_t parent_count=0;
    const double* applied_force_xyz=nullptr;
    const double* applied_couple_xyz=nullptr;
};
// Complete incident coverage for an ordinary physical node with 1..8 QEPH
// parents and no T3 incidence. Input pointer extents must describe actual owned
// arrays. The caller authenticates live accepted/prepared/load readbacks; this
// pure function verifies source/phase association and retains original packets.
// No force/geometry evaluation, mass formula, allocation or acceptance occurs.
// Every inspected range must be disjoint from output. Failure preserves output.
Report CheckQephSpinSource(const SourceAssemblyBindings&,std::uint64_t source_node) noexcept;
Report ObserveQephSpin(const QephSpinInput&,QephSpinObservation*) noexcept;
}
