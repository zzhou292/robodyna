#pragma once
#include "SourceAssemblyBindings.h"
#include "case/wall_penalty/UniformTranslationKinetic.h"

namespace crash::cases::source_assembly {
struct GroupInitialKinetic {
    std::uint64_t source_group_id=0,source_node_set_id=0;
    std::size_t member_count=0;
    tlfea::contact::Q4CertifiedIntegral native_members,aggregate;
    double generated_primary_mass_kg=0;
};
struct SourceAssemblyInitialKinetic {
    // Native sum uses the original global-node order. Aggregate sum uses each
    // ordinary node in that order, then each complete native aggregate once in
    // source-group order. They may round to the same value but are distinct
    // physical metrics. Uniform +X translation and zero spin only.
    tlfea::contact::Q4CertifiedIntegral native_nodes,with_aggregate_groups;
    tlfea::contact::Q4CertifiedIntegral generated_primary_mass_kg;
    std::size_t ordinary_nodes=0,member_nodes=0,generated_primaries=0;
    std::vector<GroupInitialKinetic> groups;
};
enum class InitialKineticStatus { Ok,InvalidInput,CertificateFailure,ResourceLimit };
struct InitialKineticReport {
    InitialKineticStatus status=InitialKineticStatus::InvalidInput;
    const char* message="Invalid assembly initial kinetic declaration";
    std::size_t node=SIZE_MAX,group=SIZE_MAX;
    explicit operator bool() const noexcept { return status==InitialKineticStatus::Ok; }
};
// Uses only already-qualified native M/J and group startup properties. This
// neither measures a live owner nor reconstructs a collocated moving sample.
// Failure leaves caller output unchanged. No primary enters the source nodes.
InitialKineticReport EncloseSourceAssemblyInitialKinetic(const SourceAssemblyBindings&,double speed,
                                                         SourceAssemblyInitialKinetic*);
} // namespace crash::cases::source_assembly
