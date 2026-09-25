#include "ResolutionData.h"
#include <algorithm>
namespace crash::modelio::vehicle {
namespace {
void CheckProfile(const VehicleSectionResolution& base,ResolutionProfile profile) {
    output::Require(profile==ResolutionProfile::OriginalRigidPartsV1 &&
        base.resolution_key().profile==ResolutionProfile::OriginalMidlayerV1 && base.includes_glass() &&
        base.counts().unresolved_parts==22 && base.counts().unresolved_shells==5102 &&
        base.counts().shells==349645 && base.counts().midlayer_shells==4251,
        "Rigid source profile requires the unchanged original midlayer resolution");
}
rigid_part::Limits SourceLimits(ResolutionLimits limits) {
    rigid_part::Limits out;
    // The larger outer overlay envelope does not expand the source reader.
    out.host_bytes = std::min(limits.host_bytes, rigid_part::Limits{}.host_bytes);
    out.parts = limits.parts;
    return out;
}
}
std::size_t VehicleSectionResolution::ForecastOriginalRigidParts(const VehicleSectionResolution& base,
    ResolutionProfile profile,ResolutionLimits limits) {
    resolution::Preflight(base.source(), base.identity(), limits, resolution::BudgetScope::CompleteRigidOverlay);
    CheckProfile(base,profile);
    std::size_t total=0;
    const auto add=[&](std::size_t count,std::size_t width) {
        output::Require(total<=limits.host_bytes && count<=(limits.host_bytes-total)/width,
                        "Rigid resolution overlay exceeds host cap");
        total+=count*width;
    };
    add(base.startup_budget_bytes(),1);
    add(rigid_part::RigidPartSource::AdditionalForecast(base.source(),SourceLimits(limits)),1);
    add(sizeof(Data)+256,1);
    add(base.parts().size(),sizeof(SectionPartResolution));
    add(base.parents().size(),sizeof(NativeParentMapping)+sizeof(tl::fea::ShellFailureParentInput));
    add(22,sizeof(tl::fea::ShellPlasticityMaterialInput));
    return total;
}
VehicleSectionResolution VehicleSectionResolution::ResolveOriginalRigidParts(const VehicleSectionResolution& base,
    const std::string& member,ResolutionProfile profile,ResolutionLimits limits) {
    const auto budget=ForecastOriginalRigidParts(base,profile,limits);
    auto next=std::make_shared<Data>(base.source());
    next->base=base.data_;next->key={base.identity(),profile};next->budget=budget;
    next->rigid.emplace(rigid_part::RigidPartSource::Prepare(base.source(),member,SourceLimits(limits)));
    next->native_materials.reserve(next->rigid->data().bodies.size());
    next->declarations.parts=base.parts();next->declarations.counts=base.counts();next->declarations.includes_glass=true;
    for (const auto& body:next->rigid->data().bodies) {
        auto& part=next->declarations.parts[body.source_part_index];
        output::Require(part.status==SectionDisposition::Unresolved,"Rigid source overwrites a resolved part");
        part.status=SectionDisposition::RigidPart;
        next->native_materials.push_back(assembly::detail::NativeMaterial(body.declaration.material));
    }
    auto& counts=next->declarations.counts;
    counts.rigid_parts=22;counts.rigid_shells=5102;counts.unresolved_parts=0;counts.unresolved_shells=0;
    auto mapped=resolution::MapNativeParents(base.parents(),next->declarations.parts,base.source().parts());
    output::Require(mapped.counts.qbat==4250 && mapped.counts.qeph+mapped.counts.t3+mapped.counts.qbat==349645,
                    "Rigid source complete formulation coverage changed");
    next->mapping=std::move(mapped.mapping);next->native_parents=std::move(mapped.parents);next->native_counts=mapped.counts;
    return VehicleSectionResolution(std::move(next));
}
}
