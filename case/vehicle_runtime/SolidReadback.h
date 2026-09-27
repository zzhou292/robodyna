#pragma once
#include "lib_src/elements/solids/resident/Batch.h"
#include "output/ArtifactIO.h"
namespace crash::cases::vehicle_runtime::detail {
inline bool HasSourceSolidControls(const tl::fea::solids::Model& model) noexcept {
    const auto* selection=model.control_selection();
    return selection && selection->profile()==tl::fea::solids::control::Profile::SourceDeclared &&
        selection->controlled_count()!=0;
}
inline std::size_t SolidReadbackBytes(const tl::fea::solids::Model& model) noexcept {
    namespace s=tl::fea::solids;const bool controlled=HasSourceSolidControls(model);
    return model.solid18().size()*sizeof(s::Result18)+model.solid6z().size()*sizeof(s::Result6z)+
        model.solid18_law44().size()*sizeof(s::Result18Law44)+
        model.solid24().size()*(controlled?sizeof(s::ProfiledResult24):sizeof(s::Result24))+
        model.solid18_law90().size()*(controlled?sizeof(s::ProfiledResult18Law90):sizeof(s::Result18Law90));
}
template<class Rows> void CheckInitialSolidStamps(const Rows& rows) {
    for(const auto& row:rows)output::Require(row.stamp.sample_index==0&&row.stamp.time_s==0,
        "Solid constructor cache has advanced");
}
} // namespace crash::cases::vehicle_runtime::detail
