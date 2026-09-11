// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../solid18/Solid18History.h"
#include "../../solid24/Solid24ForceHistory.h"
#include "../../solid6z/Solid6zForceInitialize.h"

namespace tl::fea::solids::model_detail {
ModelReport CopyParents(ModelInput input,const Scratch& scratch,
    const SolidNodeContributions& coefficients,Storage& out) {
  const auto rows=coefficients.parents();
  for(std::size_t i=0;i<rows.size();++i) {
    const auto material=scratch.material_indices[i];
    std::size_t* nodes=nullptr;
    if(i<input.solid18.size()) {
      auto& parent=out.parent18[i];
      parent.reference=input.solid18[i].reference;
      parent.material_index=material;
      nodes=parent.domain_nodes;
      solid18::History initial;
      if(solid18::InitializeHistory(parent.reference,out.material36[material].value,initial)!=solid18::Status::Success)
        return Error(ModelStatus::InvalidInput,"Solid18 reference/material startup is unsupported",input,i);
    } else if(i<input.solid18.size()+input.solid24.size()) {
      const auto local=i-input.solid18.size();
      auto& parent=out.parent24[local];
      parent.reference=input.solid24[local].reference;
      parent.material_index=material;
      nodes=parent.domain_nodes;
      solid24::History initial;
      if(solid24::InitializeHistory(parent.reference,out.material42[material].value,initial)!=solid24::ForceStatus::Success)
        return Error(ModelStatus::InvalidInput,"HEPH reference/material startup is unsupported",input,i);
    } else {
      const auto local=i-input.solid18.size()-input.solid24.size();
      auto& parent=out.parent6z[local];
      parent.reference=input.solid6z[local].reference;
      parent.profile=input.solid6z[local].profile;
      parent.material_index=material;
      nodes=parent.domain_nodes;
      solid6z::History initial;
      if(solid6z::InitializeHistory(parent.reference,out.material42[material].value,parent.profile,initial)!=solid6z::Status::Success)
        return Error(ModelStatus::InvalidInput,"S6Z reference/material/profile startup is unsupported",input,i);
    }
    for(unsigned n=0;n<8;++n)nodes[n]=n<rows[i].node_count?rows[i].domain_node[n]:SIZE_MAX;
  }
  return {};
}
} // namespace tl::fea::solids::model_detail
