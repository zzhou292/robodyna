// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OperationChecks.h"
#include "Contexts.h"

namespace tl::fea::type45 {
BatchReport Batch::Impl::PrepareContexts(FENodalState& owner,const NodalTrialToken& token,
    const NodalPreparedView& view) {
  auto* mains=util::ArenaPointer<NodalCinPhysicalMain>(staging.data(),layout.mains);
  auto* contexts=util::ArenaPointer<AutomaticStiffnessContext>(staging.data(),layout.host_contexts);
  NodalCinPhysicalMainStamp receipt;
  const auto queried=owner.CopyPreparedCinPhysicalMains(token,{mains,layout.mains.count},&receipt);
  if(queried.status!=NodalStatus::Ok) {
    if(queried.status==NodalStatus::DeviceFailure) usable=false;
    return resident_detail::NodalFailure(queried);
  }
  const auto checked=resident_detail::Contexts(model,accepted_stamp,view,cin_qualification_id,
      receipt,mains,layout.mains.count,contexts);
  if(!checked) return checked;
  // Scratch may change on a failed first attempt; the accepted slab is still
  // virgin. After first Publish these exact contexts are retained and never reset.
  return Runtime(cudaMemcpyAsync(device_header.contexts,contexts,layout.contexts.bytes,
      cudaMemcpyHostToDevice,stream),"Joint initial main context upload failed");
}
} // namespace tl::fea::type45
