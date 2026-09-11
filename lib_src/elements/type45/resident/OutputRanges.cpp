// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include "../../ShellPhysicalOutputRanges.h"
#include "../../ShellExecutionOutputRanges.h"

namespace tl::fea::type45 {
bool resident_detail::ModelOutputDisjoint(const Model& model,const void* output,std::size_t bytes) noexcept {
  using trial_identity::Disjoint;
  return model.prepared() && Disjoint(output,bytes,&model,sizeof(model)) &&
      Disjoint(output,bytes,model.joints().data(),model.joints().size()*sizeof(Joint)) &&
      shell_execution_detail::OutputDisjoint(*model.rigid_binding(),output,bytes) &&
      shell_physical_owner::OutputDisjoint(*model.rigid_binding()->coefficients(),output,bytes);
}
bool Batch::Impl::OutputDisjoint(const void* output,std::size_t bytes) const noexcept {
  using trial_identity::Disjoint;
  return Disjoint(output,bytes,this,sizeof(*this)) &&
      Disjoint(output,bytes,staging.data(),staging.bytes()) &&
      resident_detail::ModelOutputDisjoint(model,output,bytes) &&
      (!physical_scope || shell_physical_owner::OutputDisjoint(*physical_scope,output,bytes));
}
bool Batch::Impl::OutputBuffer(ResultBuffer output,const void* input,std::size_t input_bytes,
    const void* batch,std::size_t batch_bytes) const noexcept {
  const auto address=reinterpret_cast<std::uintptr_t>(output.joints);
  if(!output.joints || output.count!=model.joints().size() || address%alignof(Result) ||
      output.count>(UINTPTR_MAX-address)/sizeof(Result)) return false;
  const auto bytes=output.count*sizeof(Result);
  return OutputDisjoint(output.joints,bytes) && trial_identity::Disjoint(output.joints,bytes,input,input_bytes) &&
      trial_identity::Disjoint(output.joints,bytes,batch,batch_bytes);
}
} // namespace tl::fea::type45
