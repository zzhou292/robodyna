#include "ShellMixedSectionStorage.h"
#include "ShellLayeredSectionValues.h"

namespace tl::fea::shell_batch_plasticity_detail {
SetupReport MixedHostStorage::Read(unsigned slab,std::size_t count,cudaStream_t stream,
    const ShellBatchPlasticityBinding& catalog) noexcept {
  if(!device_||slab>1||count!=count_||!catalog.heterogeneous_sections())
    return {SetupStatus::InvalidInput,"Mixed section readback shape is invalid"};
  auto error=cudaMemcpyAsync(plastic_.data(),header_.plastic.section[slab],count*sizeof(ShellBatchSectionState),
    cudaMemcpyDeviceToHost,stream);
  if(error==cudaSuccess)error=cudaMemcpyAsync(elastic_.data(),header_.elastic_section[slab],
    count*sizeof(sections::ShellLayeredLaw1History),cudaMemcpyDeviceToHost,stream);
  if(error==cudaSuccess)error=cudaStreamSynchronize(stream);
  if(error!=cudaSuccess)return {SetupStatus::DeviceFailure,"Mixed section readback failed",error};
  // Complete finite/availability preflight precedes even private output staging.
  for(std::size_t e=0;e<count;++e) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;
    if(!catalog.Law(family_,e,&law))return {SetupStatus::InvalidInput,"Mixed section source identity is unavailable"};
    if(law==ShellSectionLaw::LayeredLaw1Nip3) {
      if(!FiniteSection(elastic_[e]))return {SetupStatus::NonfiniteResult,"Nonfinite elastic section history"};
    } else if(law==ShellSectionLaw::LayeredLaw44Nip3) {
      if(!FiniteSection(plastic_[e]))return {SetupStatus::NonfiniteResult,"Nonfinite plastic section history"};
    } else return {SetupStatus::InvalidInput,"Unsupported section readback law"};
  }
  for(std::size_t e=0;e<count;++e) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;catalog.Law(family_,e,&law);
    output_[e]=law==ShellSectionLaw::LayeredLaw1Nip3?
      ShellBatchLayeredSection::Elastic(elastic_[e]):ShellBatchLayeredSection::Plastic(plastic_[e]);
  }
  return {SetupStatus::Success,"OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
