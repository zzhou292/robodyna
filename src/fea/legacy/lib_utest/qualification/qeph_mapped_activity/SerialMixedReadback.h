#pragma once
#include "MixedOracleSupport.h"
#include "SerialMixedValues.h"

namespace qeph_activity_test::frozen_mixed {
inline SetupReport MixedHostStorage::Read(unsigned slab,std::size_t count,cudaStream_t stream,
    const ShellBatchPlasticityBinding& catalog,const ShellBatchOnePointSectionState* one_point) noexcept {
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
    } else if(law==ShellSectionLaw::Law44Nip1) {
      // The optional owner stages and validates its complete payload first.
      if(!one_point||family_!=ShellBindingFamily::T3)
        return {SetupStatus::InvalidInput,"One-point history is unavailable"};
    } else if(law!=ShellSectionLaw::RigidSkin||!catalog.execution_sections()) {
      return {SetupStatus::InvalidInput,"Unsupported section readback law"};
    }
  }
  for(std::size_t e=0;e<count;++e) {
    ShellSectionLaw law=ShellSectionLaw::Unspecified;catalog.Law(family_,e,&law);
    if(law==ShellSectionLaw::LayeredLaw1Nip3)
      output_[e]=ShellBatchLayeredSection::Elastic(elastic_[e]);
    else if(law==ShellSectionLaw::LayeredLaw44Nip3)
      output_[e]=ShellBatchLayeredSection::Plastic(plastic_[e]);
    else if(law==ShellSectionLaw::Law44Nip1)
      output_[e]=ShellBatchLayeredSection::OnePoint(one_point[e]);
    else output_[e]=ShellBatchLayeredSection::RigidSkin();
  }
  return {SetupStatus::Success,"OK"};
}
} // namespace tl::fea::shell_batch_plasticity_detail
