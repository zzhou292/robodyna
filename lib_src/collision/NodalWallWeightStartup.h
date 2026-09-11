#pragma once
#include "NodalWallContact.h"
#include "lib_utils/SourceIdentityIndex.h"

namespace tlfea::contact::nodal_wall_detail {
using WeightFeatureIndex=tl::util::SourceIdentityIndex<0>;
using WeightNodeFlags=tl::util::BoundedStartupArray<std::uint8_t,0>;
// Called only after the explicit profile's hard counts have passed. These are
// transient payloads; retained weights and caller references are separate.
inline std::size_t WeightStartupBytes(std::size_t parents,std::size_t nodes) noexcept {
  return WeightFeatureIndex::Bytes(parents)+sizeof(WeightNodeFlags)+WeightNodeFlags::ExtraBytes(nodes);
}
inline bool AccumulateWeight(Q4CertifiedIntegral& sum,Q4CertifiedIntegral value) {
  Q4IntegralInterval truth;
  return q4_bounds::Add({sum.lower,sum.upper},{value.lower,value.upper},&truth)&&
    q4_bounds::Certify(sum.value+value.value,truth,&sum);
}
// Exact owning Q4/T3 extraction, shared by source-to-physical authentication.
// This is an allocation-free value operation; failure preserves output.
NodalWallReport PrepareParentWeight(std::uint32_t,const NodalWallParentInput&,
    NodalWallParentWeight*) noexcept;
// Parent references are already validated and sorted by the legacy identity
// order. Every node receives those shares in exactly that order. Output arrays
// belong to a fresh staging value and are never externally visible on failure.
NodalWallReport BuildIndexedWeights(const NodalWallParentWeight*,std::uint32_t,
  NodalWallNodeWeight*,std::uint32_t,std::uint32_t&,Q4CertifiedIntegral&);
} // namespace tlfea::contact::nodal_wall_detail
