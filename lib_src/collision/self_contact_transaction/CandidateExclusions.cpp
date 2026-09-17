// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CandidateExclusions.h"

namespace tlfea::contact::self_contact_transaction {

PolicyExclusionSource CandidateExclusions::source() noexcept {
  return {this, &Prepare, 15, {}};
}

SelfContactTransactionReport CandidateExclusions::Prepare(
    void* context, AcceptedFeatureExclusionCertificate* output,
    std::size_t capacity, std::size_t* count) noexcept {
  auto& source = *static_cast<CandidateExclusions*>(context);
  FixedTriangleFeatureCandidate features[15];
  fixed_triangle_features::PairFeatureResult discovery;
  const auto status = fixed_triangle_features::EvaluatePairFeaturesMaskedOnce(
      source.first_accepted, source.second_accepted, source.mask,
      features, 15, &discovery);
  if (status != FixedTriangleDiscoveryStatus::Ok) {
    SelfContactTransactionReport report;
    report.status = SelfContactTransactionStatus::DiscoveryFailure;
    report.message = "Accepted policy exclusion feature replay failed";
    report.pair = source.pair_ordinal;
    report.discovery_status = status;
    return report;
  }
  return BuildAcceptedSameRigidExclusions(
      source.active_use, {features, discovery.feature_count, true},
      source.descriptors, source.triangle_order, source.facet_count,
      source.activity, output, capacity, count);
}

}  // namespace tlfea::contact::self_contact_transaction
