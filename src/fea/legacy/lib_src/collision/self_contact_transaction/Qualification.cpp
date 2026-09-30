// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {
namespace fe = tl::fea;

SelfContactTransactionReport
QualificationAccess::ClassifyAcceptedFeaturePolicies(
    SelfContactTransaction& transaction,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    FixedTriangleFeatureView features,
    AcceptedFeaturePolicyEvidence* output, std::size_t capacity,
    std::size_t* count) noexcept {
  return ClassifyAcceptedFeaturePoliciesImpl(
      transaction, assembly, nullptr, features, output, capacity, count);
}

SelfContactTransactionReport
QualificationAccess::ClassifyAcceptedFeaturePolicies(
    SelfContactTransaction& transaction,
    const QualificationPreparedCensusReceipt& census,
    FixedTriangleFeatureView features,
    AcceptedFeaturePolicyEvidence* output, std::size_t capacity,
    std::size_t* count) noexcept {
  return ClassifyAcceptedFeaturePoliciesImpl(
      transaction, census.assembly_, &census, features,
      output, capacity, count);
}

SelfContactTransactionReport
QualificationAccess::ClassifyPreparedCandidateCensus(
    SelfContactTransaction& transaction, fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    NonlinearCandidateRosterEntry* nonlinear, std::size_t nonlinear_capacity,
    std::size_t* nonlinear_count, NonlinearCandidateRosterSummary* nonlinear_summary,
    LinearWorkExhaustedRosterEntry* linear, std::size_t linear_capacity,
    std::size_t* linear_count, LinearCandidateCensusSummary* linear_summary,
    PreparedMotionCertificateView* motion) noexcept {
  return ClassifyPreparedCandidateCensusImpl(
      transaction, owner, token, prepared, assembly,
      nonlinear, nonlinear_capacity, nonlinear_count, nonlinear_summary,
      linear, linear_capacity, linear_count, linear_summary,
      motion, nullptr, nullptr);
}

SelfContactTransactionReport
QualificationAccess::ClassifyPreparedCandidateCensus(
    SelfContactTransaction& transaction, fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    NonlinearCandidateRosterEntry* nonlinear, std::size_t nonlinear_capacity,
    std::size_t* nonlinear_count, NonlinearCandidateRosterSummary* nonlinear_summary,
    LinearWorkExhaustedRosterEntry* linear, std::size_t linear_capacity,
    std::size_t* linear_count, LinearCandidateCensusSummary* linear_summary,
    PreparedMotionCertificateView* motion,
    QualificationPreparedCensusReceipt* census) noexcept {
  return ClassifyPreparedCandidateCensusImpl(
      transaction, owner, token, prepared, assembly,
      nonlinear, nonlinear_capacity, nonlinear_count, nonlinear_summary,
      linear, linear_capacity, linear_count, linear_summary,
      motion, &diagnostics, census);
}

}  // namespace tlfea::contact::self_contact_transaction
