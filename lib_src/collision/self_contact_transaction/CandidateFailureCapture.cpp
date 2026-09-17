// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CandidateFailureCapture.h"
#include "QualificationRanges.h"

#include <utility>

namespace tlfea::contact::self_contact_transaction {
namespace fe = tl::fea;
namespace {
SelfContactTransactionReport Failure(
    SelfContactTransactionStatus status, const char* message) noexcept {
  SelfContactTransactionReport report;
  report.status = status;
  report.message = message;
  return report;
}
}  // namespace

SelfContactTransactionReport
QualificationAccess::SealCandidateWithFailureObserver(
    SelfContactTransaction& transaction, fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    SelfContactTransactionReceipt* output,
    const CandidateFailureObserver& observer) {
  if (!transaction.impl_)
    return Failure(SelfContactTransactionStatus::NotInitialized,
        "Failure-observer transaction is not initialized");
  auto& state = *transaction.impl_;
  const QualificationRange outputs[]{
      QualificationBorrowedRange(output),
      QualificationBorrowedRange(
          static_cast<const unsigned char*>(observer.context),
          observer.context_bytes)};
  const QualificationRange inputs[]{
      QualificationBorrowedRange(&transaction),
      QualificationBorrowedRange(&owner),
      QualificationBorrowedRange(&token),
      QualificationBorrowedRange(&diagnostics),
      QualificationBorrowedRange(&prepared),
      QualificationBorrowedRange(&assembly),
      QualificationBorrowedRange(&observer)};
  if (!observer.capture || !observer.context || !observer.context_bytes ||
      !ValidateQualificationRanges(outputs, inputs,
          [&](const void* data, std::size_t bytes) {
            return state.OutputDisjoint(data, bytes);
          }))
    return Failure(SelfContactTransactionStatus::InvalidInput,
        "Failure-observer output/context is invalid or aliases live state");
  return transaction.SealCandidateImpl(
      owner, token, diagnostics, prepared, assembly, output, &observer);
}

void QualificationAccess::ObserveCandidateFailure(
    SelfContactTransaction& transaction,
    const CandidateFailureObserver* observer,
    const SelfContactTransactionReport& report, FixedTrianglePair facets,
    const fe::NodalStamp& accepted, const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedAssemblyReceipt& assembly,
    const SelfContactPreparedActivityReceipt& activity) noexcept {
  if (!observer || !transaction.impl_ ||
      report.status == SelfContactTransactionStatus::Ok)
    return;
  const auto& state = *transaction.impl_;
  if (state.phase != SelfContactTransaction::Impl::Phase::AssemblyRecorded ||
      !activity.valid() || facets.first >= state.facet_count ||
      facets.second >= state.facet_count || facets.first == facets.second)
    return;
  const auto view = activity.activity();
  if (!view.base || !view.current ||
      view.parent_count != state.active_use.parents().size())
    return;
  if (fixed_triangle_features::Compare(
          state.buffers.prepared_triangles[facets.second].key,
          state.buffers.prepared_triangles[facets.first].key) < 0)
    std::swap(facets.first, facets.second);
  CandidateFailureCapture capture;
  capture.transaction = &transaction;
  capture.accepted_assembly = &assembly;
  capture.report = report;
  capture.facets = facets;
  capture.accepted = accepted;
  capture.prepared = prepared;
  capture.motion = {
      state.buffers.facet_descriptors, state.buffers.facet_motion,
      state.buffers.facet_quadratic, state.buffers.accepted_triangles,
      state.buffers.prepared_triangles, state.buffers.swept_facet_bounds,
      state.facet_count, true};
  capture.accepted_events = {
      state.buffers.accepted_certificates, state.accepted_event_count, true};
  capture.activity.transaction_ = &transaction;
  capture.activity.assembly_ = assembly;
  capture.activity.activity_ = activity;
  auto& summary = capture.activity.activity_summary_;
  summary.selected = view.parent_count;
  for (std::size_t parent = 0; parent < view.parent_count; ++parent) {
    summary.accepted_active += view.base[parent] != 0;
    summary.prepared_active += view.current[parent] != 0;
    summary.removing += view.base[parent] != 0 && view.current[parent] == 0;
    summary.inactive += view.base[parent] == 0;
  }
  summary.complete = true;
  observer->capture(observer->context, capture);
}

}  // namespace tlfea::contact::self_contact_transaction
