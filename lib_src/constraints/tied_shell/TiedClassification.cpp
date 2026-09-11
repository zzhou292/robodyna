// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedClassificationTags.h"
#include <new>

namespace tl::constraints::tied_shell {
ClassificationPhase ClassificationResult::phase() const noexcept {
  return data_ ? data_->phase : ClassificationPhase::Empty;
}
std::uint64_t ClassificationResult::source_instance_id() const noexcept { return data_ ? data_->source : 0; }
ClassificationView<ClassificationNode> ClassificationResult::nodes() const noexcept {
  if(!data_) return {};
  return {data_->nodes.get(),data_->node_count};
}
ClassificationView<ClassifiedInterface> ClassificationResult::interfaces() const noexcept {
  if(!data_) return {};
  return {data_->interfaces.get(),data_->interface_count};
}
ClassificationView<std::int32_t> ClassificationResult::irupt() const noexcept {
  if(!data_) return {};
  return {data_->irupt.get(),data_->slave_count};
}
ClassificationNodes ClassificationResult::slave_nodes() const noexcept {
  return data_ ? ClassificationNodes{data_->slaves.get(),data_->slave_count} : ClassificationNodes{};
}
ClassificationView<std::int32_t> ClassificationResult::interface_decode() const noexcept {
  if(!data_) return {};
  return {data_->itf.data(),data_->itf.size()};
}
std::uint64_t ClassificationResult::native_kinset_warnings() const noexcept { return data_ ? data_->kinset_warnings : 0; }
std::uint64_t ClassificationResult::native_penalty_warnings() const noexcept { return data_ ? data_->penalty_warnings : 0; }
std::size_t ClassificationResult::owned_payload_bytes() const noexcept { return data_ ? data_->owned : 0; }
std::size_t ClassificationResult::startup_payload_bytes() const noexcept { return data_ ? data_->startup : 0; }

ClassificationReport Classify(const ClassificationInput& in,ClassificationResult* out,ClassificationLimits limits) noexcept {
  using namespace classification_detail;
  if(!out) return Fail(Status::InvalidInput);
  try {
    Counts counts;
    auto report=Preflight(in,*out,limits,sizeof(ClassificationResult::Data),counts);
    if(!report) return report;
    report=CheckContext(in.context);
    if(!report) return report;
    report=CheckRoles(in);
    if(!report) return report;
    report=CheckInterfaceIdentities(in);
    if(!report) return report;
    auto next=std::make_shared<ClassificationResult::Data>();
    Initialize(in.context,counts,*next);
    next->slave_count=counts.slaves;
    report=RunClassification(in,*next);
    if(!report) return report;
    std::shared_ptr<const ClassificationResult::Data> committed=std::move(next);
    out->data_.swap(committed);
    return {};
  } catch(const std::bad_alloc&) {
    return Fail(Status::ResourceLimit);
  }
}
ClassificationReport RegisterRigidMembers(const RigidRegistrationInput& in,ClassificationResult* out,ClassificationLimits limits) noexcept {
  using namespace classification_detail;
  if(!out) return Fail(Status::InvalidInput);
  try {
    Counts counts;
    auto report=Preflight(in,*out,limits,sizeof(ClassificationResult::Data),counts);
    if(!report) return report;
    report=CheckContext(in.context);
    if(!report) return report;
    report=CheckRoles(in);
    if(!report) return report;
    report=CheckGroupIdentities(in);
    if(!report) return report;
    auto next=std::make_shared<ClassificationResult::Data>();
    Initialize(in.context,counts,*next);
    report=RunRigidRegistration(in,*next);
    if(!report) return report;
    std::shared_ptr<const ClassificationResult::Data> committed=std::move(next);
    out->data_.swap(committed);
    return {};
  } catch(const std::bad_alloc&) {
    return Fail(Status::ResourceLimit);
  }
}
} // namespace tl::constraints::tied_shell
