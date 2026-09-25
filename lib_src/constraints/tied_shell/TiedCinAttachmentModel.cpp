// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TiedCinAttachmentInternal.h"
#include <climits>

namespace tl::constraints::tied_shell {
CinAttachmentReport ForecastCinAttachments(const PostKinChkResult& post,const fea::NodalNodeDomain& domain,
    std::size_t count,std::size_t old_bytes,CinAttachmentForecast* output,CinAttachmentLimits limits) noexcept {
  using namespace cin_detail;
  if (!output) return Fail(ResultStatus::InvalidInput);
  const CinAttachmentLimits hard;
  if (!limits.max_attachments || limits.max_attachments > hard.max_attachments ||
      !limits.max_host_bytes || limits.max_host_bytes > hard.max_host_bytes) return Fail(ResultStatus::ResourceLimit);
  if (count > limits.max_attachments || count > INT_MAX/64) return Fail(ResultStatus::ResourceLimit);
  if (!domain.prepared() || !count || count != post.slaves().count ||
      domain.source_instance_id() != post.source_instance_id()) return Fail(ResultStatus::SourceMismatch);
  util::BoundedArenaLayout budget(limits.max_host_bytes);
  util::ArenaRegion region;
  CinAttachmentForecast out;
  if (domain.owned_payload_bytes() < sizeof(fea::NodalNodeDomain)) return Fail(ResultStatus::ResourceLimit);
  // Data already owns the copied domain handle; charge its backing once.
  out.domain_payload_bytes = domain.owned_payload_bytes()-sizeof(fea::NodalNodeDomain);
  out.post_kinchk_payload_bytes = post.forecast().owned_payload_bytes;
  if (!budget.Append<unsigned char>(sizeof(TiedCinAttachmentModel),region) ||
      !budget.Append<unsigned char>(sizeof(TiedCinAttachmentModel::Data),region) ||
      !budget.Append<unsigned char>(SharedControlReserveBytes,region) ||
      !budget.Append<CinAttachmentRow>(count,region)) return Fail(ResultStatus::ResourceLimit);
  out.model_payload_bytes = budget.bytes();
  if (!budget.Append<Index::Entry>(2*count,region) || !budget.Append<unsigned char>(2*sizeof(Index),region))
    return Fail(ResultStatus::ResourceLimit);
  out.scratch_bytes = budget.bytes()-out.model_payload_bytes;
  for (const auto bytes : {old_bytes,out.domain_payload_bytes,out.post_kinchk_payload_bytes}) {
    if (!budget.Append<unsigned char>(bytes,region)) return Fail(ResultStatus::ResourceLimit);
  }
  out.startup_payload_bytes = budget.bytes();
  *output = out;
  return {};
}
CinAttachmentReport PrepareCinAttachments(const PostKinChkResult& post,const fea::NodalNodeDomain& domain,
    ClassificationView<CinAttachmentDeclaration> input,TiedCinAttachmentModel* output,CinAttachmentLimits limits) noexcept {
  using namespace cin_detail;
  if (!output) return Fail(ResultStatus::InvalidInput);
  CinAttachmentForecast bytes;
  std::size_t retained_old = output->forecast().model_payload_bytes;
  if (output->prepared()) {
    util::BoundedArenaLayout old(limits.max_host_bytes);
    util::ArenaRegion region;
    if (!old.Append<unsigned char>(retained_old,region)) return Fail(ResultStatus::ResourceLimit);
    if (!output->domain()->SharesStorage(domain) &&
        !old.Append<unsigned char>(output->forecast().domain_payload_bytes,region)) return Fail(ResultStatus::ResourceLimit);
    if (!output->classification()->SharesStorage(post) &&
        !old.Append<unsigned char>(output->classification()->forecast().owned_payload_bytes,region))
      return Fail(ResultStatus::ResourceLimit);
    retained_old = old.bytes();
  }
  auto report = ForecastCinAttachments(post,domain,input.count,retained_old,&bytes,limits);
  if (!report) return report;
  if (!fea::nodal_domain_detail::ValidRange(input.data,input.count)) return Fail(ResultStatus::InvalidInput);
  try {
    Index rank, element;
    rank.Prepare(input.count,[&](std::size_t row) { return input.data[row].ordered_master_rank; });
    element.Prepare(input.count,[&](std::size_t row) { return input.data[row].master_source.element_id; });
    auto next = std::make_shared<TiedCinAttachmentModel::Data>(post,domain);
    next->rows = std::make_unique<CinAttachmentRow[]>(input.count);
    for (std::size_t row = 0; row < input.count; ++row) {
      const auto& classified = post.slaves().data[row];
      if (post.interface_decode().data[classified.kinet] != 1)
        return Fail(ResultStatus::UnsupportedDisposition,row);
      if (row && input.data[row].original_nsv_row <= input.data[row-1].original_nsv_row)
        return Fail(ResultStatus::SourceMismatch,row);
      report = Map(input.data[row],post.slaves().data[row],domain,row,next->rows[row]);
      if (!report) return report;
      const auto first = rank.First(input.data[row].ordered_master_rank);
      if (first < row && !SameMaster(next->rows[first],next->rows[row]))
        return Fail(ResultStatus::ConflictingMaster,row);
      const auto first_element = element.First(input.data[row].master_source.element_id);
      if (first_element < row && next->rows[first_element].ordered_master_rank != next->rows[row].ordered_master_rank)
        return Fail(ResultStatus::ConflictingMaster,row);
    }
    next->count = input.count;
    next->bytes = bytes;
    output->data_ = std::move(next);
    return {};
  } catch (...) {
    return Fail(ResultStatus::ResourceLimit);
  }
}
CinAttachmentReport ForecastEmptyCinAttachments(const fea::NodalNodeDomain& domain,
    std::size_t old_bytes,CinAttachmentForecast* output,CinAttachmentLimits limits) noexcept {
  using namespace cin_detail;
  if (!output || !domain.prepared()) return Fail(ResultStatus::InvalidInput);
  const CinAttachmentLimits hard;
  if (!limits.max_host_bytes || limits.max_host_bytes>hard.max_host_bytes ||
      limits.max_attachments>hard.max_attachments) return Fail(ResultStatus::ResourceLimit);
  if (domain.owned_payload_bytes()<sizeof(fea::NodalNodeDomain)) return Fail(ResultStatus::ResourceLimit);
  CinAttachmentForecast next;
  next.domain_payload_bytes=domain.owned_payload_bytes()-sizeof(fea::NodalNodeDomain);
  util::BoundedArenaLayout budget(limits.max_host_bytes);util::ArenaRegion ignored;
  if (!budget.Append<unsigned char>(sizeof(TiedCinAttachmentModel),ignored) ||
      !budget.Append<unsigned char>(sizeof(TiedCinAttachmentModel::Data),ignored) ||
      !budget.Append<unsigned char>(SharedControlReserveBytes,ignored)) return Fail(ResultStatus::ResourceLimit);
  next.model_payload_bytes=budget.bytes();
  if (!budget.Append<unsigned char>(old_bytes,ignored) ||
      !budget.Append<unsigned char>(next.domain_payload_bytes,ignored)) return Fail(ResultStatus::ResourceLimit);
  next.startup_payload_bytes=budget.bytes();*output=next;return {};
}
CinAttachmentReport PrepareEmptyCinAttachments(const fea::NodalNodeDomain& domain,
    TiedCinAttachmentModel* output,CinAttachmentLimits limits) noexcept {
  using namespace cin_detail;
  if (!output) return Fail(ResultStatus::InvalidInput);
  util::BoundedArenaLayout old(limits.max_host_bytes);util::ArenaRegion ignored;
  if (output->prepared()) {
    const auto bytes=output->forecast();
    if (!old.Append<unsigned char>(bytes.model_payload_bytes,ignored) ||
        !old.Append<unsigned char>(bytes.post_kinchk_payload_bytes,ignored) ||
        (!output->domain()->SharesStorage(domain)&&
         !old.Append<unsigned char>(bytes.domain_payload_bytes,ignored))) return Fail(ResultStatus::ResourceLimit);
  }
  CinAttachmentForecast bytes;
  auto report=ForecastEmptyCinAttachments(domain,old.bytes(),&bytes,limits);if(!report)return report;
  try {
    auto next=std::make_shared<TiedCinAttachmentModel::Data>(PostKinChkResult{},domain);
    next->explicitly_empty=true;next->bytes=bytes;output->data_=std::move(next);return {};
  } catch(...) {return Fail(ResultStatus::ResourceLimit);}
}
bool TiedCinAttachmentModel::explicitly_empty() const noexcept {
  return data_&&data_->explicitly_empty;
}
const fea::NodalNodeDomain* TiedCinAttachmentModel::domain() const noexcept { return data_ ? &data_->domain : nullptr; }
const PostKinChkResult* TiedCinAttachmentModel::classification() const noexcept { return data_ ? &data_->post : nullptr; }
ClassificationView<CinAttachmentRow> TiedCinAttachmentModel::rows() const noexcept {
  return data_ ? ClassificationView<CinAttachmentRow>{data_->rows.get(),data_->count} : ClassificationView<CinAttachmentRow>{};
}
CinAttachmentForecast TiedCinAttachmentModel::forecast() const noexcept { return data_ ? data_->bytes : CinAttachmentForecast{}; }
}
