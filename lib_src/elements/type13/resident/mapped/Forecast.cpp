// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Startup.h"
#include "../../../ShellPhysicalOutputRanges.h"
#include "../../../ShellExecutionOutputRanges.h"

namespace tl::fea::type13::mapped {
BatchReport MakeForecast(const BatchConfig& config, const ShellPhysicalBinding& physical,
    const NodalRigidAssemblyBinding& rigid, const NodalCinWitnessSource& cin,
    BatchMappedLimits limits, std::size_t private_bytes, Forecast& output) noexcept {
  if (!limits.max_host_bytes || limits.max_host_bytes > BatchMappedLimits::Vehicle().max_host_bytes) {
    return {BatchStatus::ResourceLimit, "Mapped TYPE13 complete host cap is invalid"};
  }
  if (!physical.prepared() || !physical.coefficients()->type13() || !rigid.prepared() ||
      !rigid.coefficients()->Matches(*physical.coefficients()) ||
      !rigid.domain()->SharesStorage(*physical.domain()) ||
      config.assembly != BatchAssembly::CinNativeStiffness) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 requires the complete physical/rigid/CIN scope"};
  }
  const auto& source = *physical.coefficients()->type13();
  const auto checked = batch_detail::SourceGeometryPreflight(config, source, true);
  if (!checked) return checked;
  if (!source.domain()->SharesStorage(*physical.domain()) ||
      !cin.model || !cin.model->prepared() ||
      !cin.model->domain()->SharesStorage(*physical.domain()) ||
      !cin.range_count || cin.range_count != cin.model->rows().count ||
      cin.range_count > NodalCinLimits{}.max_attachments ||
      !cin.witness_count || cin.witness_count > NodalCinLimits{}.max_witnesses ||
      !cin.ranges || !cin.witnesses) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 source/CIN domain or roster extent differs"};
  }
  BatchForecast local;
  const auto local_budget = batch_detail::SourceForecast(config, source, private_bytes, local);
  if (!local_budget) return local_budget;
  Forecast next;
  if (config.owner.node_count > config.limits.max_nodes ||
      !batch_detail::MakeLayout(source.model()->property_count(), source.model()->connection_count(),
                               config.limits, next.device) ||
      !shell_physical_owner::ForecastProof(config.owner.node_count, cin.range_count,
                                          limits.max_host_bytes, next.proof)) {
    return {BatchStatus::ResourceLimit, "Mapped TYPE13 arena or proof exceeds its count/byte cap"};
  }
  // The optional execution handle already retains this actual rigid backing.
  // Equal values alone do not authorize discounting independently owned arrays.
  std::size_t rigid_bytes = rigid.owned_payload_bytes();
  const auto* execution = physical.execution();
  if (execution && rigid.groups().size() && rigid.members().size() &&
      execution->rigid()->groups().data() == rigid.groups().data() &&
      execution->rigid()->members().data() == rigid.members().data()) {
    rigid_bytes = 0;
  }
  util::BoundedArenaLayout host(limits.max_host_bytes);
  util::ArenaRegion ignored;
  // Exact source model/contribution backing is retained by the physical ledger.
  // A separate non-execution rigid handle is conservatively charged inclusively;
  // no nested ledger discount is inferred from numerically equivalent values.
  if (!host.Append<std::byte>(private_bytes + sizeof(State), ignored) ||
      !host.Append<std::byte>(physical.owned_payload_bytes(), ignored) ||
      !host.Append<std::byte>(rigid_bytes, ignored) ||
      !host.Append<std::byte>(next.device.bytes, ignored) ||
      !host.Append<Evaluation>(source.model()->connection_count(), ignored) ||
      !host.Append<std::byte>(next.proof.bytes, ignored)) {
    return {BatchStatus::ResourceLimit, "Mapped TYPE13 simultaneous retained/startup payload exceeds cap"};
  }
  next.host_bytes = host.bytes();
  output = next;
  return {};
}
} // namespace tl::fea::type13::mapped

namespace tl::fea::type13 {
BatchReport Batch::ForecastMapped(const BatchConfig& config, const ShellPhysicalBinding& physical,
    const NodalRigidAssemblyBinding& rigid, const NodalCinWitnessSource& cin,
    BatchForecast& output, BatchMappedLimits limits) noexcept {
  using trial_identity::Disjoint;
  if (!shell_physical_owner::OutputDisjoint(physical, &output, sizeof(output)) ||
      !shell_execution_detail::OutputDisjoint(rigid, &output, sizeof(output)) ||
      !Disjoint(&output, sizeof(output), &config, sizeof(config)) ||
      !Disjoint(&output, sizeof(output), &cin, sizeof(cin))) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 forecast output overlaps retained sources"};
  }
  mapped::Forecast next;
  const auto checked = mapped::MakeForecast(config, physical, rigid, cin, limits,
                                            sizeof(Batch) + sizeof(Impl), next);
  if (!checked) return checked;
  const auto rows = cin.model->rows();
  const auto* classification = cin.model->classification();
  const auto slaves = classification->slaves();
  const auto decode = classification->interface_decode();
  if (!Disjoint(&output, sizeof(output), cin.model, sizeof(*cin.model)) ||
      !Disjoint(&output, sizeof(output), rows.data, rows.count * sizeof(*rows.data)) ||
      !Disjoint(&output, sizeof(output), classification, sizeof(*classification)) ||
      !Disjoint(&output, sizeof(output), slaves.data, slaves.count * sizeof(*slaves.data)) ||
      !Disjoint(&output, sizeof(output), decode.data, decode.count * sizeof(*decode.data)) ||
      !Disjoint(&output, sizeof(output), cin.ranges,
                cin.range_count * sizeof(tl::constraints::tied_shell::cin::WitnessRange)) ||
      !Disjoint(&output, sizeof(output), cin.witnesses,
                cin.witness_count * sizeof(tl::constraints::tied_shell::cin::ActiveWitness))) {
    return {BatchStatus::InvalidInput, "Mapped TYPE13 forecast output overlaps CIN roster"};
  }
  output = {next.device.bytes, next.host_bytes};
  return {};
}
} // namespace tl::fea::type13
