// SPDX-License-Identifier: MIT
#pragma once
#include "SerialFlow.h"
#include "CandidateFlow.h"
#include "lib_src/elements/t3/T3Startup.h"
#include <gtest/gtest.h>
#include <vector>

namespace t3_readback_test {
namespace fe = tl::fea;
namespace t3 = fe::t3;
using Setup = fe::shell_batch_plasticity_detail::SetupStatus;
using Report = t3::BatchReport;
using Status = t3::BatchStatus;

struct Catalog {
  std::vector<fe::ShellSectionLaw> laws;
  t3::OnePointMaterial material;
  std::size_t absent_parameter = SIZE_MAX;
  bool Law(fe::ShellBindingFamily, std::size_t row, fe::ShellSectionLaw* out) const {
    if (row >= laws.size()) return false;
    *out = laws[row];
    return true;
  }
  bool Parameters(fe::ShellBindingFamily, std::size_t row, fe::sections::PointParameters* out) const {
    if (row == absent_parameter) return false;
    *out = material;
    return true;
  }
};
struct Binding {
  t3::ReferenceData reference;
  const auto& t3_reference(std::size_t) const { return reference; }
};
struct Physical {
  Catalog* source;
  Binding* binding;
  const Catalog* catalog() const { return source; }
  const Binding* shells() const { return binding; }
};
struct Histories {
  bool one_point = true, catalog_present = true;
  Catalog* source = nullptr;
  Setup fault = Setup::Success;
  std::vector<fe::ShellBatchLayeredSection> sections, input_sections;
  std::vector<fe::ShellBatchFailureState> failures;
  unsigned reads = 0;
  bool one_point_sections() const { return one_point; }
  const Catalog* section_catalog() const { return catalog_present ? source : nullptr; }
  const auto* section_staging() const { return sections.data(); }
  const auto* failure_staging() const { return failures.data(); }
  fe::shell_batch_plasticity_detail::SetupReport ReadSections(unsigned, std::size_t, cudaStream_t, double) {
    ++reads;
    if (fault != Setup::Success) return {fault, "section fault", cudaErrorInvalidValue};
    sections = input_sections;
    return {Setup::Success, "OK"};
  }
};
struct State {
  t3::T3BatchConfig config;
  fe::NodalStamp accepted_stamp;
  Catalog catalog;
  Binding binding;
  Physical mapped{&catalog, &binding};
  Physical* physical = &mapped;
  Binding* joined_binding = &binding;
  Histories histories;
  Histories* plasticity = &histories;
  struct Storage { int slab[2]{}; } backing;
  Storage* storage = &backing;
  cudaStream_t stream = nullptr;
  std::vector<t3::ForceTrial> device, staging;
  unsigned copies = 0, pending_checks = 0, fail_pending = 0, fail_copy = 0;
  bool poisoned = false;
  explicit State(std::size_t count = 7) {
    t3::ReferenceInput input;
    input.node_ids[0] = 101;
    input.node_ids[1] = 205;
    input.node_ids[2] = 333;
    input.position[0] = {0, 0, 0};
    input.position[1] = {1, 0, 0};
    input.position[2] = {.2, .8, 0};
    input.density = 1000;
    input.young_modulus = 250e6;
    input.poisson_ratio = .35;
    input.thickness = .0005;
    EXPECT_EQ(t3::InitializeReference(input, binding.reference), t3::Status::kSuccess);
    const tl::material::TabulatedShellPlasticityRate rate{
        true, 0, 1, 10000, tl::material::ShellPlasticityRatePolicy::FilteredZeroC};
    EXPECT_EQ(tl::material::PrepareLinearLaw44ShellPlasticity(250e6, .35, 1000,
        {10e6, 1e6}, rate, catalog.material), tl::material::TabulatedShellPlasticityStatus::Ok);
    config.element_count = count;
    config.owner.fixed_dt = 0x1p-15;
    histories.source = &catalog;
    device.resize(count);
    staging.resize(count);
    histories.failures.resize(count);
    using Law = fe::ShellSectionLaw;
    for (std::size_t row = 0; row < count; ++row) {
      const Law law[]{Law::Law44Nip1, Law::LayeredLaw1Nip3, Law::LayeredLaw44Nip3, Law::RigidSkin};
      catalog.laws.push_back(law[row % 4]);
      fe::ShellBatchOnePointSectionState point;
      point.point.reported_thickness_m = binding.reference.input.thickness;
      if (law[row % 4] == Law::Law44Nip1)
        histories.input_sections.push_back(fe::ShellBatchLayeredSection::OnePoint(point));
      else if (law[row % 4] == Law::LayeredLaw1Nip3)
        histories.input_sections.push_back(fe::ShellBatchLayeredSection::Elastic({}));
      else if (law[row % 4] == Law::LayeredLaw44Nip3)
        histories.input_sections.push_back(fe::ShellBatchLayeredSection::Plastic({}));
      else histories.input_sections.push_back(fe::ShellBatchLayeredSection::RigidSkin());
      EXPECT_EQ(t3::InitializeHistory(binding.reference, {0, 0}, device[row].proposed_history),
          t3::Status::kSuccess);
    }
  }
  unsigned AcceptedSlabIndex() const { return 0; }
  Report Runtime(cudaError_t error, const char* message) {
    if (error == cudaSuccess) return {Status::Success, "OK"};
    poisoned = true;
    return {Status::DeviceFailure, message};
  }
  Report PendingError() {
    ++pending_checks;
    return Runtime(poisoned || pending_checks == fail_pending ? cudaErrorInvalidValue : cudaSuccess,
        "pending fault");
  }
  Report ReadResults(const int* slab) {
    auto report = PendingError();
    if (report.status != Status::Success) return report;
    const unsigned selected = slab == &storage->slab[0] ? 0 : 1;
    if (slab != &storage->slab[selected]) return {Status::InvalidInput, "Unknown T3 result slab"};
    ++copies;
    if (copies == fail_copy) return Runtime(cudaErrorInvalidValue, "copy fault");
    staging = device; // Transport seam only; complete mapped checker below is frozen source.
    return serial::ValidateMappedResults(*this, selected);
  }
  Report ValidateMappedSections(unsigned slab) { return serial::ValidateMappedSections(*this, slab); }
  Report ValidateOnePointReadback(unsigned slab, double time, std::uint64_t epoch) {
    return serial::OnePoint(*this, slab, time, epoch);
  }
};
inline void Same(const Report& a, const Report& b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_STREQ(a.message, b.message);
  EXPECT_EQ(a.element, b.element);
}
} // namespace t3_readback_test
