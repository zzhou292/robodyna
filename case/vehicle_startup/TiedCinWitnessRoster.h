#pragma once
#include "TiedCinAttachments.h"
#include "VehicleShellBinding.h"
#include "lib_src/constraints/tied_shell/runtime/CinStageTypes.h"

namespace crash::cases::vehicle_startup {
namespace cin_stage = tl::constraints::tied_shell::cin;
struct TiedCinWitnessLimits {
    std::size_t host_bytes = std::size_t{2}*1024*1024*1024;
    std::size_t witnesses = 262144;
};
struct TiedCinWitnessOrigin {
    std::uint64_t source_part_id = 0;
    std::array<std::uint64_t,4> source_node_ids{};
    std::uint32_t source_parent_row = 0, canonical_parent = 0;
    tl::fea::ShellBindingFamily family = tl::fea::ShellBindingFamily::None;
};
struct TiedCinWitnessCounts {
    std::size_t source_parents = 0, attachments = 0, witnesses = 0, maximum_per_row = 0;
    std::size_t rows_without_shell_witness = 0, missing_domain_slots = 0;
    std::size_t declared_parent_witnesses = 0, additional_containing_parents = 0;
};
struct TiedCinWitnessForecast {
    // Conservative retained bounds from existing immutable producers; their
    // already-retired parsing scratch may be included. No RSS claim is made.
    std::size_t retained_source_bound = 0, incidence_scratch_bytes = 0;
    std::size_t roster_capacity_bytes = 0, fixed_bytes = 0, total_host_bytes = 0;
};
struct TiedCinWitnessData {
    std::vector<cin_stage::WitnessRange> ranges;
    std::vector<cin_stage::ActiveWitness> witnesses;
    std::vector<TiedCinWitnessOrigin> origins;
    TiedCinWitnessCounts counts;
};
// Sufficient shell-only positive-witness roster. Native CHK can also use solid
// faces: absence here is pending, never evidence of release. This handle owns
// no activity, solver state, stiffness, coefficients or current geometry.
class TiedCinWitnessRoster {
  public:
    static TiedCinWitnessForecast Forecast(const TiedCinAttachments&,const VehicleShellBinding&,
                                           TiedCinWitnessLimits = {});
    static TiedCinWitnessRoster Prepare(const TiedCinAttachments&,const VehicleShellBinding&,
                                        TiedCinWitnessLimits = {});
    TiedCinWitnessRoster(const TiedCinWitnessRoster&) noexcept = default;
    TiedCinWitnessRoster(TiedCinWitnessRoster&& other) noexcept : data_(other.data_) {}
    TiedCinWitnessRoster& operator=(const TiedCinWitnessRoster&) = delete;
    const TiedCinAttachments& attachments() const noexcept;
    const VehicleShellBinding& binding() const noexcept;
    const TiedCinWitnessData& data() const noexcept;
    const TiedCinWitnessForecast& forecast() const noexcept;
    // Existing owner profile: complete mapped shell witnesses, 1..4 per row.
    // False is a pending composition obligation, never a release instruction.
    bool runtime_mappable() const noexcept;
  private:
    struct Data;
    explicit TiedCinWitnessRoster(std::shared_ptr<const Data> value) : data_(std::move(value)) {}
    std::shared_ptr<const Data> data_;
};
}
