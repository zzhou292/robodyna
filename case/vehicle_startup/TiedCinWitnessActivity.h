#pragma once
#include "TiedCinWitnessRoster.h"
#include "lib_src/elements/ShellBatchPublication.h"
#include "lib_src/solvers/NodalCinRuntime.h"

namespace crash::cases::vehicle_startup {
enum class TiedCinActivityStatus { Success, PendingPositiveShellWitness, InvalidInput,
    StaleOwner, ReadbackFailure, DeviceFailure };
struct TiedCinActivityReport {
    TiedCinActivityStatus status = TiedCinActivityStatus::InvalidInput;
    const char* message = "Invalid CIN witness activity request";
    std::size_t row = SIZE_MAX;
};
struct TiedCinActivityLimits { std::size_t workspace_bytes = 128u*1024*1024; };
struct TiedCinActivityForecast {
    std::size_t retained_roster_bound = 0, workspace_bytes = 0, total_host_bytes = 0;
};
// Reusable host readback workspace. Accepted activity can only be captured from
// the actual common owner/attached complete native families. No byte-input
// factory, declaration-derived startup flag, stiffness producer or new clock.
// Calls are serialized with the owner. Supplied owner/participants must remain
// alive through each call. A CUDA copy failure poisons this adapter; no recovery
// or subsequent upload is admitted. Accepted native state is never modified.
class TiedCinWitnessActivity {
  public:
    static TiedCinActivityForecast Forecast(const TiedCinWitnessRoster&,TiedCinActivityLimits = {});
    static std::unique_ptr<TiedCinWitnessActivity> Create(const TiedCinWitnessRoster&,TiedCinActivityLimits = {});
    ~TiedCinWitnessActivity();
    TiedCinWitnessActivity(const TiedCinWitnessActivity&) = delete;
    TiedCinWitnessActivity& operator=(const TiedCinWitnessActivity&) = delete;
    TiedCinActivityReport CaptureAccepted(const tl::fea::FENodalState&,const tl::fea::ShellBatchPublication&,
                                         const tl::fea::ShellFormulationParticipants&);
    // Requires a previously captured accepted endpoint and the exact CIN
    // source retained by this owner. Any failure discards the attempted nodal
    // assembly; accepted fields/previous snapshot remain unchanged. This only
    // fills the activity channel; it grants no force/coefficient admission.
    TiedCinActivityReport UploadAttempt(tl::fea::FENodalState&,const tl::fea::NodalTrialToken&);
    // Borrowed views remain valid until the next successful capture or destruction.
    bool has_accepted_activity() const noexcept;
    const tl::fea::NodalStamp& accepted_stamp() const noexcept;
    native_search::ClassificationView<std::uint8_t> accepted_flags() const noexcept;
    const TiedCinActivityForecast& forecast() const noexcept;
  private:
    struct Impl;
    explicit TiedCinWitnessActivity(std::unique_ptr<Impl>);
    std::unique_ptr<Impl> impl_;
};
}
