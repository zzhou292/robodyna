#pragma once
#include "case/vehicle_wall/native/EnvelopeOwnerSource.h"
#include "lib_src/collision/radioss_type25/tied_removal/Types.h"
namespace crash::cases::vehicle_self_contact::native::initial_controls { class InitializerControlsSource; }
namespace crash::cases::vehicle_native_contact {
namespace tied_removal = tlfea::contact::radioss_type25::tied_removal;
struct TiedRemovalLimits {
    std::size_t host_bytes = 64u << 20;
    std::size_t mains = 524288, rows = 65536, canonical_nodes = 1048576;
};
struct TiedRemovalForecast {
    std::size_t mains = 0, rows = 0, canonical_nodes = 0;
    // Incremental payload only. The case separately charges the exact shared
    // complete owner/attachment source graph retained by this adapter.
    std::size_t retained_bytes = 0, temporary_bytes = 0, peak_bytes = 0;
};
class TiedRemovalSource {
  public:
    using Controls = vehicle_self_contact::native::initial_controls::InitializerControlsSource;
    static TiedRemovalForecast Preflight(const vehicle_wall::native::EnvelopeOwnerSource&,
        const Controls&, TiedRemovalLimits = {});
    static TiedRemovalSource Prepare(const vehicle_wall::native::EnvelopeOwnerSource&,
        const Controls&, TiedRemovalLimits = {});
    const vehicle_startup::TiedCinAttachments& attachments() const noexcept;
    tl::util::ConstView<tied_removal::Interface> interfaces() const noexcept;
    const TiedRemovalForecast& forecast() const noexcept;
  private:
    struct Data;
    explicit TiedRemovalSource(std::shared_ptr<const Data> data) : data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_native_contact
