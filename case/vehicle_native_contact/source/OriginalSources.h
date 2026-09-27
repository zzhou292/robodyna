#pragma once
#include "../SourceAdmission.h"
#include "case/vehicle_self_contact/native/InitializerControlsSource.h"
#include "case/vehicle_run/source/OriginalPaths.h"
#include "modelio/solid_control_packets/NativePacketSource.h"
namespace crash::cases::vehicle_native_contact::source {
using Owner=vehicle_wall::native::EnvelopeOwnerSource;
using Self=vehicle_self_contact::native::mixed_starter::MixedStarterSource;
using Wall=vehicle_wall::native::wall_interface::FiniteWallContactSource;
using Controls=vehicle_self_contact::native::initial_controls::InitializerControlsSource;
struct Limits {
    std::size_t host_bytes=std::size_t{18}<<30;
    std::size_t member_bytes=64u<<20;
};
struct Forecast {
    std::size_t member_storage_bytes=0,packet_authority_reservation=0,metadata_bytes=0;
    std::size_t input_peak=0,owner_peak=0,contact_peak=0,retained_bytes=0,peak_bytes=0;
    vehicle_native_contact::detail::SourceAdmission sources;
};
// One owned original-member/canonical authority and the explicit native V6
// source graph. Preparation includes the existing CUDA tied search, but creates
// no mechanical owner, force history, physical clock or contact transaction.
class OriginalSources {
  public:
    static OriginalSources Prepare(const vehicle_run::OriginalPaths&,
        const modelio::solid_control_packets::Artifact&,Limits={});
    const Owner& owner()const noexcept;
    const Self& self()const noexcept;
    const Wall& wall()const noexcept;
    const Controls& controls()const noexcept;
    const modelio::solid_control_packets::NativePacketSource& packets()const noexcept;
    vehicle_native_contact::detail::SourceInputs inputs()const noexcept;
    const Forecast& forecast()const noexcept;
  private:
    struct Data;
    explicit OriginalSources(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::cases::vehicle_native_contact::source
