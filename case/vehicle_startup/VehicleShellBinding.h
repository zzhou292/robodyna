#pragma once
#include "VehicleShellReferences.h"
#include "lib_src/elements/ShellBatchBinding.h"

namespace crash::cases::vehicle_startup {
struct VehicleShellBindingLimits {
    std::size_t host_bytes=std::size_t{2}*1024*1024*1024;
    tl::fea::ShellHostBindingLimits native=tl::fea::ShellHostBindingLimits::Vehicle();
};
struct VehicleShellBindingForecast {
    std::size_t source_reference_bound=0, native_owned_reservation=0, native_scratch_reservation=0;
    std::size_t input_bytes=0, mapping_bytes=0, decode_bytes=0, decode_temporary_bytes=0;
    std::size_t fixed_bytes=0, total_bytes=0;
};
VehicleShellBindingForecast ForecastShellBinding(const VehicleShellReferences&,
                                                VehicleShellBindingLimits={});
// Complete shell-local geometry/mass only. The retained source rows preserve
// constitutive/rigid roles and actual family indices; no role is a force policy.
// Extra mechanical nodes, coefficients, DOFs and runtime admission are separate.
class VehicleShellBinding {
  public:
    static VehicleShellBinding Prepare(const VehicleShellReferences&,VehicleShellBindingLimits={});
    VehicleShellBinding(const VehicleShellBinding&) noexcept=default;
    VehicleShellBinding(VehicleShellBinding&& other) noexcept:data_(other.data_) {}
    VehicleShellBinding& operator=(const VehicleShellBinding&)=delete;
    const VehicleShellReferences& references() const noexcept;
    const tl::fea::ShellBatchBinding& shells() const noexcept;
    const VehicleShellBindingForecast& forecast() const noexcept;
  private:
    friend VehicleShellBindingForecast ForecastShellBinding(const VehicleShellReferences&,VehicleShellBindingLimits);
    struct Data;
    explicit VehicleShellBinding(std::shared_ptr<const Data> data):data_(std::move(data)) {}
    std::shared_ptr<const Data> data_;
};
}
