#pragma once
#include "SourceAssemblyData.h"

namespace crash::modelio::assembly {
enum class Law1DriverStatus { Unavailable, NotElastic, NativeA62Type1 };
// Derived from reader-authenticated declarations, never a claim about an
// arbitrary Engine deck. The native default/ordinary execution requirements
// are retained separately in Law1ExecutionPolicy.
class SourceLaw1Driver {
  public:
    Law1DriverStatus status() const noexcept { return status_; }
    SourceId material_id() const noexcept { return material_; }
    SourceId section_id() const noexcept { return section_; }
    unsigned source_elform() const noexcept { return elform_; }
    unsigned raw_nip() const noexcept { return available() ? 3 : 0; }
    int resolved_npt() const noexcept { return available() ? 0 : -1; }
    int ismstr() const noexcept { return available() ? 2 : -1; }
    int ithick() const noexcept { return available() ? 1 : -1; }
    int iplas() const noexcept { return available() ? 1 : -1; }
    bool available() const noexcept { return status_ == Law1DriverStatus::NativeA62Type1; }
    static constexpr const char* revision() noexcept {
        return "a62b27e6baa555d222a580d6218867d0be4d70b5";
    }
    SourceLaw1Driver() = default;
  private:
    Law1DriverStatus status_ = Law1DriverStatus::Unavailable;
    SourceId material_ = 0;
    SourceId section_ = 0;
    unsigned elform_ = 0;
    friend SourceLaw1Driver ResolveLaw1SourceDriver(const Material&, const Section&) noexcept;
};
// Value consistency check. SourceAssembly and VehicleSourcePlan supply its
// authenticated immutable inputs; calling this on arbitrary values supplies
// no source authority. No copied keyword parser or constitutive math.
SourceLaw1Driver ResolveLaw1SourceDriver(const Material&, const Section&) noexcept;
}
