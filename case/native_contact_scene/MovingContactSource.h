#pragma once
#include "ContactSource.h"
namespace crash::cases::native_scene {
// Explicit v2 all-shell source. Initial normals remain the genuine Starter
// cache; the native moving transaction owns subsequent activation and updates.
class MovingContactSource {
  public:
    static MovingContactSource Prepare(const PhysicalSource&,ContactIdentity,ContactLimits={});
    const PhysicalSource& physical_source() const noexcept;
    const native::MovingMainSource& source() const noexcept;
    const native::TransactionConfig& config() const noexcept;
    const ContactForecast& forecast() const noexcept;
    native::search_startup::Initialization preprocessing() const noexcept;
  private:
    struct Data;std::shared_ptr<const Data> data_;
    explicit MovingContactSource(std::shared_ptr<const Data> p):data_(std::move(p)){}
};
}
