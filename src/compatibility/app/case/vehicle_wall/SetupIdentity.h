#pragma once
#include <memory>
namespace crash::cases::vehicle_wall {
class VehicleWallSetup;
// Typed immutable backing authority. Copies retain the actual prepared setup;
// no numeric ID, settings comparison or caller-created tag can construct it.
class SetupIdentity {
  public:
    SetupIdentity()=default;
    explicit operator bool() const noexcept {return bool(backing_);}
    bool Matches(const SetupIdentity& other) const noexcept {return backing_==other.backing_;}
  private:
    explicit SetupIdentity(std::shared_ptr<const void> p):backing_(std::move(p)) {}
    std::shared_ptr<const void> backing_;
    friend class VehicleWallSetup;
};
} // namespace crash::cases::vehicle_wall
