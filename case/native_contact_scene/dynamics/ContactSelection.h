#pragma once
#include "../MovingContactSource.h"
#include <utility>
#include <variant>
namespace crash::cases::native_scene {
// Closed immutable choice of already compiled source profiles. Dispatch occurs
// during startup; every physical interval uses the same native Transaction.
class ContactSelection {
  public:
    enum class Kind { FixedWall, MovingShells };
    ContactSelection(const ContactSource& source):value_(source){}
    ContactSelection(const MovingContactSource& source):value_(source){}
    static ContactSelection Prepare(const PhysicalSource& physical,ContactIdentity identity,ContactLimits limits={});
    // Preserves the concrete checked source type at the TL initializer. No
    // untyped flag or common descriptor grants moving physical admission.
    template<class Visitor> decltype(auto) Visit(Visitor&& visitor) const {
        return std::visit(std::forward<Visitor>(visitor),value_);
    }
    Kind kind() const noexcept {return std::holds_alternative<ContactSource>(value_)?Kind::FixedWall:Kind::MovingShells;}
    const char* profile_name() const noexcept {
        if(physical_source().declared().data().rigid_patch)
            return "ordinary_rigid_patch_moving_shells_accepted_owner_mass_native_activation_local_single_worker";
        return kind()==Kind::FixedWall?"ordinary_fixed_main_all_active_ready_normals_local_single_worker":
            "ordinary_moving_shells_starter_cache_native_activation_local_single_worker";
    }
    const PhysicalSource& physical_source() const noexcept {
        return Visit([](const auto& source)->const PhysicalSource& {return source.physical_source();});
    }
    const native::ContactSourceInput& source() const noexcept {
        return Visit([](const auto& source)->const native::ContactSourceInput& {return source.source();});
    }
    const native::TransactionConfig& config() const noexcept {
        return Visit([](const auto& source)->const native::TransactionConfig& {return source.config();});
    }
    const ContactForecast& forecast() const noexcept {
        return Visit([](const auto& source)->const ContactForecast& {return source.forecast();});
    }
    native::search_startup::Initialization preprocessing() const noexcept {
        return Visit([](const auto& source) {return source.preprocessing();});
    }
  private:
    std::variant<ContactSource,MovingContactSource> value_;
};
}
