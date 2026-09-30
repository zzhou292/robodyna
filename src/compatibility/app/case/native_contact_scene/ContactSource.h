#pragma once
#include "PhysicalSource.h"
#include "lib_src/collision/radioss_type25/runtime/Types.h"
#include "lib_src/collision/RadiossType25SearchStartup.h"
namespace crash::cases::native_scene {
namespace native=tlfea::contact::radioss_type25;
struct ContactIdentity {std::uint64_t source=0,topology=0,generation=0;};
struct ContactLimits {
    std::size_t nodes=2048,physical_shells=1024;
    std::size_t host_bytes=128u<<20,scratch_bytes=64u<<20;
    native::search_startup::Initialization preprocessing=native::search_startup::Initialization::InvariantNoExpansion;
};
struct ContactForecast {
    // Incremental contact-source buffers, excluding the shared PhysicalSource
    // handle's existing storage and excluding any runtime/device owner.
    std::size_t owned_bytes=0,startup_scratch_bytes=0,peak_bytes=0;
    std::size_t native_model_nodes=0; // Source geometry plus generated rigid primaries, not owner node count.
};
// The explicit declared fixed-wall/moving-patch compiler profile. This is not a
// general native deck parser. Produces all numerical contact startup fields via
// TL's source/topology/search producers; no observed K/gap/normal arrays enter.
// Copies retain immutable storage. The returned views survive while any copy
// lives; runtime Initialize copies its own bounded source storage as usual.
class ContactSource {
  public:
    static ContactSource Prepare(const PhysicalSource&,ContactIdentity,ContactLimits={});
    const PhysicalSource& physical_source() const noexcept;
    const native::FixedMainSource& source() const noexcept;
    const native::TransactionConfig& config() const noexcept;
    const ContactForecast& forecast() const noexcept;
    native::search_startup::Initialization preprocessing() const noexcept;
  private:
    struct Data;std::shared_ptr<const Data> data_;
    explicit ContactSource(std::shared_ptr<const Data> p):data_(std::move(p)){}
};
}
