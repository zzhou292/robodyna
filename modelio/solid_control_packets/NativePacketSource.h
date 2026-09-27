#pragma once
#include "modelio/solid_source/VehicleSolidSource.h"
#include "modelio/solid_control/EffectiveSource.h"
#include "lib_src/elements/solids/control/Types.h"
#include <filesystem>
namespace crash::modelio::solid_control_packets {
namespace control=tl::fea::solids::control;
struct Artifact {
    std::filesystem::path path;
    std::size_t bytes=0;
    std::string sha256,case_profile;
};
struct Limits {
    std::size_t file_bytes=16u<<20,parents=16384,packets=16384,partitions=1024;
    std::size_t retained_bytes=8u<<20,startup_bytes=256u<<20;
};
struct Forecast {std::size_t parse_bytes=0,retained_bytes=0,startup_bytes=0;};
// Offline native packet import. Runtime never invokes a native solver/debugger.
// This joins an authenticated export to independent original solid/control
// authority. TL Model performs the final coefficient/domain schedule admission.
class NativePacketSource {
  public:
    static NativePacketSource Prepare(const solid_source::VehicleSolidSource&,
        const solid_control::EffectiveSource&,const Artifact&,Limits={});
    control::Input InputFor(std::uint64_t source_instance_id) const;
    const Artifact& artifact()const noexcept;
    const solid_source::VehicleSolidSource& solid_source()const noexcept;
    const Forecast& forecast()const noexcept;
    std::size_t controlled_count()const noexcept;
    std::size_t owned_payload_bytes()const noexcept;
  private:
    struct Data;
    explicit NativePacketSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
} // namespace crash::modelio::solid_control_packets
