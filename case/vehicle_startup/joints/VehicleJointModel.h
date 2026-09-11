#pragma once
#include "../physical_model/VehiclePhysicalModel.h"
#include "modelio/type45/VehicleType45Source.h"
#include "lib_src/elements/type45/Model.h"

namespace crash::cases::vehicle_startup::joints {
using Physical=physical_model::VehiclePhysicalModel;
using Source=modelio::type45::VehicleType45Source;
struct Limits {
    std::size_t host_bytes=std::size_t{8}<<30;
    tl::fea::type45::ModelLimits model;
};
struct Forecast {
    std::size_t physical_reservation=0,source_reservation=0,model_reservation=0;
    std::size_t packing_bytes=0,total_bytes=0;
};
// Immutable connection between original retained joint cards and the actual
// physical rigid/domain authority. Boundary rows stay in source(); they are
// excluded explicitly from the 38 force operators. No auto-K, owner or clock.
class VehicleJointModel {
  public:
    static Forecast Preflight(const Physical&,const Source&,Limits={});
    static VehicleJointModel Prepare(const Physical&,const Source&,Limits={});
    const Physical& physical() const noexcept;
    const Source& source() const noexcept;
    const tl::fea::type45::Model& model() const noexcept;
    const std::vector<std::uint32_t>& source_rows() const noexcept;
    const Forecast& forecast() const noexcept;
    // Additional retained payload when physical() is already owned by the
    // caller. Includes this wrapper, original rows and native joint model;
    // excludes only the exact shared physical/domain/rigid backing.
    std::size_t additional_owned_payload_bytes() const;
  private:
    struct Storage;
    explicit VehicleJointModel(std::shared_ptr<const Storage> value):storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::cases::vehicle_startup::joints
