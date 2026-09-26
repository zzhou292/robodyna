#pragma once
#include "Types.h"
#include "case/CanonicalWall.h"
namespace crash::cases::vehicle_wall::native {
struct Preparation;
// Immutable declared input/domain supplement. No physical owner, final contact
// NSV/main arrays, accepted state or runtime-ready authority exists here.
class WallSource {
  public:
    // Exact immutable backing identity; no geometry/hash equivalence claim.
    bool SharesStorage(const WallSource& other) const noexcept { return data_ == other.data_; }
    static Forecast Preflight(const modelio::physical_domain::VehiclePhysicalDomain&,
        const modelio::native_spring_ids::ImportMembers&,Limits={});
    static Preparation Prepare(const modelio::physical_domain::VehiclePhysicalDomain&,
        const modelio::native_spring_ids::ImportMembers&,const case_data::CanonicalWall&,
        const std::string& authenticated_wall_bytes,const Declaration&,Limits={});
    const modelio::physical_domain::VehiclePhysicalDomain& vehicle_origin() const noexcept;
    const tl::fea::NodalNodeDomain& domain() const noexcept;
    const VehiclePrefix& vehicle_prefix() const noexcept;
    const Declaration& declaration() const noexcept;
    const Geometry& geometry() const noexcept;
    const AllocatedIds& ids() const noexcept;
    const NamespaceReport& namespace_report() const noexcept;
    const tl::fea::ShellBatchStartup& startup() const noexcept;
    // Wall-only constraint contribution over the fresh full domain. Zero vehicle
    // prefix is not proof of absent CIN/rigid or other original constraints; the
    // later physical context must retain/OR genuine vehicle constraint sources.
    tl::util::ConstView<std::uint8_t> translation_fixed_bits() const noexcept;
    tl::util::ConstView<std::uint8_t> rotation_fixed() const noexcept;
    const std::string& digest() const noexcept;
    const Forecast& forecast() const noexcept;
  private:
    struct Data;
    explicit WallSource(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
struct Preparation {Report report;std::optional<WallSource> source;};
} // namespace crash::cases::vehicle_wall::native
