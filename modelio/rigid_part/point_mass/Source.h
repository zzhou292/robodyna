#pragma once
#include "../RigidPartSource.h"

namespace crash::modelio::vehicle::rigid_part::point_mass {
struct Limits {
    std::size_t records=4096,blocks=16384,host_bytes=512*1024*1024;
};
struct Record {
    assembly::AuxiliaryPointMass value;
    std::size_t retained_source=SIZE_MAX;
};
struct Consumed {
    std::size_t record=SIZE_MAX,root_index=SIZE_MAX;
};
struct Data {
    std::vector<Record> records; // Every original main-member ELEMENT_MASS row.
    std::vector<Consumed> consumed; // Original row order, including every rigid member.
    std::size_t outside_records=0,startup_budget_bytes=0;
};
// Immutable original ELEMENT_MASS source selection for rigid PARTs. The native
// producer later converts value.supplied_mass_source using source units. This
// object supplies no global owner index, scalar J, rubber mass or source closure.
class Source {
  public:
    static Source Prepare(const RigidPartSource&,Limits={});
    static std::size_t Forecast(const RigidPartSource&,Limits={});
    Source(const Source&) noexcept=default;
    Source(Source&& other) noexcept:storage_(other.storage_) {}
    Source& operator=(const Source&)=delete;
    const RigidPartSource& rigid_source() const noexcept;
    const Data& data() const noexcept;
  private:
    struct Storage;
    explicit Source(std::shared_ptr<const Storage> value):storage_(std::move(value)) {}
    std::shared_ptr<const Storage> storage_;
};
} // namespace crash::modelio::vehicle::rigid_part::point_mass
