#pragma once
#include "VehicleConnectivity.h"
#include "Components.h"
#include "output/ArtifactIO.h"
#include "lib_utils/BoundedArena.h"

namespace crash::cases::vehicle_startup::connectivity::detail {
using output::Require;
Counts Extents(const VehiclePhysicalAttachments&);
Forecast Budget(const VehiclePhysicalAttachments&, const Counts&, Limits, std::size_t fixed);
class Relations {
  public:
    Relations(Data&, const Counts&);
    void Append(Kind, Role, std::uint64_t id, std::uint64_t part, std::size_t source_row,
                const std::size_t* slots, std::size_t count, std::size_t rigid_root = SIZE_MAX);
    void Complete() const;
  private:
    Data& data_;
    Counts expected_;
};
void VisitShells(const VehiclePhysicalAttachments&, Relations&);
void VisitElements(const VehiclePhysicalAttachments&, Relations&);
void VisitConstraints(const VehiclePhysicalAttachments&, Relations&);
} // namespace crash::cases::vehicle_startup::connectivity::detail
