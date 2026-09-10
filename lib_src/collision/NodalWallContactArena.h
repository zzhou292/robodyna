#pragma once
#include "NodalWallContactStorage.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::nodal_wall_device_detail {
using tl::util::ArenaRegion;
struct ResultRegions { ArenaRegion parents,nodes,wall_face; };
struct ArenaLayout {
  ArenaRegion header,parents,nodes,incident_offsets,incident_slots,positions,inverse,fixed,status,shares,force,error;
  ResultRegions base,result;
  std::size_t bytes=0;
  std::size_t parent_count=0,node_count=0,global_count=0;
  NodalWallDeviceProfile profile=NodalWallDeviceProfile::Legacy;
};
// Counts and all aligned byte extents are admitted before borrowed reads.
bool BuildArenaLayout(std::size_t parents,std::size_t nodes,std::size_t global_nodes,
    std::size_t cap,ArenaLayout&,NodalWallDeviceProfile=NodalWallDeviceProfile::Legacy) noexcept;
std::size_t HostPreparationBytes(const ArenaLayout&) noexcept;
class PreparedModel {
 public:
  PreparedModel()=default;
  PreparedModel(PreparedModel&&) noexcept=default;
  PreparedModel& operator=(PreparedModel&&) noexcept=default;
  PreparedModel(const PreparedModel&)=delete;
  PreparedModel& operator=(const PreparedModel&)=delete;
  bool Initialize(const ArenaLayout&);
  bool prepared() const noexcept { return arena_ && model().prepared; }
  Model& model() noexcept { return storage().model; }
  const Model& model() const noexcept { return storage().model; }
  Storage& storage() noexcept { return *tl::util::ArenaPointer<Storage>(arena_->data(),layout_.header); }
  const Storage& storage() const noexcept { return *tl::util::ArenaPointer<Storage>(arena_->data(),layout_.header); }
  const ArenaLayout& layout() const noexcept { return layout_; }
  const void* data() const noexcept { return arena_?arena_->data():nullptr; }
  // Returns a HOST value whose pointer fields target the supplied device arena.
  // Callers never read pointer fields by dereferencing the device header.
  Storage Rebase(void* device_base) const noexcept;
 private:
  ArenaLayout layout_;
  std::unique_ptr<tl::util::HostArena> arena_;
};
} // namespace tlfea::contact::nodal_wall_device_detail
