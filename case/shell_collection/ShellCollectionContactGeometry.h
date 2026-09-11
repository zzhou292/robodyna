#pragma once
#include "lib_src/elements/ShellBatchBinding.h"
#include "lib_src/assembly/ShellNodeMap.h"
#include "lib_src/collision/NodalWallContact.h"
#include "lib_src/collision/PlanarWallBox.h"
#include <array>
#include <memory>

namespace crash::cases {
struct ShellContactGeometryLimits {
    // Peak owned payload includes immutable binding, weights and temporary
    // native-area preparation. Allocator bookkeeping is outside this ledger.
    std::size_t max_startup_bytes=32*1024*1024;
    tlfea::contact::NodalWallWeightLimits weights{};
    static constexpr ShellContactGeometryLimits Vehicle() noexcept {
        return {2ULL*1024*1024*1024,tlfea::contact::NodalWallWeightLimits::Vehicle()};
    }
};
struct ShellContactParent {
    tl::fea::ShellBindingFamily family=tl::fea::ShellBindingFamily::None;
    std::size_t family_index=0;
    std::uint64_t source_id=0;
};
enum class ShellContactGeometryStatus { Ok,InvalidInput,ResourceLimit,ReferenceFailure,WeightFailure };
struct ShellContactGeometryReport {
    ShellContactGeometryStatus status=ShellContactGeometryStatus::InvalidInput;
    const char* message="Invalid shell contact geometry";
    ShellContactParent parent;
    explicit operator bool() const noexcept { return status==ShellContactGeometryStatus::Ok; }
};

// Source-neutral adapter from complete native shell binding to the existing
// reference-area nodal contact law. Area measures never create physical mass.
// Exact coordinates, source IDs and cyclic connectivity are retained. Sorted
// weight order is mapped explicitly to each original family index. No wall
// placement, force, owner, clock or trajectory admission belongs here.
class ShellCollectionContactGeometry {
  public:
    ShellCollectionContactGeometry();
    ~ShellCollectionContactGeometry();
    ShellCollectionContactGeometry(const ShellCollectionContactGeometry&)=delete;
    ShellCollectionContactGeometry& operator=(const ShellCollectionContactGeometry&)=delete;
    ShellContactGeometryReport Initialize(const tl::fea::ShellBatchBinding&,const ShellContactGeometryLimits& = {});
    // Exact domain coordinates and mapped QEPH/T3/QBAT connectivity. Extra
    // physical nodes receive no surface area and do not enlarge surface bounds.
    // This geometry operation does not admit constraints or a contact runtime.
    ShellContactGeometryReport InitializeMapped(const tl::fea::ShellNodeMap&,const ShellContactGeometryLimits& = {});
    bool prepared() const noexcept;
    const tl::fea::ShellBatchBinding* binding() const noexcept;
    const tl::fea::ShellNodeMap* mapping() const noexcept;
    const tlfea::contact::NodalWallWeights* weights() const noexcept;
    tlfea::contact::VectorView positions() const noexcept;
    std::array<tlfea::contact::Vec3,2> reference_bounds() const noexcept;
    // The declared world box must contain the full reference collection. Both
    // X endpoints are projected onto the actual finite mesh plane; its exposed
    // edges and holes remain checked by the existing TL coverage operation.
    tlfea::contact::PlanarContactReport CheckWallCoverage(const tlfea::contact::PlanarWallGeometry&,
        tlfea::contact::PlanarWallBox motion,double exposed_clearance,std::uint64_t binding_id,
        tlfea::contact::PlanarWallBoxCoverage*) const;
    const ShellContactParent* parent_from_weight(std::size_t) const noexcept;
    std::size_t startup_payload_bytes() const noexcept;
  private:
    struct Impl;
    ShellContactGeometryReport InitializeImpl(const tl::fea::ShellBatchBinding&,
        const tl::fea::ShellNodeMap*,const ShellContactGeometryLimits&);
    std::unique_ptr<const Impl> impl_;
};
} // namespace crash::cases
