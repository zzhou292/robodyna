#pragma once

#include "SourceContactForceFixture.h"
#include "collision/NodalWallContact.h"
#include "collision/PlanarWallBox.h"
#include <memory>

namespace crash::qualification::source_contact::nodal {
namespace sc=tlfea::contact;
namespace cf=force;
inline constexpr unsigned Workers=128,MaxFaces=400,MaxShares=4*ParentCount;
enum class Status { Ok,InvalidInput,PreflightFailure,OutsideWall,PointFailure,Accuracy,Nonfinite };
struct Report {
    Status status=Status::InvalidInput;
    sc::NodalWallReport point;
    unsigned node=UINT32_MAX,parent=UINT32_MAX;
};

// Immutable source preparation is host-only. Geometry references are initialized
// once from the authenticated original positions; prescribed configurations are
// separate. This is the chosen fixture mass, not source mechanics equivalence.
class PreparedSource {
  public:
    PreparedSource();
    ~PreparedSource();
    PreparedSource(const PreparedSource&)=delete;
    PreparedSource& operator=(const PreparedSource&)=delete;
    bool Initialize(const SourcePartContactFixture&,std::string& diagnostic);
    bool prepared() const noexcept;
    const sc::NodalWallWeights& weights() const;
    const cf::FixtureMass& mass() const;
    // Source-order selection for explicitly independent per-parent experiments.
    // Copies one owning reference measure into staged weights, without changing
    // the all-parent preparation or inventing a coherent whole-part motion.
    bool ParentWeights(unsigned source_parent,sc::NodalWallWeights*) const;
  private:
    struct Data;
    std::unique_ptr<const Data> data_;
};

// Bounded qualification packet: actual copied wall faces and all physical source
// indices. Host creation proves full endpoint sweep coverage; raw GPU endpoint
// lookup does not authenticate a caller-modified copy or prove coverage alone.
struct Input {
    sc::NodalWallParentWeight parents[ParentCount];
    sc::NodalWallNodeWeight nodes[NodeCount];
    unsigned first_share[NodeCount+1]{};
    unsigned share_slot[MaxShares]{}; // Slot=4*sorted_parent+native_local_node.
    sc::planar_detail::WallFace faces[MaxFaces];
    double position[3*NodeCount]{},velocity[3*NodeCount]{};
    double inverse_mass[NodeCount]{};
    std::uint8_t fixed[NodeCount]{};
    sc::NodalWallConfig config;
    double wall_tolerance=0;
    unsigned face_count=0,share_count=0;
    std::uint64_t base_epoch=0,attempt=0;
    bool prepared=false;
};
struct Result {
    sc::NodalWallResult contact;
    std::uint64_t wall_face[sc::MaxNodalWallNodes]{}; // Stable smallest-ID owner.
};

// All 94 parents are preflighted before any packet is published. The existing
// shared interval helper checks finite mass/position/velocity and fixed motion;
// fixed positive or unresolved penetration additionally rejects this model.
// Conservative box expansion handles zero projected spans without moving nodes.
// Caller output is unchanged on every failure, including outside/hole coverage.
Report BuildInput(const PreparedSource&,const cf::Coordinates& base,const cf::Coordinates& endpoint,
    const cf::Coordinates& velocity,const cf::FixtureMass&,const sc::PlanarWallGeometry&,
    std::uint64_t attempt,Input*,std::string& diagnostic);
Report EvaluateHost(const PreparedSource&,const Input&,Result*);
} // namespace crash::qualification::source_contact::nodal
