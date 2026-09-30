#pragma once
#include "Values.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
#include "lib_src/collision/radioss_type25/UnitConversions.h"
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
struct Failure {Report report;};
[[noreturn]] inline void Reject(Status status,const char* reason) {throw Failure{{status,reason}};}
struct Shape {
    std::size_t nodes=0, vehicle_nodes=0, shells=0, quads=0;
    std::array<std::uint32_t,4> wall_nodes{};
    n::UnitScale units;
};
struct Plan {
    Shape shape;
    Forecast forecast;
    s::Forecast topology;
    s::Limits topology_limits;
    n::source_gaps::Limits gap_limits;
};
struct Fields {
    std::vector<std::uint64_t> ids;
    std::vector<double> positions,global_k,secondary_k,secondary_gap;
    std::vector<n::lifecycle::Node> nodes;
    std::vector<std::uint32_t> nsv;
    std::array<std::uint32_t,4> msr{};
    std::array<double,4> main_node_gaps{};
    std::array<double,2> main_k{};
    std::array<n::source_gaps::MainGapFields,2> main_gaps{};
    s::PrimaryFace primary;
    tl::util::HostArena topology_arena,ready_arena;
    s::Snapshot starter;
    s::FixedMainView ready;
    n::source_gaps::Report gaps;
    std::size_t capacity_bytes() const noexcept;
    s::Input Mesh(Declaration,n::UnitScale) const noexcept;
};
Shape Check(const EnvelopeOwnerSource&,const VehicleSource&,Declaration,Limits);
Plan Budget(const EnvelopeOwnerSource&,const VehicleSource&,Declaration,Limits);
void Allocate(Fields&,const Plan&);
void PackNodes(Fields&,const EnvelopeOwnerSource&,const VehicleSource&,const Plan&);
void BuildTopology(Fields&,const WallSource&,Declaration,const Plan&);
void BuildGaps(Fields&,const VehicleSource&,const Component&,const Controls&,const Plan&);
std::string Digest(const Fields&,const Controls&,const Provenance&,Declaration,std::size_t);
}
