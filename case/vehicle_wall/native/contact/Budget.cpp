#include "Internal.h"
#include "lib_utils/BoundedArena.h"
#include <algorithm>
namespace crash::cases::vehicle_wall::native::wall_interface::detail {
namespace {
std::size_t Sum(std::size_t a,std::size_t b) {
    if(b>SIZE_MAX-a)Reject(Status::ResourceLimit,"Finite-wall source forecast overflows");return a+b;
}
}
Plan Budget(const EnvelopeOwnerSource& owner,const VehicleSource& vehicle,Declaration declaration,Limits limits) {
    Plan plan;plan.shape=Check(owner,vehicle,declaration,limits);
    auto& f=plan.forecast;const auto nodes=plan.shape.nodes;
    // This is an owning published bound, not a private witness/joint formula.
    f.owner_retained=owner.retained_host_upper_bound(limits.coexistence_bytes);
    f.owner_prior_peak=owner.forecast().peak_bytes;
    f.vehicle_retained=vehicle.forecast().retained_bytes;f.vehicle_prior_peak=vehicle.forecast().peak_bytes;
    plan.topology_limits.max_nodes=limits.nodes;plan.topology_limits.max_primary_faces=1;
    plan.topology_limits.max_output_bytes=limits.own_bytes;plan.topology_limits.max_scratch_bytes=limits.own_bytes;
    plan.topology=s::Preflight(nodes,1,plan.topology_limits);
    if(plan.topology.status!=s::Status::Ok)Reject(Status::ResourceLimit,"Wall topology storage forecast rejected");
    tl::util::BoundedArenaLayout retained(limits.own_bytes);tl::util::ArenaRegion unused;
    // Each vector's actual capacity is checked; twice its exact count is an
    // explicit upper reservation, not an assumed allocator behavior.
    if(!retained.Append<std::uint64_t>(2*nodes,unused) || !retained.Append<double>(2*7*nodes,unused) ||
        !retained.Append<n::lifecycle::Node>(2*nodes,unused) || !retained.Append<std::uint32_t>(2*nodes,unused) ||
        !retained.Append<std::byte>(plan.topology.output_bytes,unused) ||
        !retained.Append<std::byte>(plan.topology.ready_output_bytes,unused) ||
        !retained.Append<std::byte>(64u<<10,unused))
        Reject(Status::ResourceLimit,"Finite-wall retained output exceeds its component cap");
    f.own_retained=retained.bytes();
    f.constraint_packing=vehicle_runtime::detail::PackingBytes(nodes,limits.own_bytes);
    f.coefficient_scratch=1u<<20;
    f.topology_scratch=std::max(plan.topology.scratch_bytes,plan.topology.ready_scratch_bytes);
    plan.gap_limits=WallGapLimits(limits);
    f.gap_scratch=plan.gap_limits.scratch_bytes; // Enforced public leaf ceiling before packing real shells.
    if(plan.shape.shells>SIZE_MAX/(2*sizeof(n::source_gaps::PhysicalShell)))
        Reject(Status::ResourceLimit,"Wall gap roster reservation overflows");
    f.gap_shell_copy=2*plan.shape.shells*sizeof(n::source_gaps::PhysicalShell);
    f.digest_scratch=(4u<<20)+16*limits.metadata_bytes;
    const auto temporary=std::max({f.constraint_packing,f.coefficient_scratch,f.topology_scratch,
        Sum(f.gap_scratch,f.gap_shell_copy),f.digest_scratch});
    f.own_peak=Sum(f.own_retained,temporary);
    if(f.own_peak>limits.own_bytes)Reject(Status::ResourceLimit,"Finite-wall source component peak exceeds cap");
    f.coexistence_peak=Sum(Sum(f.owner_retained,f.vehicle_retained),f.own_peak);
    // Report either possible input-construction order conservatively. Actual
    // case ownership may discount only separately proved exact shared backing.
    f.complete_construction_bound=std::max({f.coexistence_peak,Sum(f.owner_prior_peak,f.vehicle_retained),
        Sum(f.vehicle_prior_peak,f.owner_retained)});
    if(f.complete_construction_bound>limits.coexistence_bytes)
        Reject(Status::ResourceLimit,"Complete wall/source coexistence exceeds existing owner-source allowance");
    return plan;
}
}
