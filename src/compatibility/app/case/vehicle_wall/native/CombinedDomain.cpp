#include "Internal.h"
#include "lib_src/math/ScalarBits.h"
namespace crash::cases::vehicle_wall::native::detail {
// These masks are the wall constraint contribution: zero on the vehicle prefix.
// Current V5 packing declares zero world-fixed bits; future composition must
// still retain/OR genuine vehicle constraints. CIN/rigid membership stays in
// the vehicle origin handle and is not replaced by these mask values.

DomainValues BuildDomain(const tl::fea::NodalNodeDomain& vehicle,const AllocatedIds& ids,
        const Geometry& geometry,Limits limits) {
    const auto count=vehicle.node_count();
    const auto hard=tl::fea::NodalDomainLimits::Vehicle();
    if(!count||count>hard.max_nodes-4)Reject(Status::ResourceLimit,"Combined domain node cap exceeded");
    std::vector<tl::fea::NodalDomainNode> nodes;nodes.reserve(count+4);
    for(const auto& node:vehicle.nodes())nodes.push_back(node);
    for(unsigned i=0;i<4;++i)nodes.push_back({ids.nodes[i],geometry.reference_m[i]});
    if(nodes.capacity()>2*(count+4))Reject(Status::ResourceLimit,"Combined input packing exceeded reservation");
    auto cap=hard;cap.max_host_bytes=limits.domain_bytes;
    DomainValues result;
    const auto built=result.domain.Initialize({vehicle.source_instance_id(),nodes.data(),nodes.size()},cap);
    if(!built)Reject(Status::InvalidInput,built.message,{},built.node);
    for(std::size_t i=0;i<count;++i) {
        const auto& a=vehicle.nodes()[i];const auto& b=result.domain.nodes()[i];
        Require(a.source_id==b.source_id&&tl::math::SameScalarBits(a.position.x,b.position.x)&&
            tl::math::SameScalarBits(a.position.y,b.position.y)&&tl::math::SameScalarBits(a.position.z,b.position.z),
            "Vehicle prefix changed in combined domain");
    }
    result.fixed.assign(count+4,0);result.rotation.assign(count+4,0);
    for(std::size_t i=count;i<count+4;++i){result.fixed[i]=7;result.rotation[i]=1;}
    if(result.fixed.capacity()+result.rotation.capacity()>4*(count+4))
        Reject(Status::ResourceLimit,"Combined mask capacity exceeded reservation");
    return result;
}
} // namespace crash::cases::vehicle_wall::native::detail
