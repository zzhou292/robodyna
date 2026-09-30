#include "Storage.h"
namespace crash::cases::vehicle_startup::joints::detail {
tl::fea::type45::JointInput Pack(const modelio::type45::Row& row,const modelio::type45::Data& data) {
    namespace src=modelio::type45;
    output::Require(row.disposition==src::Disposition::Required && row.property_index<data.properties.size(),
        "Only retained original joint rows may enter the physical model");
    tl::fea::type45::JointInput result;
    result.property=data.properties[row.property_index].value;
    result.geometry=row.Geometry();
    for(unsigned endpoint=0;endpoint<2;++endpoint) {
        const auto& node=row.nodes[endpoint];
        output::Require(node.domain_index!=SIZE_MAX && node.body.retained && node.body.source_id &&
            (node.body.kind==src::BodyKind::PlainGroup || node.body.kind==src::BodyKind::PartRoot),
            "Retained joint endpoint has no actual source rigid body");
        result.body[endpoint]={node.body.kind==src::BodyKind::PlainGroup?
            tl::fea::RigidBindingSourceKind::NodalGroup:tl::fea::RigidBindingSourceKind::Part,
            node.body.source_id};
    }
    return result;
}
} // namespace crash::cases::vehicle_startup::joints::detail
