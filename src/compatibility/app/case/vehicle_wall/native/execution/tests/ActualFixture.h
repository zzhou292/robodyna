#pragma once
#include "../../EnvelopeOwnerSource.h"
#include "../../physical/tests/ActualFixture.h"
#include "case/vehicle_startup/physical_attachments/tests/PreparePost.h"
#include "case/vehicle_self_contact/native/TopologyDigestFields.h"
namespace crash::cases::vehicle_wall::native::execution_test {
namespace base=physical_test;
inline const EnvelopePhysicalSource& Mechanical() {
    static const auto value=EnvelopePhysicalSource::Prepare(base::Wall(),base::References());return value;
}
inline const EnvelopeExecutionSource& Executed() {
    static const auto value=EnvelopeExecutionSource::Prepare(Mechanical());return value;
}
inline const vehicle_startup::TiedSearchPostKinChk& Post() {
    static const auto value=vehicle_startup::physical_attachments::test::PreparePost(
        base::Wall().vehicle_origin().source(),base::original::Inputs().member);return value;
}
inline const modelio::type45::VehicleType45Source& JointSource() {
    static const auto value=modelio::type45::VehicleType45Source::Prepare(base::Wall().vehicle_origin(),
        modelio::type45::Policy::OriginalDirectSdiType45VehicleSupportsV5);return value;
}
inline std::string WitnessDigest(const vehicle_startup::TiedCinWitnessRoster& source) {
    namespace hash=vehicle_self_contact::native::detail::digest;
    const auto& data=source.data();hash::Fields h("retained-original-cin-witnesses-v1",1u<<20);
    h.Add<std::uint64_t>("ranges",data.ranges.size(),2,[&](auto i) {
        const auto& row=data.ranges[i/2];return std::uint64_t(i%2?row.count:row.offset);
    });
    h.Add<std::uint64_t>("actual_witnesses",data.witnesses.size(),3,[&](auto i) {
        const auto& row=data.witnesses[i/3];
        const std::uint64_t v[]{std::uint64_t(row.family),row.native_parent_index,row.source_element_id};return v[i%3];
    });
    h.Add<std::uint64_t>("actual_source_nodes",data.origins.size(),6,[&](auto i) {
        const auto& row=data.origins[i/6];
        const std::uint64_t v[]{row.source_part_id,row.source_parent_row,row.source_node_ids[0],row.source_node_ids[1],
            row.source_node_ids[2],row.source_node_ids[3]};return v[i%6];
    });
    return h.Finish().sha256;
}
}
