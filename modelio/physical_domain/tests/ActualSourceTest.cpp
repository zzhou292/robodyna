#include "../VehiclePhysicalDomain.h"
#include "modelio/physical_scope/tests/ActualSupport.h"
#include "modelio/point_mass/VehiclePointMassSource.h"
#include "modelio/type25/VehicleType25Source.h"
#include <set>

namespace crash::modelio::physical_domain {
namespace {
constexpr auto Original=Policy::RetainedShellAssembliesV1;
const VehiclePhysicalDomain& Actual() {
    static const auto result=[] {
        const auto& source=physical_scope::test::Actual();
        const auto forecast=VehiclePhysicalDomain::Preflight(source,Original);
        std::cout << "VehiclePhysicalDomain complete preflight bytes=" << forecast.total_bytes << std::endl;
        return VehiclePhysicalDomain::Prepare(source,Original);
    }();
    return result;
}
}
TEST(VehiclePhysicalDomainOriginal,CompleteSelectedGeometryAndExplicitSourceReductionShareOneCanonicalDomain) {
    const auto& result=Actual();
    const auto& source=result.source();
    const auto counts=result.counts();
    EXPECT_EQ(result.domain().node_count(),372435);
    EXPECT_EQ(counts.complete_groups,727);
    EXPECT_EQ(counts.restricted_groups,26);
    EXPECT_EQ(counts.omitted_groups,6);
    EXPECT_EQ(counts.plain_members,7372);
    EXPECT_EQ(counts.retained_point_masses,148);
    ASSERT_EQ(result.plain_groups().size(),759);
    std::size_t restricted_members=0;
    for(const auto& group:result.plain_groups()) {
        const auto& original=source.data().plain_groups.at(group.source_group);
        EXPECT_EQ(group.members.size()+group.excluded_members.size(),original.members.size());
        std::size_t kept=0,excluded=0;
        for(const auto& member:original.members) {
            if(kept<group.members.size() && group.members[kept]==member.node) {
                EXPECT_NE(result.domain().Find(member.node),SIZE_MAX); ++kept;
            } else {
                ASSERT_LT(excluded,group.excluded_members.size());
                EXPECT_EQ(group.excluded_members[excluded++],member.node);
                EXPECT_EQ(result.domain().Find(member.node),SIZE_MAX);
            }
        }
        if(group.disposition==GroupDisposition::Restricted) {
            restricted_members+=group.excluded_members.size();
            EXPECT_GE(group.members.size(),4);
            EXPECT_NE(group.case_node_set_id,original.node_set_id);
        }
    }
    EXPECT_EQ(restricted_members,87);
    const auto& original=source.point_mass_source().rigid_source().topology();
    const auto& topology=result.topology();
    ASSERT_EQ(topology.part_count(),22);
    ASSERT_EQ(topology.root_count(),20);
    ASSERT_EQ(topology.member_count(),5452);
    ASSERT_EQ(topology.other_rigid_member_count(),7372);
    for(std::size_t p=0;p<topology.part_count();++p) {
        EXPECT_EQ(topology.parts()[p].source_part_id,original.parts()[p].source_part_id);
        EXPECT_EQ(topology.parts()[p].member_offset,original.parts()[p].member_offset);
        EXPECT_EQ(topology.parts()[p].member_count,original.parts()[p].member_count);
        EXPECT_EQ(topology.parts()[p].root_index,original.parts()[p].root_index);
    }
    for(std::size_t n=0;n<topology.member_count();++n) {
        EXPECT_EQ(topology.original_members()[n],original.original_members()[n]);
        EXPECT_EQ(topology.root_members()[n],original.root_members()[n]);
        EXPECT_NE(result.domain().Find(topology.original_members()[n]),SIZE_MAX);
    }
    const auto masses=point_mass::VehiclePointMassSource::Prepare(source,result.domain());
    EXPECT_EQ(masses.contributions().records().size(),148);
    const auto welds=type25::VehicleType25Source::Prepare(source,result.domain(),
        {type25::Policy::OriginalDefaultSpotweldsV1,0x59415249533235ULL});
    EXPECT_EQ(welds.model().connection_count(),2828);
    EXPECT_TRUE(masses.contributions().domain()->SharesStorage(result.domain()));
    EXPECT_TRUE(welds.domain().SharesStorage(result.domain()));
    RecordProperty("domain_nodes",result.domain().node_count());
    RecordProperty("plain_members",counts.plain_members);
    RecordProperty("point_mass_records",counts.retained_point_masses);
    RecordProperty("complete_forecast",result.forecast().total_bytes);
    RecordProperty("domain_owned_bytes",result.domain().owned_payload_bytes());
    RecordProperty("topology_owned_bytes",topology.owned_payload_bytes());
}
TEST(VehiclePhysicalDomainOriginal,ExactForecastAndNativeCapsPreserveExistingSelectionOnFailure) {
    auto result=Actual();
    const auto& source=result.source();
    const auto* before=result.domain().nodes().data();
    Limits limits;
    limits.host_bytes=result.forecast().total_bytes-1;
    EXPECT_THROW(result=VehiclePhysicalDomain::Prepare(source,Original,limits),std::runtime_error);
    EXPECT_EQ(result.domain().nodes().data(),before);
    ++limits.host_bytes;
    EXPECT_NO_THROW(result=VehiclePhysicalDomain::Prepare(source,Original,limits));
    EXPECT_TRUE(result.domain().Matches(Actual().domain()));
    limits={};limits.domain_bytes=result.domain().startup_payload_bytes()-1;
    EXPECT_THROW(VehiclePhysicalDomain::Prepare(source,Original,limits),std::runtime_error);
    ++limits.domain_bytes;
    EXPECT_NO_THROW(VehiclePhysicalDomain::Prepare(source,Original,limits));
    limits={};limits.topology_bytes=result.topology().startup_payload_bytes()-1;
    EXPECT_THROW(VehiclePhysicalDomain::Prepare(source,Original,limits),std::runtime_error);
    ++limits.topology_bytes;
    EXPECT_NO_THROW(VehiclePhysicalDomain::Prepare(source,Original,limits));
    EXPECT_THROW(VehiclePhysicalDomain::Prepare(source,static_cast<Policy>(99)),std::runtime_error);
}
} // namespace crash::modelio::physical_domain
