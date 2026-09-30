#include "ActualMappingSupport.h"
#include "../MappingDraft.h"
#include <map>
namespace crash::output::full_shell::source::test {
namespace {
struct GlobalOrder:ActualOrder {
    MappingExecution execution;
    explicit GlobalOrder(const CanonicalSource& source):ActualOrder(source) {
        execution.profile=MappingExecutionProfile::NativeA62OrdinaryLaw1;
        execution.projection_working_length_m=execution.coefficient_working_length_m=source.data().inputs.units.length_to_m;
        const auto& a=FindArray(source.data(),"shells_records");const auto r=arrays::Decode<std::uint64_t>(a.descriptor,a.bytes);
        const auto& b=FindArray(source.data(),"shells_node_indices");const auto c=arrays::Decode<std::uint32_t>(b.descriptor,b.bytes);
        std::map<std::uint64_t,MappingExecutionPart> parts;
        std::uint32_t q=0,t=0;
        for(auto& p:parents) {
            const auto& d=FindPart(source.data(),r[6*p.canonical_parent+1]);
            if(d.material_role!=SourceMaterialRole::Elastic)continue;
            const bool tri=c[4*p.canonical_parent+2]==c[4*p.canonical_parent+3];
            p.native_family=tri?2:1;p.family_index=tri?t++:q++;p.native_points=0;p.plastic=PlasticField::NotApplicable;
            auto [at,added]=parts.try_emplace(d.part,MappingExecutionPart{d.part,d.material,d.section,0,0});
            tri?++at->second.t3:++at->second.qeph;
        }
        for(const auto& p:parts)execution.parts.push_back(p.second);
    }
    MappingInput NativeView() const {auto value=View();value.execution=&execution;return value;}
};
RecordFile Metadata(const std::filesystem::path& root,const std::string& name,const Document& doc) {
    WriteJson(root/name,doc);const auto s=ReadBounded(root/name,MappingMetadataByteCap);return {name,Sha256(s),s.size()};
}
}
TEST(SourceMappingActual, GlobalLaw1ProvenanceRoundTripBindsSourceRolesAndKeepsEightArrays) {
    if(!std::getenv("ROBO_STATIC_CANONICAL"))GTEST_SKIP()<<"Explicit source fixture required";
    const auto& source=ActualSource();GlobalOrder order(source);
    ASSERT_EQ(order.execution.parts.size(),10u);
    std::uint64_t q=0,t=0;for(const auto& p:order.execution.parts){q+=p.qeph;t+=p.t3;}
    EXPECT_EQ(q,26225u);EXPECT_EQ(t,952u);
    const auto mapping=PreparedSourceMapping::Prepare(source,order.NativeView());
    ASSERT_NE(mapping.execution(),nullptr);EXPECT_EQ(mapping.arrays().size(),8u);
    EXPECT_NE(mapping.digest(),MappingDigest(mapping.arrays()));
    full_shell::test::Directory dir;
    const auto file=WriteMappingRecord(dir.path,mapping,"native");
    const auto restored=ReadMappingRecord(dir.path,source,file,mapping.digest());
    ASSERT_NE(restored.execution(),nullptr);EXPECT_EQ(restored.execution()->parts.size(),10u);
    EXPECT_EQ(restored.digest(),mapping.digest());
    for(std::size_t i=0;i<8;++i)EXPECT_EQ(restored.arrays()[i].bytes,mapping.arrays()[i].bytes);
    const auto& legacy=ActualMapping();EXPECT_EQ(legacy.execution(),nullptr);
    EXPECT_EQ(legacy.digest(),MappingDigest(legacy.arrays()));
    const auto plan=PlanMappingRecord(legacy,"legacy");
    const auto doc=array_json::Parse(plan.metadata_bytes,MappingMetadataByteCap);
    EXPECT_EQ(array_json::Text(doc["schema"]),MappingSchema);EXPECT_FALSE(doc.HasMember("execution"));
}
TEST(SourceMappingActual, MissingDescriptorWrongMaterialAndForgedPointCountsReject) {
    if(!std::getenv("ROBO_STATIC_CANONICAL"))GTEST_SKIP()<<"Explicit source fixture required";
    const auto& source=ActualSource();GlobalOrder order(source);
    EXPECT_THROW(PreparedSourceMapping::Prepare(source,order.View()),std::exception);
    const auto good=order.execution;
    for(unsigned fault=0;fault<5;++fault) {
        order.execution=good;
        switch(fault) {
        case 0:order.execution.parts.erase(order.execution.parts.begin());break;
        case 1:++order.execution.parts[0].material;break;
        case 2:++order.execution.parts[0].qeph;break;
        case 3:order.execution.coefficient_working_length_m=1.;break;
        case 4:order.execution.parts.push_back(order.execution.parts[0]);break;
        }
        EXPECT_THROW(PreparedSourceMapping::Prepare(source,order.NativeView()),std::exception);
    }
    order.execution=good;
    const auto at=std::find_if(order.parents.begin(),order.parents.end(),[](const auto& p){return p.plastic==PlasticField::NotApplicable;});
    ASSERT_NE(at,order.parents.end());at->native_points=3;
    EXPECT_THROW(PreparedSourceMapping::Prepare(source,order.NativeView()),std::exception);
    at->native_points=0;
    EXPECT_NO_THROW(PreparedSourceMapping::Prepare(source,order.NativeView()));
}
TEST(SourceMappingActual, RehashedDescriptorDeletionAndSchemaDowngradeCannotReplaceMapping) {
    if(!std::getenv("ROBO_STATIC_CANONICAL"))GTEST_SKIP()<<"Explicit source fixture required";
    const auto& source=ActualSource();GlobalOrder order(source);
    const auto mapping=PreparedSourceMapping::Prepare(source,order.NativeView());
    full_shell::test::Directory dir;const auto file=WriteMappingRecord(dir.path,mapping,"good");
    const auto plan=PlanMappingRecord(mapping,"good");
    auto visible=std::make_unique<PreparedSourceMapping>(ReadMappingRecord(dir.path,source,file,mapping.digest()));
    const auto* before=visible.get();
    for(unsigned fault=0;fault<4;++fault) {
        auto doc=detail::MappingDocument(source,mapping.digest(),plan.arrays,mapping.execution());
        switch(fault) {
        case 0:doc.RemoveMember("execution");break;
        case 1:doc.RemoveMember("execution");doc["schema"].SetString(MappingSchema,doc.GetAllocator());break;
        case 2:doc["execution"]["resolved_npt"].SetUint(3);break;
        case 3:doc["execution"]["parts"][0]["qeph_parents"].SetUint64(1048576);break;
        }
        const auto changed=Metadata(dir.path,"bad"+std::to_string(fault)+".json",doc);
        EXPECT_THROW(visible=std::make_unique<PreparedSourceMapping>(ReadMappingRecord(dir.path,source,changed,mapping.digest())),std::exception);
        EXPECT_EQ(visible.get(),before);
    }
    EXPECT_NO_THROW(visible=std::make_unique<PreparedSourceMapping>(ReadMappingRecord(dir.path,source,file,mapping.digest())));
}
TEST(SourceMappingActual, ExecutionDescriptorAndCounterWorkspaceAreAdmittedBeforeArrays) {
    if(!std::getenv("ROBO_STATIC_CANONICAL"))GTEST_SKIP()<<"Explicit source fixture required";
    const auto& source=ActualSource();GlobalOrder order(source);
    const auto legacy=detail::MappingWorkingBytes(source.data(),order.View());
    const auto total=detail::MappingWorkingBytes(source.data(),order.NativeView());
    EXPECT_GT(total,legacy+order.execution.parts.size()*sizeof(MappingExecutionPart));
    auto data=source.data();data.limits.host_bytes=total;
    EXPECT_EQ(detail::MappingWorkingBytes(data,order.NativeView()),total);
    --data.limits.host_bytes;
    auto invalid=order.NativeView();invalid.canonical_nodes=reinterpret_cast<const std::uint32_t*>(1);
    invalid.parents=reinterpret_cast<const NativeParent*>(1);
    // The capacity failure precedes either deliberately invalid borrowed array.
    EXPECT_THROW(detail::BuildMapping(data,invalid),std::exception);
    data.limits.host_bytes=legacy;EXPECT_EQ(detail::MappingWorkingBytes(data,order.View()),legacy);
}

TEST(SourceMappingActual, LegacyRigidZeroPointRowsStillUseMappingV1) {
    if(!std::getenv("ROBO_STATIC_CANONICAL"))GTEST_SKIP()<<"Explicit source fixture required";
    const auto& source=ActualSource();ActualOrder order(source);
    const auto& a=FindArray(source.data(),"shells_records");
    const auto records=arrays::Decode<std::uint64_t>(a.descriptor,a.bytes);
    std::size_t rigid=0;
    for(auto& p:order.parents) {
        const auto& part=FindPart(source.data(),records[6*p.canonical_parent+1]);
        if(part.material_role==SourceMaterialRole::Rigid) {
            p.plastic=PlasticField::NotApplicable;p.native_points=0;++rigid;
        }
    }
    ASSERT_EQ(rigid,5102u);
    const auto mapping=PreparedSourceMapping::Prepare(source,order.View());
    EXPECT_EQ(mapping.execution(),nullptr);EXPECT_EQ(mapping.digest(),MappingDigest(mapping.arrays()));
    full_shell::test::Directory dir;
    const auto file=WriteMappingRecord(dir.path,mapping,"rigid");
    const auto restored=ReadMappingRecord(dir.path,source,file,mapping.digest());
    EXPECT_EQ(restored.execution(),nullptr);EXPECT_EQ(restored.digest(),mapping.digest());
}

}
