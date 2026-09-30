#include "Fixture.h"
#include <climits>
namespace crash::modelio::native_spring_ids::test {
TEST(NativeSpringIds, CompleteClosedSourceDerivesMaxAndCrossKindSelectionOrder) {
    Fixture fixture; const auto context=fixture.Context();
    ASSERT_EQ(context.diagnostic.status,Readiness::Ready);
    ASSERT_EQ(context.precursors.size(),3u); ASSERT_EQ(context.welds.size(),2u); ASSERT_EQ(context.joints.size(),1u);
    const auto result=detail::ResolveRows(context,Retained(context),{});
    ASSERT_EQ(result.diagnostic.status,Readiness::Ready);
    ASSERT_EQ(result.rows.size(),6u); EXPECT_EQ(result.existing_maximum,100u);
    EXPECT_EQ(result.Find(SourceKind::DefaultSpotweld,7)->native_id,101u);
    EXPECT_EQ(result.Find(SourceKind::DefaultSpotweld,8)->native_id,102u);
    EXPECT_EQ(result.Find(SourceKind::RegularJoint,31)->native_id,103u);
    EXPECT_EQ(result.final_maximum,103u); EXPECT_EQ(result.physical_order.size(),4u);
    for (const auto index : result.physical_order) EXPECT_NE(result.rows[index].kind,SourceKind::DiscreteNamespaceOnly);
    EXPECT_NE(result.existing_maximum,100000000u);
}
TEST(NativeSpringIds, NamespaceOnlyPrecursorChangesIdsWithoutAddingMechanics) {
    Fixture first(100), second(900); const auto a=first.Context(), b=second.Context();
    const auto left=detail::ResolveRows(a,Retained(a),{}), right=detail::ResolveRows(b,Retained(b),{});
    EXPECT_EQ(left.counts.physical,right.counts.physical);
    EXPECT_EQ(left.counts.discrete_namespace_only,2u);
    EXPECT_EQ(right.Find(SourceKind::DefaultSpotweld,7)->native_id,901u);
    EXPECT_NE(left.mapping_digest,right.mapping_digest); EXPECT_NE(left.source_digest,right.source_digest);
}
TEST(NativeSpringIds, UnknownOrIncompleteImportsRemainUnreadyRatherThanAssigningIds) {
    Fixture fixture; auto input=fixture.Input(); input.profile=Profile::Unknown;
    EXPECT_THROW(detail::BuildContext(fixture.canonical,input,{}),detail::Failure);
    input=fixture.Input(); input.members.pop_back();
    EXPECT_THROW(detail::BuildContext(fixture.canonical,input,{}),detail::Failure);
    input=fixture.Input(); input.entry_member="wall.key";
    EXPECT_THROW(detail::BuildContext(fixture.canonical,input,{}),detail::Failure);
    input=fixture.Input(); input.members[0].bytes="*KEYWORD\n*END\n";
    EXPECT_THROW(detail::BuildContext(fixture.canonical,input,{}),detail::Failure);
    Fixture transformed_spring(100,true);
    EXPECT_THROW(transformed_spring.Context(),detail::Failure);
}
TEST(NativeSpringIds, OverflowDuplicateAndWrongEndpointBindingsNeverProducePartialMapping) {
    Fixture fixture; const auto accepted=fixture.Context();
    auto context=accepted; context.precursors.back().original_id=INT_MAX;
    EXPECT_THROW(detail::ResolveRows(context,Retained(context),{}),detail::Failure);
    context=accepted; context.precursors.push_back(context.precursors.front());
    EXPECT_THROW(detail::ResolveRows(context,Retained(accepted),{}),detail::Failure);
    auto retained=Retained(accepted); retained.back().endpoints[0]+=1;
    EXPECT_THROW(detail::ResolveRows(accepted,retained,{}),detail::Failure);
    retained=Retained(accepted); retained.pop_back();
    EXPECT_THROW(detail::ResolveRows(accepted,retained,{}),detail::Failure);
    auto cap=Limits{}; cap.resolve_bytes=1;
    EXPECT_THROW(detail::ResolveRows(accepted,Retained(accepted),cap),detail::Failure);
    EXPECT_EQ(detail::ResolveRows(accepted,Retained(accepted),{}).final_maximum,103u);
}
TEST(NativeSpringIds, SourceIndexesRemainTypedWhileNativeRowsAreSorted) {
    Fixture fixture; const auto context=fixture.Context(); auto retained=Retained(context);
    for (auto& row : retained) row.source_index=17+unsigned(row.kind)*10+row.original_id;
    std::reverse(retained.begin(),retained.end());
    const auto result=detail::ResolveRows(context,retained,{});
    for (const auto& original : retained) {
        const auto* row=result.Find(original.kind,original.original_id); ASSERT_NE(row,nullptr);
        EXPECT_EQ(row->source_index,original.source_index); EXPECT_EQ(row->endpoints,original.endpoints);
    }
    for (std::size_t i=1;i<result.rows.size();++i) EXPECT_LT(result.rows[i-1].native_id,result.rows[i].native_id);
    EXPECT_EQ(result.Find(SourceKind::Type13,7),nullptr);
}
TEST(NativeSpringIds, UnrelatedMaterialPrefixesCannotEnterAuditedDispatch) {
    for (const auto* keyword : {"*MAT_1000", "*MAT_ELASTIC_UNAUDITED", "*MAT_SPOTWELD_TITLE"}) {
        SCOPED_TRACE(keyword);
        Fixture fixture; output::Document metadata;
        metadata.Parse(fixture.canonical.canonical_bytes.c_str());
        metadata["materials"][0]["keyword"].SetString(keyword,metadata.GetAllocator());
        rapidjson::StringBuffer bytes; rapidjson::Writer<rapidjson::StringBuffer> writer(bytes);
        metadata.Accept(writer); fixture.canonical.canonical_bytes.assign(bytes.GetString(),bytes.GetSize());
        EXPECT_THROW(fixture.Context(),detail::Failure);
    }
}
TEST(NativeSpringIds, BeamDispatchUsesExactSourceSectionInsteadOfShellConvenienceField) {
    Fixture fixture;
    ASSERT_EQ(fixture.canonical.parts.front().source_elform,0u);
    EXPECT_EQ(fixture.Context().precursors.size(),3u);
    const auto populate = [&](const output::Document& metadata) {
        auto context = fixture.Context();
        context.precursors.clear();
        context.non_spring_beams = 0;
        detail::PopulateElements(fixture.canonical,metadata,context,{});
        return context;
    };
    output::Document metadata;
    metadata.Parse(fixture.canonical.canonical_bytes.c_str());
    metadata["materials"][0]["keyword"].SetString("*MAT_024",metadata.GetAllocator());
    metadata["sections"][0]["formulation_field_raw"].SetString("1",metadata.GetAllocator());
    const auto ordinary = populate(metadata);
    EXPECT_EQ(ordinary.non_spring_beams,1u);
    EXPECT_EQ(ordinary.precursors.size(),2u);
    for (unsigned invalid=0; invalid<8; ++invalid) {
        SCOPED_TRACE(invalid);
        metadata.Parse(fixture.canonical.canonical_bytes.c_str());
        switch (invalid) {
        case 0: metadata["sections"][0]["source_section_id"].SetUint(11); break;
        case 1: metadata["sections"][0]["keyword"].SetString("*SECTION_BEAM_TITLE",metadata.GetAllocator()); break;
        case 2: metadata["sections"][0]["formulation_field_raw"].SetString("9.0",metadata.GetAllocator()); break;
        case 3: metadata["sections"][0]["formulation_field_raw"].SetString("",metadata.GetAllocator()); break;
        case 4: metadata["sections"][0]["formulation_field_raw"].SetString("-9",metadata.GetAllocator()); break;
        case 5: metadata["sections"][0]["formulation_field_raw"].SetString("2147483648",metadata.GetAllocator()); break;
        case 6: metadata["sections"][0]["formulation_field_raw"].SetString("1",metadata.GetAllocator()); break;
        case 7: {
            output::Value duplicate;
            duplicate.CopyFrom(metadata["sections"][0],metadata.GetAllocator());
            metadata["sections"].PushBack(duplicate,metadata.GetAllocator());
            break;
        }
        }
        EXPECT_THROW(populate(metadata),detail::Failure);
    }
    EXPECT_EQ(fixture.Context().precursors.size(),3u);
}
TEST(NativeSpringIds, FailureContextCannotBeUpgradedByResolution) {
    ContextData context; context.diagnostic={Readiness::UnsupportedSource,"Unproven preload",{},0,0};
    const auto result=detail::ResolveRows(context,{},{});
    EXPECT_EQ(result.diagnostic.status,Readiness::UnsupportedSource); EXPECT_TRUE(result.rows.empty());
    EXPECT_TRUE(result.physical_order.empty()); EXPECT_TRUE(result.source_order.empty());
    EXPECT_EQ(result.Find(SourceKind::Type13,1),nullptr);
}
} // namespace crash::modelio::native_spring_ids::test
