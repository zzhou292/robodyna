#include "ActualFixture.h"
#include "../TopologyInput.h"
#include "lib_src/collision/RadiossType25MainGeometry.h"
#include "output/BoundedArrayJson.h"
#include <algorithm>
namespace crash::cases::vehicle_self_contact::native::coated::test {
namespace {
constexpr std::size_t IdentityColumns = 14, ValueColumns = 6;
std::size_t GeometryReservation(const Forecast& base) {
    // All selected parents could be coated. Identity/value vectors and encoded
    // copies coexist with the complete prior source/topology reservation.
    const auto rows = Selection().data().counts.retained_shells;
    const auto extra = 4*rows*(IdentityColumns*sizeof(std::uint64_t)+ValueColumns*sizeof(double)) + (1u<<20);
    output::Require(base.admitted && base.peak_bytes <= Limits{}.host_bytes &&
        extra <= Limits{}.host_bytes-base.peak_bytes, "Complete geometry census exceeds source envelope");
    return base.peak_bytes+extra;
}
void Attach(output::Document& doc, const char* name, const output::arrays::Descriptor& descriptor) {
    const auto value=output::arrays::DescriptorDocument(descriptor);
    output::array_json::Child(doc,name,value);
}
}
TEST(V5MainGeometryActual, ForecastCompleteUnprojectedSourceBeforeGeometryAllocation) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto base=Preflight(physical::Model(),Selection());
    const auto bytes=GeometryReservation(base);
    output::Document doc;doc.SetObject();
    output::String(doc,"scope","Unprojected reader-coordinate diagnostic; no offset/topology/runtime admission");
    output::Integer(doc,"peak_reservation_bytes",bytes);
    Write("forecast.json",doc);
    EXPECT_LE(bytes,Limits{}.host_bytes);
}
TEST(V5MainGeometryActual, EverySelectedCoatingReportsNativePrimaryGeometryWithoutChangingPartners) {
    ASSERT_NO_FATAL_FAILURE(SourceCounts());
    const auto& model=physical::Model();const auto& selected=Selection();
    const auto forecast=Preflight(model,selected);const auto reserved=GeometryReservation(forecast);
    const auto input=detail::PrepareInputs(model,selected,physical::Inputs().member,{},{});
    const auto classified=Classify(input);
    ASSERT_TRUE(classified.complete);
    ASSERT_TRUE(classified.contact_complete);
    const auto order=SurfaceOrder(input,classified);
    detail::TopologyInput packed(input,classified,order);const auto view=packed.View();
    tl::util::HostArena output_arena,scratch;
    ASSERT_TRUE(output_arena.Initialize(forecast.topology.output_bytes));
    ASSERT_TRUE(scratch.Initialize(forecast.topology.scratch_bytes));
    s::Snapshot snapshot;
    ASSERT_EQ(s::BuildStarter(view,Limits{}.topology,output_arena,scratch,&snapshot).status,s::Status::Ok);
    const auto& canonical=selected.canonical().data();const auto& declared=selected.data();
    const Provenance provenance{canonical.inputs.canonical_manifest,canonical.inputs.scope_report,
        canonical.inputs.source_member,declared.auxiliary_sha256,declared.combine_sha256,canonical.inputs.units};
    const auto input_digest=detail::InputDigest(input,classified,&order,{},detail::SourceBinding(provenance),Limits{}.metadata_bytes);
    const auto before=detail::OutputDigest(snapshot,input_digest.sha256,Limits{}.metadata_bytes);
    // Exact previously sealed output: shared packing/refactored normal helper
    // cannot silently change the earlier source phase while adding this census.
    EXPECT_EQ(before.sha256,"0c8c4cfdebcbca4cb10a4d31733023e518e290d2727eabe675d7b7ae40b94beb");
    std::vector<std::uint64_t> identities;std::vector<double> values;
    const auto count=classified.contact.unique;
    identities.reserve(count*IdentityColumns);values.reserve(count*ValueColumns);
    std::size_t reversed=0,failed=0,nonpositive_volume=0;
    for(std::size_t primary=0;primary<order.primary_to_physical.size();++primary) {
        const auto physical=order.primary_to_physical[primary];const auto& role=classified.roles[physical];
        if(role.state==RoleState::Ordinary)continue;
        ASSERT_EQ(role.matches,1u);
        ASSERT_LT(role.first_solid,input.solids.size());
        const auto& shell=input.shells[physical];const auto& solid=input.solids[role.first_solid];
        const auto& main=snapshot.mains[primary];
        n::NativeExteriorMainGeometryInput packet;packet.layout=shell.primary.layout;
        for(unsigned k=0;k<4;++k)packet.face[k]=input.nodes[main.nodes[k]].native_position;
        for(unsigned k=0;k<8;++k)packet.solid_raw[k]=input.nodes[solid.nodes[k]].native_position;
        n::NativeExteriorMainGeometryResult geometry;
        const auto status=n::EvaluateNativeExteriorMainGeometry(packet,&geometry);
        failed+=status!=n::CoefficientStatus::Ok;
        reversed+=status==n::CoefficientStatus::Ok && geometry.reversed;
        nonpositive_volume+=status==n::CoefficientStatus::Ok && !(geometry.signed_volume>0);
        identities.insert(identities.end(),{shell.primary.source_id,shell.part_id,primary,solid.source_id,
            solid.part_id,std::uint64_t(solid.kind),solid.family,std::uint64_t(role.state),
            std::uint64_t(geometry.reversed),std::uint64_t(status),geometry.source_corner[0],
            geometry.source_corner[1],geometry.source_corner[2],geometry.source_corner[3]});
        values.insert(values.end(),{geometry.normal_before_orientation.x,geometry.normal_before_orientation.y,
            geometry.normal_before_orientation.z,geometry.area,geometry.signed_volume,geometry.center_projection});
    }
    ASSERT_EQ(identities.size(),IdentityColumns*count);
    ASSERT_EQ(values.size(),ValueColumns*count);
    const auto after=detail::OutputDigest(snapshot,input_digest.sha256,Limits{}.metadata_bytes);
    EXPECT_EQ(after.sha256,before.sha256);
    const auto* destination=std::getenv("ROBO_V5_COATED_OUTPUT");
    output::Require(destination && *destination,"Missing geometry diagnostic output directory");
    const std::filesystem::path path(destination);
    output::Require(std::filesystem::create_directory(path),"Geometry diagnostic output already exists");
    const auto ids=output::arrays::Write<std::uint64_t>(path,"geometry-source.bin",
        {output::arrays::Scalar::UInt64,count,IdentityColumns,{"shell_eid","shell_pid","primary_ordinal","solid_eid",
        "solid_pid","reader_kind","solid_family","source_side_role","primary_reversed","status","corner0","corner1","corner2","corner3"}},
        identities.data(),identities.size());
    const auto fields=output::arrays::Write<double>(path,"geometry-values.bin",
        {output::arrays::Scalar::Float64,count,ValueColumns,{"normal_x","normal_y","normal_z","area","signed_volume","center_projection"}},
        values.data(),values.size());
    output::Document doc;doc.SetObject();
    output::String(doc,"schema","robo_dyna.v5_main_geometry_census.v1");
    output::String(doc,"scope","All selected coated-shell unique supports under unchanged native reader coordinates; diagnostic only");
    output::String(doc,"coordinate_authority","X branch assumed explicitly for this census; offset context and final topology are separate admission gates");
    output::String(doc,"phase","SH2 primary nodes before I25GAPM; native geometry observed without mutating primary or partner");
    output::String(doc,"value_validity","Geometry values/corner permutations are native observations only for status==Ok; other rows retain diagnostic placeholders");
    output::String(doc,"input_digest",input_digest.sha256);output::String(doc,"unchanged_topology_digest",after.sha256);
    output::Integer(doc,"coated_primaries",count);output::Integer(doc,"primary_reversals",reversed);
    output::Integer(doc,"geometry_failures",failed);output::Integer(doc,"nonpositive_volume",nonpositive_volume);
    output::Integer(doc,"peak_reservation_bytes",reserved);
    output::Boolean(doc,"requires_primary_phase_mutation",reversed!=0);
    output::Boolean(doc,"runtime_admitted",false);
    Attach(doc,"source_identity",ids);Attach(doc,"geometry_values",fields);
    output::WriteJson(path/"geometry.json",doc);
    RecordProperty("primary_reversals",std::to_string(reversed));RecordProperty("geometry_failures",std::to_string(failed));
    EXPECT_EQ(count,741u);EXPECT_EQ(failed,0u);
}
}
