#include "SourceNodalWallTest.h"
#include <map>

namespace crash::qualification::source_contact::nodal::test {
TEST_F(Check, ActualGpuCoherentSourceMatchesOwningHostAcrossThreeFiniteWalls) {
    Device device; ASSERT_EQ(device.initialization_status(),cudaSuccess);
    Result first; bool has_first=false;
    for (auto kind:{cw::WallTessellationKind::Original,cw::WallTessellationKind::FlipConvexPairs,cw::WallTessellationKind::UniformFour}) {
        cw::WallTessellation geometry; ASSERT_NO_FATAL_FAILURE(Geometry(geometry,kind));
        Input input; ASSERT_NO_FATAL_FAILURE(Coherent(*geometry.geometry(),input));
        ASSERT_EQ(input.face_count,kind==cw::WallTessellationKind::UniformFour?400u:100u);
        const auto source_packet=Bytes(input); Result expected,actual;
        ASSERT_EQ(EvaluateHost(prepared,input,&expected).status,Status::Ok);
        Report report; Timing timing;
        ASSERT_EQ(device.Evaluate(input,actual,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
        ASSERT_NO_FATAL_FAILURE(Compare(expected,actual)); Unchanged(input,source_packet);
        EXPECT_GT(actual.contact.resultant.lower,0); EXPECT_GT(actual.contact.potential.lower,0);
        if (has_first) ASSERT_NO_FATAL_FAILURE(Compare(first,actual,false)); else { first=actual; has_first=true; }
        RecordProperty(std::string(cw::WallTessellationName(kind))+"_event_ms",Number(timing.kernel_ms));
        RecordProperty(std::string(cw::WallTessellationName(kind))+"_mesh_sha256",geometry.metadata()->mesh_sha256);
    }
    RecordProperty("explicit_device_bytes",static_cast<int>(sizeof(Storage)));
    RecordProperty("executing_threads",static_cast<int>(Workers));
    RecordProperty("reduction","GPU; sorted parent shares then unique nodes; no owner transaction"); Metadata();
}

TEST_F(Check, ActualGpuInternalSeamHasOneSpringAndMissingFiniteFaceRejects) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original)); const auto view=original.view();
    const unsigned active=MaximumX(); const auto target=source.positions().at(active);
    using Edge=std::pair<unsigned,unsigned>; std::map<Edge,unsigned> edges;
    for (unsigned p=0;p<view.triangle_count;++p) for (unsigned l=0;l<3;++l)
        ++edges[std::minmax(view.triangles[p].nodes[l],view.triangles[p].nodes[(l+1)%3])];
    double nearest=std::numeric_limits<double>::infinity(); sc::Vec3 midpoint; bool found=false;
    for (const auto& entry:edges) if (entry.second==2) {
        const auto c=sc::Scale(sc::Add(view.vertices[entry.first.first].position,view.vertices[entry.first.second].position),.5);
        const double d=(c.y-target.y)*(c.y-target.y)+(c.z-target.z)*(c.z-target.z);
        if (d<nearest) { nearest=d; midpoint=c; found=true; }
    }
    ASSERT_TRUE(found); const auto& geometry=*original.geometry(); const double shift=cf::WholeShift(source,geometry.wall_x());
    auto base=source.coordinates(),endpoint=cf::Shift(source,shift);
    const double dy=midpoint.y-target.y,dz=midpoint.z-target.z;
    for (unsigned n=0;n<NodeCount;++n) { base[3*n+1]+=dy; endpoint[3*n+1]+=dy; base[3*n+2]+=dz; endpoint[3*n+2]+=dz; }
    Input input; ASSERT_EQ(BuildInput(prepared,base,endpoint,cf::Velocity(shift),prepared.mass(),geometry,31,&input,diagnostic).status,Status::Ok)<<diagnostic;
    const sc::Vec3 point{geometry.wall_x(),endpoint[3*active+1],endpoint[3*active+2]};
    unsigned containing=0; std::uint64_t smallest=UINT64_MAX;
    for (const auto& face:geometry.faces()) {
        sc::TrianglePointGeometry closest;
        ASSERT_EQ(sc::ClosestPointOnTriangle(point,face.geometry,&closest),sc::Status::kOk);
        if (closest.distance<=geometry.tolerance()) { ++containing; smallest=std::min(smallest,face.geometry.face_id); }
    }
    ASSERT_GE(containing,2u); Device device; ASSERT_EQ(device.initialization_status(),cudaSuccess);
    Result expected,actual; ASSERT_EQ(EvaluateHost(prepared,input,&expected).status,Status::Ok);
    Report report; Timing timing; ASSERT_EQ(device.Evaluate(input,actual,report,timing),cudaSuccess);
    ASSERT_EQ(report.status,Status::Ok); ASSERT_NO_FATAL_FAILURE(Compare(expected,actual));
    EXPECT_EQ(actual.wall_face[active],smallest); EXPECT_GT(actual.contact.nodes[active].force.lower,0);
    const auto saved=Bytes(actual);
    sc::PlanarWallGeometry hole; sc::Vec3 center; ASSERT_NO_FATAL_FAILURE(Hole(original,target,hole,center));
    // Fault packet deliberately bypasses the host's already-tested rejection:
    // the device must perform its own finite endpoint query, not trust a flag.
    input.position[3*active+1]=center.y; input.position[3*active+2]=center.z;
    input.face_count=static_cast<unsigned>(hole.faces().size()); std::copy(hole.faces().begin(),hole.faces().end(),input.faces);
    ASSERT_EQ(device.Evaluate(input,actual,report,timing),cudaSuccess); EXPECT_EQ(report.status,Status::OutsideWall);
    Unchanged(actual,saved); Metadata();
}

TEST_F(Check, ActualGpuLateFailureKeepsPublishedFieldsThenRetryAndCudaPoisonAreExplicit) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original));
    Input input; ASSERT_NO_FATAL_FAILURE(Coherent(*original.geometry(),input)); const Input good=input;
    Device device; ASSERT_EQ(device.initialization_status(),cudaSuccess);
    Result clean,output; Report report; Timing timing;
    ASSERT_EQ(device.Evaluate(input,clean,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    output=clean; const auto saved=Bytes(output);
    input.inverse_mass[NodeCount-1]=0;
    ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); EXPECT_EQ(report.status,Status::PointFailure);
    EXPECT_EQ(report.node,NodeCount-1); Unchanged(output,saved);
    input=good; input.position[3*(NodeCount-1)+1]=100;
    ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); EXPECT_EQ(report.status,Status::OutsideWall); Unchanged(output,saved);
    input=good; input.share_slot[input.share_count-1]=input.share_slot[0];
    ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); EXPECT_EQ(report.status,Status::InvalidInput); Unchanged(output,saved);
    input=good; unsigned positive=0;
    for (unsigned p=1;p<ParentCount;++p)
        if (clean.contact.parents[p].potential.lower>clean.contact.parents[positive].potential.lower) positive=p;
    ASSERT_GT(.1*clean.contact.parents[positive].potential.lower,cf::EnergyBudget);
    // Wider but conservative area-share enclosure changes no nominal force or
    // declared budget. The per-parent arithmetic certificate must now reject.
    auto& share=input.parents[positive].share;
    ASSERT_TRUE(sc::q4_bounds::Certify(share.value,{share.lower*.9,share.upper*1.1},&share));
    ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); EXPECT_EQ(report.status,Status::Accuracy); Unchanged(output,saved);
    input=good; ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(ExactResult(clean,output));
    // The fixed-node branch uses the actual fixed mask, zero inverse mass and
    // unchanged separated position, without a fabricated dynamic effective mass.
    auto fixed_mass=prepared.mass(); fixed_mass.fixed[NodeCount-1]=1; fixed_mass.inverse[NodeCount-1]=0;
    ASSERT_EQ(BuildInput(prepared,source.coordinates(),source.coordinates(),cf::Coordinates{},fixed_mass,
        *original.geometry(),31,&input,diagnostic).status,Status::Ok)<<diagnostic;
    Result fixed_expected; ASSERT_EQ(EvaluateHost(prepared,input,&fixed_expected).status,Status::Ok);
    ASSERT_EQ(device.Evaluate(input,output,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(Compare(fixed_expected,output));
    const auto before_poison=Bytes(output); ASSERT_EQ(cudaPeekAtLastError(),cudaSuccess);
    const auto injected=InjectInvalidLaunchForCheck();
    ASSERT_TRUE(injected==cudaErrorInvalidValue || injected==cudaErrorInvalidConfiguration);
    EXPECT_EQ(device.Evaluate(good,output,report,timing),injected); Unchanged(output,before_poison);
    EXPECT_EQ(cudaPeekAtLastError(),cudaSuccess); EXPECT_EQ(device.Evaluate(good,output,report,timing),injected);
    Unchanged(output,before_poison);
    Device healthy; ASSERT_EQ(healthy.initialization_status(),cudaSuccess);
    ASSERT_EQ(healthy.Evaluate(good,output,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(ExactResult(clean,output)); Metadata();
}
} // namespace crash::qualification::source_contact::nodal::test
