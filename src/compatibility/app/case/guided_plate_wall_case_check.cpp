#include "GuidedPlateCase.h"
#include "GuidedPlateArtifacts.h"
#include "CanonicalWallArtifacts.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <tuple>

namespace {
using namespace crash::case_data;
namespace ref=crash::reference;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
namespace fs=std::filesystem;
using Code=GuidedPlateStatus;
using Kind=WallTessellationKind;
std::string asset;

class TempRoot {
  public:
    TempRoot() {
        const auto pattern=(fs::temp_directory_path()/"guided-wall-variant-XXXXXX").string();
        std::vector<char> buffer(pattern.begin(),pattern.end()); buffer.push_back(0);
        const auto* made=::mkdtemp(buffer.data());
        if (!made) throw std::runtime_error("Could not create isolated wall variant test root");
        path=made;
    }
    ~TempRoot() { std::error_code error; fs::remove_all(path,error); }
    fs::path path;
};
class GuidedPlateWallCase:public ::testing::Test {
  protected:
    CanonicalWall wall;
    std::string bytes;
    void SetUp() override {
        int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
        ASSERT_NO_THROW(bytes=ReadPinnedWallManifest(asset));
        std::istringstream input(bytes); const auto report=wall.Load(input);
        ASSERT_EQ(report.status,WallStatus::Ok)<<report.message;
    }
};
template<class V> auto Components(const V& p) { return std::make_tuple(p.x,p.y,p.z); }
auto Components(const shell::Quaternion& q) { return std::make_tuple(q.w,q.x,q.y,q.z); }
void SamePoint(const shell::ShellPointReference& a,const shell::ShellPointReference& b) {
    EXPECT_EQ(a.area_weight,b.area_weight);
    for (unsigned c=0;c<9;++c) EXPECT_EQ(a.frame_offset.v[c],b.frame_offset.v[c]);
    for (unsigned axis=0;axis<2;++axis) {
        EXPECT_EQ(a.natural[axis],b.natural[axis]);
        EXPECT_EQ(Components(a.strain0[axis]),Components(b.strain0[axis]));
        EXPECT_EQ(Components(a.curvature0[axis]),Components(b.curvature0[axis]));
        for (unsigned n=0;n<4;++n) EXPECT_EQ(a.gradient[n][axis],b.gradient[n][axis]);
    }
    for (unsigned n=0;n<4;++n) EXPECT_EQ(a.shape[n],b.shape[n]);
}
void SameModel(const ref::ElasticCouponData& a,const ref::ElasticCouponData& b) {
    EXPECT_EQ(a.connectivity,b.connectivity); EXPECT_EQ(a.inverse_mass,b.inverse_mass);
    EXPECT_EQ(a.inverse_isotropic_inertia,b.inverse_isotropic_inertia); EXPECT_EQ(a.fixed,b.fixed);
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        EXPECT_EQ(Components(a.reference_configuration.position[n]),Components(b.reference_configuration.position[n]));
        EXPECT_EQ(Components(a.reference_configuration.rotation[n]),Components(b.reference_configuration.rotation[n]));
        EXPECT_EQ(a.nodal_mass[n].mass,b.nodal_mass[n].mass);
        EXPECT_EQ(a.nodal_mass[n].physical_tangential_inertia,b.nodal_mass[n].physical_tangential_inertia);
        EXPECT_EQ(a.nodal_mass[n].artificial_drilling_inertia,b.nodal_mass[n].artificial_drilling_inertia);
    }
    for (unsigned e=0;e<ref::kCouponElements;++e) {
        EXPECT_EQ(a.reference[e].prepared,b.reference[e].prepared);
        EXPECT_EQ(a.section[e].prepared,b.section[e].prepared);
        EXPECT_EQ(a.section[e].thickness,b.section[e].thickness); EXPECT_EQ(a.section[e].density,b.section[e].density);
        for (unsigned c=0;c<144;++c) EXPECT_EQ(a.section[e].stiffness[c],b.section[e].stiffness[c]);
        for (unsigned n=0;n<4;++n) {
            EXPECT_EQ(Components(a.reference[e].initial_position[n]),Components(b.reference[e].initial_position[n]));
            EXPECT_EQ(Components(a.reference[e].initial_rotation[n]),Components(b.reference[e].initial_rotation[n]));
            EXPECT_EQ(Components(a.reference[e].node_frame_offset[n]),Components(b.reference[e].node_frame_offset[n]));
            SamePoint(a.reference[e].gauss[n],b.reference[e].gauss[n]);
            SamePoint(a.reference[e].ans[n],b.reference[e].ans[n]);
        }
    }
}
void SameReference(contact::Q4PlanarReferenceView a,contact::Q4PlanarReferenceView b) {
    ASSERT_EQ(a.parent_count,b.parent_count); ASSERT_TRUE(a.parents); ASSERT_TRUE(b.parents);
    EXPECT_EQ(a.global_node_count,b.global_node_count); EXPECT_EQ(a.wall_x,b.wall_x); EXPECT_EQ(a.wall_tolerance,b.wall_tolerance);
    for (unsigned p=0;p<a.parent_count;++p) {
        const auto& x=a.parents[p]; const auto& y=b.parents[p];
        EXPECT_TRUE(x.covered); EXPECT_TRUE(y.covered); EXPECT_EQ(x.projected_area,y.projected_area);
        EXPECT_EQ(x.area_enclosure.lower,y.area_enclosure.lower); EXPECT_EQ(x.area_enclosure.upper,y.area_enclosure.upper);
        EXPECT_EQ(x.parent.feature_id,y.parent.feature_id); EXPECT_EQ(x.parent.parent_element_id,y.parent.parent_element_id);
        EXPECT_EQ(x.parent.parent_face_id,y.parent.parent_face_id); EXPECT_EQ(x.parent.half_thickness,y.parent.half_thickness);
        for (unsigned n=0;n<4;++n) {
            EXPECT_EQ(x.parent.nodes[n],y.parent.nodes[n]);
            EXPECT_EQ(Components(x.reference_projection[n]),Components(y.reference_projection[n]));
        }
    }
}
void SameFrame(const GuidedPlateFrame& a,const GuidedPlateFrame& b) {
    // Owner IDs, scratch attempts and wall binding deliberately differ. The
    // same prescribed initial state and separated two-step prefix must not.
    EXPECT_EQ(a.position,b.position); EXPECT_EQ(a.rotation,b.rotation);
    EXPECT_EQ(a.velocity,b.velocity); EXPECT_EQ(a.omega,b.omega);
    EXPECT_EQ(a.reaction_force,b.reaction_force); EXPECT_EQ(a.reaction_couple,b.reaction_couple);
    EXPECT_EQ(a.stamp.epoch,b.stamp.epoch); EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.fixed_dt,b.stamp.fixed_dt);
    EXPECT_EQ(a.stamp.reactions_valid,b.stamp.reactions_valid); EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch);
    EXPECT_EQ(a.stamp.reaction_time,b.stamp.reaction_time);
    EXPECT_EQ(a.metrics.initial_energy,b.metrics.initial_energy);
    EXPECT_EQ(a.metrics.shell.elastic_energy,b.metrics.shell.elastic_energy);
    EXPECT_EQ(a.metrics.work.total_energy,b.metrics.work.total_energy);
    EXPECT_EQ(a.metrics.maximum_relative_energy_error,b.metrics.maximum_relative_energy_error);
    EXPECT_EQ(a.metrics.shell_midpoint_work,b.metrics.shell_midpoint_work);
    EXPECT_EQ(a.metrics.shell_coordinate_work,b.metrics.shell_coordinate_work);
    EXPECT_EQ(a.metrics.contact_midpoint_work,b.metrics.contact_midpoint_work);
    EXPECT_EQ(a.metrics.contact_coordinate_work,b.metrics.contact_coordinate_work);
    EXPECT_EQ(Components(a.metrics.wall_impulse),Components(b.metrics.wall_impulse));
    EXPECT_EQ(Components(a.metrics.wall_moment_impulse),Components(b.metrics.wall_moment_impulse));
    EXPECT_EQ(a.metrics.contact.potential.upper,0); EXPECT_EQ(b.metrics.contact.potential.upper,0);
    EXPECT_EQ(a.metrics.applied_contact.potential.upper,0); EXPECT_EQ(b.metrics.applied_contact.potential.upper,0);
    for (unsigned e=0;e<ref::kCouponElements;++e) {
        EXPECT_EQ(a.element[e].energy,b.element[e].energy); EXPECT_EQ(a.element[e].bending_energy,b.element[e].bending_energy);
        for (unsigned n=0;n<4;++n) {
            EXPECT_EQ(Components(a.element[e].force[n]),Components(b.element[e].force[n]));
            EXPECT_EQ(Components(a.element[e].couple[n]),Components(b.element[e].couple[n]));
            EXPECT_EQ(Components(a.parent[e].integration.nodal.forces[n]),Components(b.parent[e].integration.nodal.forces[n]));
        }
    }
}
struct AllocationSnapshot {
    tl::fea::NodalAllocationInfo state,element,contact;
    explicit AllocationSnapshot(const GuidedPlateCase& run):state(run.state_allocations()),
        element(run.element_allocations()),contact(run.contact_allocations()) {}
    void Check(const GuidedPlateCase& run) const {
        const AllocationSnapshot actual(run);
        EXPECT_EQ(state.device_allocations,actual.state.device_allocations); EXPECT_EQ(state.device_bytes,actual.state.device_bytes);
        EXPECT_EQ(element.device_allocations,actual.element.device_allocations); EXPECT_EQ(element.device_bytes,actual.element.device_bytes);
        EXPECT_EQ(contact.device_allocations,actual.contact.device_allocations); EXPECT_EQ(contact.device_bytes,actual.contact.device_bytes);
        EXPECT_LE(actual.state.device_bytes+actual.element.device_bytes+actual.contact.device_bytes,kGuidedPlateDeviceBudget);
    }
};

TEST(GuidedWallBinding, ExplicitKindsAreDistinctAndInvalidIsUnbound) {
    const auto original=WallTessellationBindingId(Kind::Original),flip=WallTessellationBindingId(Kind::FlipConvexPairs),
               split=WallTessellationBindingId(Kind::UniformFour);
    EXPECT_EQ(original,kGuidedPlateWallBinding); EXPECT_EQ(original,0x5941524953574131ULL);
    EXPECT_NE(original,flip); EXPECT_NE(original,split); EXPECT_NE(flip,split); EXPECT_NE(flip,0u); EXPECT_NE(split,0u);
    EXPECT_EQ(WallTessellationBindingId(static_cast<Kind>(255)),0u);
}

TEST_F(GuidedPlateWallCase, ExplicitVariantsPreserveCanonicalSetupAndTransactionalPrefix) {
    GuidedPlateCase baseline; auto report=baseline.Initialize(wall);
    ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; EXPECT_EQ(baseline.wall_tessellation(),nullptr);
    std::array<GuidedPlateFrame,3> expected;
    report=baseline.Capture(expected[0]); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
    for (unsigned step=1;step<=2;++step) {
        report=baseline.Step(); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
        report=baseline.Capture(expected[step]); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
    }
    TempRoot temporary;
    for (const auto kind:{Kind::Original,Kind::FlipConvexPairs,Kind::UniformFour}) {
        SCOPED_TRACE(WallTessellationName(kind));
        GuidedPlateCase run; report=run.InitializeTessellated(wall,bytes,kind);
        ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
        const auto binding=WallTessellationBindingId(kind); const auto* meta=run.wall_tessellation();
        ASSERT_NE(meta,nullptr); EXPECT_EQ(meta->kind,kind); EXPECT_EQ(meta->source_manifest_sha256,kCanonicalWallManifestSha256);
        EXPECT_EQ(meta->mesh_sha256.size(),64u); EXPECT_EQ(meta->original_vertices,62u); EXPECT_EQ(meta->original_triangles,100u);
        EXPECT_EQ(run.wall_provenance()->model_archive_reference_sha256,wall.provenance().model_archive_reference_sha256);
        EXPECT_EQ(run.wall_mesh().vertex_count,kind==Kind::UniformFour?223u:62u);
        EXPECT_EQ(run.wall_mesh().triangle_count,kind==Kind::UniformFour?400u:100u);
        EXPECT_EQ(meta->faces.size(),run.wall_mesh().triangle_count);
        EXPECT_EQ(meta->midpoints.size(),kind==Kind::UniformFour?161u:0u);
        EXPECT_EQ(meta->flipped_source_quads.size(),kind==Kind::FlipConvexPairs?45u:0u);
        EXPECT_EQ(run.guided_data()->wall_binding_id,binding);
        SameModel(*run.model_data(),*baseline.model_data()); SameReference(run.contact_reference(),baseline.contact_reference());
        EXPECT_EQ(Components(run.guided_data()->pose.rotation),Components(baseline.guided_data()->pose.rotation));
        EXPECT_EQ(Components(run.guided_data()->pose.translation),Components(baseline.guided_data()->pose.translation));
        EXPECT_EQ(run.guided_data()->translation_fixed_bits,baseline.guided_data()->translation_fixed_bits);
        EXPECT_EQ(run.guided_data()->rotation_fixed,baseline.guided_data()->rotation_fixed);
        EXPECT_EQ(run.modal()->step_count,baseline.modal()->step_count); EXPECT_EQ(run.modal()->time_step,baseline.modal()->time_step);
        EXPECT_EQ(run.modal()->horizon,baseline.modal()->horizon); EXPECT_EQ(run.modal()->selected_mode,baseline.modal()->selected_mode);
        EXPECT_EQ(run.modal()->squared_frequency,baseline.modal()->squared_frequency);
        EXPECT_EQ(run.modal()->initial_mode_increment,baseline.modal()->initial_mode_increment);
        EXPECT_EQ(run.modal()->sampled_structural_operator_norm,baseline.modal()->sampled_structural_operator_norm);
        EXPECT_EQ(run.modal()->contact_rate_bound,baseline.modal()->contact_rate_bound);
        EXPECT_EQ(run.modal()->combined_rate_envelope,baseline.modal()->combined_rate_envelope);
        EXPECT_EQ(run.modal()->proposed_step_limit,baseline.modal()->proposed_step_limit);
        EXPECT_EQ(run.diagnostic_stride(),baseline.diagnostic_stride());
        GuidedPlateFrame initial,first,recovered,last;
        report=run.Capture(initial); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; SameFrame(initial,expected[0]);
        EXPECT_EQ(initial.contact_association.wall_binding_id,binding);
        if (kind!=Kind::Original) {
            const auto destination=temporary.path/WallTessellationName(kind);
            EXPECT_ANY_THROW({GuidedPlateArtifacts rejected(destination.string(),bytes,wall,run,100);});
            EXPECT_FALSE(fs::exists(destination)); // Admission before ANY filesystem write.
        }
        const AllocationSnapshot allocations(run); const auto* retained_metadata=run.wall_tessellation();
        report=run.Step(); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
        report=run.Capture(first); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; SameFrame(first,expected[1]);
        EXPECT_EQ(first.contact_association.wall_binding_id,binding); EXPECT_EQ(first.metrics.applied_contact.wall_binding_id,binding);
        const auto visible=run.output()->surface().mesh()->GetCoordsVertices();
        report=run.Step({1e-9}); ASSERT_EQ(report.status,Code::AdmissionFailure)<<report.diagnostic;
        EXPECT_NE(report.diagnostic.find("displacement"),std::string::npos);
        EXPECT_EQ(run.output()->surface().mesh()->GetCoordsVertices(),visible);
        report=run.Capture(recovered); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; SameFrame(recovered,first);
        EXPECT_EQ(recovered.metrics.contact.attempt,first.metrics.contact.attempt);
        EXPECT_EQ(recovered.metrics.applied_contact.attempt,first.metrics.applied_contact.attempt);
        EXPECT_EQ(recovered.contact_association.phase,contact::Q4PlanarContactPhase::AcceptedBase);
        EXPECT_EQ(recovered.element_association.phase,shell::ShellBatchPhase::kAcceptedBase);
        EXPECT_EQ(recovered.contact_association.wall_binding_id,binding);
        EXPECT_EQ(recovered.contact_association.base_epoch,first.stamp.epoch);
        EXPECT_EQ(recovered.contact_association.attempt,recovered.element_association.attempt);
        EXPECT_GT(recovered.contact_association.attempt,first.contact_association.attempt);
        report=run.Step(); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
        report=run.Capture(last); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; SameFrame(last,expected[2]);
        EXPECT_EQ(last.contact_association.wall_binding_id,binding); EXPECT_EQ(last.metrics.applied_contact.wall_binding_id,binding);
        EXPECT_EQ(run.wall_tessellation(),retained_metadata); allocations.Check(run);
    }
    // This is a separated prefix/state/binding gate, not contact response,
    // tessellation invariance after activation, or a complete trajectory.
}

TEST_F(GuidedPlateWallCase, SourceAndLifecycleFailuresPreserveOutputAndAllowCleanInitialization) {
    GuidedPlateCase run; GuidedPlateFrame sentinel; sentinel.stamp.owner_id=923; sentinel.position.fill(17);
    EXPECT_EQ(run.Capture(sentinel).status,Code::NotInitialized);
    EXPECT_EQ(sentinel.stamp.owner_id,923u); for (const auto x:sentinel.position) EXPECT_EQ(x,17);
    auto report=run.InitializeTessellated(wall,bytes+" ",Kind::UniformFour); EXPECT_EQ(report.status,Code::InvalidInput);
    EXPECT_EQ(run.metrics(),nullptr); EXPECT_EQ(run.wall_tessellation(),nullptr); EXPECT_EQ(run.state_allocations().device_bytes,0u);
    report=run.InitializeTessellated(wall,bytes,static_cast<Kind>(255)); EXPECT_EQ(report.status,Code::InvalidInput);
    report=run.InitializeTessellated(wall,bytes,Kind::UniformFour,{3,20}); EXPECT_EQ(report.status,Code::InvalidInput);
    EXPECT_EQ(run.metrics(),nullptr); EXPECT_EQ(run.wall_mesh().vertex_count,0u); EXPECT_EQ(run.contact_reference().parent_count,0u);
    report=run.InitializeTessellated(wall,bytes,Kind::UniformFour); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
    GuidedPlateFrame initial,after;
    report=run.Capture(initial); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic;
    const auto* metadata=run.wall_tessellation(); const AllocationSnapshot allocations(run);
    EXPECT_EQ(run.Initialize(wall).status,Code::AlreadyInitialized);
    EXPECT_EQ(run.InitializeTessellated(wall,"bad",Kind::Original).status,Code::AlreadyInitialized);
    report=run.Capture(after); ASSERT_EQ(report.status,Code::Ok)<<report.diagnostic; SameFrame(after,initial);
    EXPECT_EQ(initial.stamp.owner_id,after.stamp.owner_id); EXPECT_EQ(run.wall_tessellation(),metadata); allocations.Check(run);
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=2) { std::cerr<<"Required: actual canonical wall manifest path\n"; return 2; }
    asset=argv[1]; return RUN_ALL_TESTS();
}
