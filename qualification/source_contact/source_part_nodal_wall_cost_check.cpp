#include "SourceNodalWallTest.h"
#include "SourceContactCudaFixture.h"

namespace crash::qualification::source_contact::nodal::test {
namespace baseline=cuda_test;
static_assert(sizeof(Storage)+sizeof(baseline::Storage)<=2*1024*1024,
              "Candidate and unchanged integral baseline coexist inside the same explicit allocation cap");
namespace {
void Baseline(const SourcePartContactFixture& source,const sc::PlanarWallGeometry& wall,
              const cf::Aggregate& expected,const cf::FixtureMass& mass,baseline::Input& input) {
    input.wall_x=wall.wall_x();
    std::copy(mass.inverse.begin(),mass.inverse.end(),input.inverse_mass);
    std::copy(mass.fixed.begin(),mass.fixed.end(),input.fixed);
    const double shift=cf::WholeShift(source,wall.wall_x());
    const auto position=cf::Shift(source,shift),velocity=cf::Velocity(shift);
    for (unsigned p=0;p<ParentCount;++p) {
        auto& packet=input.parents[p]; packet.arity=source.parents()[p].arity;
        packet.area=expected.parents[p].area; packet.attempt=31;
        std::copy(position.begin(),position.end(),packet.position); std::copy(velocity.begin(),velocity.end(),packet.velocity);
        if (packet.arity==4) ASSERT_TRUE(source.q4_parent(p,packet.quad));
        else {
            ASSERT_TRUE(source.t3_parent(p,packet.triangle));
            ASSERT_EQ(sc::PrepareT3MaterialMeasure(source.positions(),packet.triangle,&packet.triangle_reference),sc::SurfaceMeasureStatus::Ok);
        }
    }
}
void CompareBaseline(const cf::Aggregate& expected,const baseline::Result& actual) {
    for (unsigned p=0;p<ParentCount;++p) {
        const auto& host=expected.parents[p]; ASSERT_TRUE(actual.accepted[p]); SCOPED_TRACE(host.source_id);
        const bool quad=host.arity==4;
        const auto& q=actual.parents[p].quad.integration; const auto& t=actual.parents[p].triangle;
        EXPECT_TRUE(quad?q.valid:t.valid); EXPECT_EQ(quad?q.parent_element_id:t.parent_element_id,host.source_id);
        EXPECT_EQ(quad?q.feature_id:t.feature_id,host.feature_id);
        EXPECT_EQ(quad?q.base_epoch:t.base_epoch,host.base_epoch); EXPECT_EQ(quad?q.attempt:t.attempt,host.attempt);
        const auto force=quad?q.resultant:t.resultant,potential=quad?q.potential:t.potential;
        Cert(host.resultant,force); Cert(host.potential,potential);
        EXPECT_LE(force.error,cf::ForceBudget); EXPECT_LE(potential.error,cf::EnergyBudget);
        const auto area=quad?q.active_area:t.active_area;
        EXPECT_LE(area.lower,host.active_area.upper); EXPECT_GE(area.upper,host.active_area.lower);
        for (unsigned l=0;l<host.arity;++l) {
            Cert(host.magnitude[l],quad?q.force[l]:t.force[l]); Vector(host.force[l],quad?q.nodal.forces[l]:t.nodal.forces[l]);
            EXPECT_EQ(host.nodes[l],quad?q.nodal.nodes[l]:t.nodal.nodes[l]);
        }
    }
}
double Median(std::array<double,5> values) { std::sort(values.begin(),values.end()); return values[2]; }
} // namespace

TEST_F(Check, Original100FaceCoherentGpuMeetsPredeclaredCostAgainstUnchangedIntegral) {
    cw::WallTessellation original; ASSERT_NO_FATAL_FAILURE(Geometry(original)); const auto& geometry=*original.geometry();
    Input input; ASSERT_NO_FATAL_FAILURE(Coherent(geometry,input)); ASSERT_EQ(input.face_count,100u);
    Result expected; ASSERT_EQ(EvaluateHost(prepared,input,&expected).status,Status::Ok);
    cf::Harness host(source); ASSERT_TRUE(host.mass.Initialize(source)); cf::Aggregate integral;
    const double shift=cf::WholeShift(source,geometry.wall_x());
    ASSERT_TRUE(host.EvaluateAll(source.coordinates(),cf::Shift(source,shift),cf::Velocity(shift),geometry,&integral))<<host.diagnostic;
    long double integral_force=0,integral_potential=0,force_error=0,energy_error=0;
    for (const auto& parent:integral.parents) {
        integral_force+=parent.resultant.value; integral_potential+=parent.potential.value;
        force_error+=parent.resultant.error; energy_error+=parent.potential.error;
    }
    EXPECT_GE(static_cast<long double>(expected.contact.resultant.value)+expected.contact.resultant.error,
              integral_force-force_error);
    EXPECT_GE(static_cast<long double>(expected.contact.potential.value)+expected.contact.potential.error,
              integral_potential-energy_error);
    auto old_input=std::make_unique<baseline::Input>(); ASSERT_NO_FATAL_FAILURE(Baseline(source,geometry,integral,host.mass,*old_input));
    auto old_result=std::make_unique<baseline::Result>();
    std::size_t free_before=0,total=0,free_objects=0,free_warm=0,free_final=0;
    ASSERT_EQ(cudaMemGetInfo(&free_before,&total),cudaSuccess); // Explicitly after CUDA context creation.
    Device device; ASSERT_EQ(device.initialization_status(),cudaSuccess);
    baseline::Device old_device; ASSERT_EQ(old_device.initialization_status(),cudaSuccess);
    ASSERT_EQ(cudaMemGetInfo(&free_objects,&total),cudaSuccess);
    Result actual; Report report; Timing timing; float old_ms=0;
    ASSERT_EQ(old_device.Evaluate(*old_input,*old_result,old_ms),cudaSuccess);
    ASSERT_NO_FATAL_FAILURE(CompareBaseline(integral,*old_result));
    ASSERT_EQ(device.Evaluate(input,actual,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
    ASSERT_NO_FATAL_FAILURE(Compare(expected,actual));
    const Result warmed=actual;
    ASSERT_EQ(cudaMemGetInfo(&free_warm,&total),cudaSuccess);
    std::array<double,5> nodal_ms{},integral_ms{},end_to_end{};
    for (unsigned run=0;run<5;++run) {
        ASSERT_EQ(old_device.Evaluate(*old_input,*old_result,old_ms),cudaSuccess);
        ASSERT_NO_FATAL_FAILURE(CompareBaseline(integral,*old_result)); integral_ms[run]=old_ms;
        ASSERT_EQ(device.Evaluate(input,actual,report,timing),cudaSuccess); ASSERT_EQ(report.status,Status::Ok);
        ASSERT_NO_FATAL_FAILURE(Compare(expected,actual));
        ASSERT_NO_FATAL_FAILURE(ExactResult(warmed,actual));
        nodal_ms[run]=timing.kernel_ms; end_to_end[run]=timing.checked_end_to_end_ms;
        ASSERT_GT(nodal_ms[run],0); ASSERT_GT(integral_ms[run],0);
        RecordProperty("nodal_event_ms_"+std::to_string(run),Number(nodal_ms[run]));
        RecordProperty("integral_event_ms_"+std::to_string(run),Number(integral_ms[run]));
        RecordProperty("checked_end_to_end_ms_"+std::to_string(run),Number(end_to_end[run]));
    }
    ASSERT_EQ(cudaMemGetInfo(&free_final,&total),cudaSuccess);
    const double median=Median(nodal_ms),maximum=*std::max_element(nodal_ms.begin(),nodal_ms.end());
    const double baseline_median=Median(integral_ms),speedup=baseline_median/median;
    // Frozen engineering gate for the real ORIGINAL100-face coherent profile.
    // Other two tessellations have separate numerical/identity gates and timings.
    EXPECT_LE(median,2.0); EXPECT_LE(maximum,5.0); EXPECT_GE(speedup,20.0);
    RecordProperty("nodal_event_median_ms",Number(median)); RecordProperty("nodal_event_max_ms",Number(maximum));
    RecordProperty("unchanged_integral_event_median_ms",Number(baseline_median)); RecordProperty("event_speedup",Number(speedup));
    RecordProperty("checked_end_to_end_median_ms",Number(Median(end_to_end)));
    RecordProperty("candidate_owned_bytes",static_cast<int>(sizeof(Storage)));
    RecordProperty("baseline_owned_bytes",static_cast<int>(sizeof(baseline::Storage)));
    RecordProperty("simultaneous_owned_bytes",static_cast<int>(sizeof(Storage)+sizeof(baseline::Storage)));
    RecordProperty("device_free_after_context_before_objects_bytes",std::to_string(free_before));
    RecordProperty("device_free_after_objects_before_warm_bytes",std::to_string(free_objects));
    RecordProperty("device_free_after_warm_bytes",std::to_string(free_warm));
    RecordProperty("device_free_after_measured_calls_bytes",std::to_string(free_final));
    RecordProperty("memory_scope","device-wide readings include context and BOTH compiled kernels; owned bytes are separate");
    RecordProperty("coherent_force_model_difference_N",Number(static_cast<double>(expected.contact.resultant.value-integral_force)));
    RecordProperty("coherent_potential_model_difference_J",Number(static_cast<double>(expected.contact.potential.value-integral_potential)));
    RecordProperty("timing_scope","one warm plus five checked calls each; candidate includes face query/law/device reduction; integral retains prior raw-kernel scope");
    Metadata();
}
} // namespace crash::qualification::source_contact::nodal::test
