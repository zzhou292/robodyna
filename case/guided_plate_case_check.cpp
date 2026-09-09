#include "GuidedPlateCase.h"
#include "chrono/core/ChQuaternion.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <tuple>

namespace {
using namespace crash::case_data;
namespace ref=crash::reference;
namespace shell=tl::fea::reissner;
namespace contact=tlfea::contact;
using Vec=chrono::ChVector3d;
using Frame=GuidedPlateFrame;
using Code=GuidedPlateStatus;
std::string asset;

class GuidedPlate : public ::testing::Test {
  protected:
    CanonicalWall wall;
    void SetUp() override {
        int devices=0; ASSERT_EQ(cudaGetDeviceCount(&devices),cudaSuccess); ASSERT_GT(devices,0);
        const auto loaded=wall.LoadFile(asset); ASSERT_EQ(loaded.status,WallStatus::Ok)<<loaded.message;
    }
};
std::string Precise(double value) { std::ostringstream text; text.precision(17); text<<value; return text.str(); }
Vec Read(const std::array<double,3*ref::kCouponNodes>& value,std::size_t n) { return {value[3*n],value[3*n+1],value[3*n+2]}; }
Vec ToVector(shell::Vec3 value) { return {value.x,value.y,value.z}; }
Vec ToVector(contact::Vec3 value) { return {value.x,value.y,value.z}; }
double Tip(const Frame& frame) { return .5*(frame.position[12]+frame.position[15]); }
void ExpectVector(const Vec& actual,const Vec& expected,double absolute,double relative=1e-9) {
    EXPECT_LE((actual-expected).Length(),absolute+relative*expected.Length());
}
void ExpectGuides(const Frame& frame,const Frame& initial) {
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        for (unsigned axis:{1u,2u}) {
            EXPECT_EQ(frame.position[3*n+axis],initial.position[3*n+axis]); EXPECT_EQ(frame.velocity[3*n+axis],0);
        }
        if (n==1 || n==2) {
            EXPECT_EQ(frame.position[3*n],initial.position[3*n]); EXPECT_EQ(frame.velocity[3*n],0);
            for (unsigned axis=0;axis<3;++axis) EXPECT_EQ(frame.omega[3*n+axis],0);
            for (unsigned c=0;c<4;++c) EXPECT_EQ(frame.rotation[4*n+c],initial.rotation[4*n+c]);
        }
    }
}
auto WorkValues(const GuidedPlateWorkReport& w) {
    return std::make_tuple(w.kinetic_energy,w.total_energy,w.relative_energy_error,w.combined_kinetic_residual,
        w.kinetic_arithmetic_budget,w.shell_coordinate_work_budget,w.contact_defect_lower_limit,w.contact_defect_upper_limit);
}
auto ContactValues(const contact::Q4PlanarContactDiagnostics& d) {
    return std::make_tuple(d.potential.value,d.potential.lower,d.potential.upper,d.potential.error,
        d.active_area.lower,d.active_area.upper,d.maximum_penetration,d.force_on_surface.x,d.wall_reaction.x,
        d.wall_moment.y,d.wall_moment.z,d.surface_power,d.base_potential,d.base_potential_error,d.potential_increment,
        d.kinetic_midpoint_work,d.kinetic_midpoint_roundoff,d.force_coordinate_work,d.force_coordinate_roundoff,
        d.conservative_force_coordinate_defect,d.continuum_work_uncertainty,d.quadratic_work_upper);
}
void ExpectAcceptedEqual(const Frame& a,const Frame& b,bool same_owner) {
    EXPECT_EQ(a.position,b.position); EXPECT_EQ(a.velocity,b.velocity); EXPECT_EQ(a.rotation,b.rotation); EXPECT_EQ(a.omega,b.omega);
    EXPECT_EQ(a.reaction_force,b.reaction_force); EXPECT_EQ(a.reaction_couple,b.reaction_couple);
    EXPECT_EQ(a.stamp.epoch,b.stamp.epoch); EXPECT_EQ(a.stamp.time,b.stamp.time); EXPECT_EQ(a.stamp.fixed_dt,b.stamp.fixed_dt);
    EXPECT_EQ(a.stamp.reactions_valid,b.stamp.reactions_valid); EXPECT_EQ(a.stamp.reaction_base_epoch,b.stamp.reaction_base_epoch);
    EXPECT_EQ(a.stamp.reaction_time,b.stamp.reaction_time);
    const auto& am=a.metrics; const auto& bm=b.metrics;
    EXPECT_EQ(WorkValues(am.work),WorkValues(bm.work)); EXPECT_EQ(ContactValues(am.contact),ContactValues(bm.contact));
    EXPECT_EQ(ContactValues(am.applied_contact),ContactValues(bm.applied_contact));
    EXPECT_EQ(am.initial_energy,bm.initial_energy); EXPECT_EQ(am.maximum_relative_energy_error,bm.maximum_relative_energy_error);
    EXPECT_EQ(am.peak_penetration,bm.peak_penetration); ExpectVector(ToVector(am.wall_impulse),ToVector(bm.wall_impulse),0,0);
    ExpectVector(ToVector(am.wall_moment_impulse),ToVector(bm.wall_moment_impulse),0,0);
    EXPECT_EQ(am.shell_midpoint_work,bm.shell_midpoint_work); EXPECT_EQ(am.contact_midpoint_work,bm.contact_midpoint_work);
    EXPECT_EQ(am.shell_coordinate_work,bm.shell_coordinate_work); EXPECT_EQ(am.contact_coordinate_work,bm.contact_coordinate_work);
    EXPECT_EQ(am.last_operator_norm,bm.last_operator_norm); EXPECT_EQ(am.last_operator_epoch,bm.last_operator_epoch);
    EXPECT_EQ(am.required_steps,bm.required_steps); EXPECT_EQ(am.full_state_audit_reads,bm.full_state_audit_reads);
    EXPECT_EQ(am.shell.base_epoch,bm.shell.base_epoch); EXPECT_EQ(am.shell.phase,bm.shell.phase);
    EXPECT_EQ(am.contact.wall_binding_id,bm.contact.wall_binding_id);
    if (same_owner) {
        EXPECT_EQ(a.stamp.owner_id,b.stamp.owner_id); EXPECT_EQ(am.shell.attempt,bm.shell.attempt);
        EXPECT_EQ(am.contact.attempt,bm.contact.attempt); EXPECT_EQ(am.applied_contact.attempt,bm.applied_contact.attempt);
    }
    for (unsigned e=0;e<ref::kCouponElements;++e) {
        EXPECT_EQ(a.element[e].energy,b.element[e].energy); EXPECT_EQ(a.element[e].bending_energy,b.element[e].bending_energy);
        EXPECT_EQ(a.parent[e].integration.potential.value,b.parent[e].integration.potential.value);
        for (unsigned n=0;n<4;++n) {
            ExpectVector(ToVector(a.element[e].force[n]),ToVector(b.element[e].force[n]),0,0);
            ExpectVector(ToVector(a.element[e].couple[n]),ToVector(b.element[e].couple[n]),0,0);
            ExpectVector(ToVector(a.parent[e].integration.nodal.forces[n]),ToVector(b.parent[e].integration.nodal.forces[n]),0,0);
            for (unsigned c=0;c<12;++c) {
                EXPECT_EQ(a.element[e].strain[n][c],b.element[e].strain[n][c]);
                EXPECT_EQ(a.element[e].resultant[n][c],b.element[e].resultant[n][c]);
            }
        }
    }
}
struct Allocations {
    tl::fea::NodalAllocationInfo state,element,contact;
    explicit Allocations(const GuidedPlateCase& run) : state(run.state_allocations()),element(run.element_allocations()),contact(run.contact_allocations()) {}
    void Check(const GuidedPlateCase& run) const {
        const Allocations current(run);
        EXPECT_EQ(current.state.device_bytes,state.device_bytes); EXPECT_EQ(current.state.device_allocations,state.device_allocations);
        EXPECT_EQ(current.element.device_bytes,element.device_bytes); EXPECT_EQ(current.element.device_allocations,element.device_allocations);
        EXPECT_EQ(current.contact.device_bytes,contact.device_bytes); EXPECT_EQ(current.contact.device_allocations,contact.device_allocations);
        EXPECT_LE(current.state.device_bytes+current.element.device_bytes+current.contact.device_bytes,kGuidedPlateDeviceBudget);
    }
};
struct Ledger { Vec linear,angular; double translation=0,physical=0,drilling=0; };
Ledger Measure(const Frame& frame,const ref::ElasticCouponData& data) {
    Ledger result;
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        const auto& mass=data.nodal_mass[n]; const auto x=Read(frame.position,n),v=Read(frame.velocity,n),omega=Read(frame.omega,n);
        result.linear+=mass.mass*v;
        result.angular+=chrono::Vcross(x,mass.mass*v)+mass.physical_tangential_inertia*omega;
        shell::Quaternion offset; bool found=false;
        for (unsigned e=0;e<ref::kCouponElements && !found;++e) for (unsigned local=0;local<4;++local)
            if (data.connectivity[e][local]==n) { offset=data.reference[e].node_frame_offset[local]; found=true; break; }
        const chrono::ChQuaterniond q(frame.rotation[4*n],frame.rotation[4*n+1],frame.rotation[4*n+2],frame.rotation[4*n+3]);
        const auto director=(q*chrono::ChQuaterniond(offset.w,offset.x,offset.y,offset.z)).Rotate(Vec(0,0,1));
        const auto spin=omega.Dot(director);
        result.translation+=.5*mass.mass*v.Length2();
        result.physical+=.5*mass.physical_tangential_inertia*chrono::Vcross(omega,director).Length2();
        result.drilling+=.5*mass.artificial_drilling_inertia*spin*spin;
    }
    return result;
}
void CheckInterval(const Frame& before,const Frame& after,const ref::ElasticCouponData& data) {
    const double h=after.stamp.fixed_dt;
    std::array<Vec,ref::kCouponNodes> forces{},couples{},shell_forces{},contact_forces{};
    for (unsigned e=0;e<ref::kCouponElements;++e) for (unsigned local=0;local<4;++local) {
        const auto n=data.connectivity[e][local];
        forces[n]+=ToVector(before.element[e].force[local])+ToVector(before.parent[e].integration.nodal.forces[local]);
        shell_forces[n]+=ToVector(before.element[e].force[local]);
        contact_forces[n]+=ToVector(before.parent[e].integration.nodal.forces[local]);
        couples[n]+=ToVector(before.element[e].couple[local]);
    }
    // Independent numerical-load work oracle. Read BASE element/parent forces
    // and accepted endpoint kinematics; do not reuse either module's work law or
    // its reported increments. Chrono's quaternion log is independent of TL's
    // matrix rotation-vector calculation and gives the WORLD spin increment.
    enum WorkTerm { ShellMidpoint, ContactMidpoint, ShellCoordinate, ContactCoordinate, WorkCount };
    std::array<long double,WorkCount> work{},magnitude{};
    long double couple_magnitude=0;
    auto add=[&](WorkTerm which,long double value) { work[which]+=value; magnitude[which]+=std::abs(value); };
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        const chrono::ChQuaterniond q0(before.rotation[4*n],before.rotation[4*n+1],
                                      before.rotation[4*n+2],before.rotation[4*n+3]);
        const chrono::ChQuaterniond q1(after.rotation[4*n],after.rotation[4*n+1],
                                      after.rotation[4*n+2],after.rotation[4*n+3]);
        auto relative=q1*q0.GetConjugate();
        if (relative.e0()<0) relative=-relative;
        const auto spin=relative.GetRotVec();
        for (unsigned axis=0;axis<3;++axis) {
            const auto offset=3*n+axis;
            const long double mean_v=.5L*(static_cast<long double>(before.velocity[offset])+after.velocity[offset]);
            const long double mean_w=.5L*(static_cast<long double>(before.omega[offset])+after.omega[offset]);
            const long double dx=static_cast<long double>(after.position[offset])-before.position[offset];
            add(ShellMidpoint,static_cast<long double>(h)*shell_forces[n][axis]*mean_v);
            add(ShellMidpoint,static_cast<long double>(h)*couples[n][axis]*mean_w);
            add(ContactMidpoint,static_cast<long double>(h)*contact_forces[n][axis]*mean_v);
            add(ShellCoordinate,static_cast<long double>(shell_forces[n][axis])*dx);
            add(ShellCoordinate,static_cast<long double>(couples[n][axis])*spin[axis]);
            add(ContactCoordinate,static_cast<long double>(contact_forces[n][axis])*dx);
            couple_magnitude+=std::abs(static_cast<long double>(couples[n][axis]));
        }
    }
    constexpr long double eps=std::numeric_limits<double>::epsilon();
    std::array<double,WorkCount> budget{};
    budget[ShellMidpoint]=static_cast<double>(128*eps*std::max(magnitude[ShellMidpoint],
                                                                             static_cast<long double>(after.metrics.initial_energy)));
    // Unit rotations in the admitted small-step chart incur O(eps) absolute
    // spin error under independent quaternion/matrix reconstruction. Multiplying
    // that by the captured couples supplies its dimensionally consistent work budget.
    budget[ShellCoordinate]=static_cast<double>(128*eps*(std::max(magnitude[ShellCoordinate],
                                                                static_cast<long double>(after.metrics.initial_energy))+couple_magnitude));
    budget[ContactMidpoint]=static_cast<double>(128*eps*magnitude[ContactMidpoint])+after.metrics.contact.kinetic_midpoint_roundoff;
    budget[ContactCoordinate]=static_cast<double>(128*eps*magnitude[ContactCoordinate])+after.metrics.contact.force_coordinate_roundoff;
    const std::array<double,WorkCount> reported{after.metrics.shell.kinetic_midpoint_work,
        after.metrics.contact.kinetic_midpoint_work,after.metrics.shell.force_coordinate_work,
        after.metrics.contact.force_coordinate_work};
    const std::array<double,WorkCount> accumulated_before{before.metrics.shell_midpoint_work,
        before.metrics.contact_midpoint_work,before.metrics.shell_coordinate_work,before.metrics.contact_coordinate_work};
    const std::array<double,WorkCount> accumulated_after{after.metrics.shell_midpoint_work,
        after.metrics.contact_midpoint_work,after.metrics.shell_coordinate_work,after.metrics.contact_coordinate_work};
    for (unsigned term=0;term<WorkCount;++term) {
        SCOPED_TRACE(::testing::Message()<<"independent work term="<<term);
        EXPECT_NEAR(reported[term],static_cast<double>(work[term]),budget[term]);
        const double sum_roundoff=static_cast<double>(4*eps*(std::abs(static_cast<long double>(accumulated_before[term]))+
                                                            std::abs(static_cast<long double>(accumulated_after[term]))));
        EXPECT_NEAR(accumulated_after[term]-accumulated_before[term],static_cast<double>(work[term]),budget[term]+sum_roundoff);
    }
    Vec reactions,moments; double constraint_work=0;
    for (unsigned n=0;n<ref::kCouponNodes;++n) {
        const auto reaction=Read(after.reaction_force,n),couple=Read(after.reaction_couple,n);
        const unsigned mask=data.fixed[n]?7:6;
        for (unsigned axis=0;axis<3;++axis)
            EXPECT_NEAR(reaction[axis],(mask&(1u<<axis))?-forces[n][axis]:0,1e-11);
        ExpectVector(couple,data.fixed[n]?-couples[n]:Vec(0),1e-12);
        reactions+=reaction; moments+=chrono::Vcross(Read(before.position,n),reaction)+couple;
        constraint_work+=reaction.Dot(Read(after.position,n)-Read(before.position,n))+h*couple.Dot(Read(after.omega,n));
    }
    const auto a=Measure(before,data),b=Measure(after,data);
    const auto& applied=after.metrics.applied_contact;
    EXPECT_EQ(applied.force_on_surface.x,before.contact_association.force_on_surface.x);
    EXPECT_EQ(applied.wall_moment.y,before.contact_association.wall_moment.y);
    EXPECT_EQ(applied.wall_moment.z,before.contact_association.wall_moment.z);
    EXPECT_EQ(after.metrics.contact.base_potential,before.contact_association.potential.value);
    ExpectVector(b.linear-a.linear,h*(reactions-ToVector(applied.wall_reaction)),1e-12);
    ExpectVector(b.angular-a.angular,h*(moments-ToVector(applied.wall_moment)),1e-13);
    ExpectVector(ToVector(after.metrics.wall_impulse)-ToVector(before.metrics.wall_impulse),h*ToVector(applied.wall_reaction),1e-16);
    ExpectVector(ToVector(after.metrics.wall_moment_impulse)-ToVector(before.metrics.wall_moment_impulse),h*ToVector(applied.wall_moment),1e-16);
    EXPECT_EQ(constraint_work,0); EXPECT_EQ(applied.phase,contact::Q4PlanarContactPhase::AcceptedBase);
    EXPECT_EQ(applied.base_epoch,before.stamp.epoch); EXPECT_EQ(after.stamp.reaction_base_epoch,before.stamp.epoch);
    EXPECT_EQ(after.stamp.reaction_time,before.stamp.time);
    EXPECT_NEAR(b.translation,after.metrics.shell.kinetic_translation,1e-15);
    EXPECT_NEAR(b.physical,after.metrics.shell.kinetic_physical_rotation,1e-15);
    EXPECT_NEAR(b.drilling,after.metrics.shell.kinetic_artificial_drilling,1e-15);
    const double change=b.translation+b.physical+b.drilling-a.translation-a.physical-a.drilling;
    const double midpoint=after.metrics.shell.kinetic_midpoint_work+after.metrics.contact.kinetic_midpoint_work;
    EXPECT_NEAR(change,midpoint,after.metrics.work.kinetic_arithmetic_budget+1e-16);
}

TEST_F(GuidedPlate, InvalidLifecycleConfigurationAndStopRequestsPreservePublication) {
    GuidedPlateCase run; CanonicalWall empty; Frame sentinel; sentinel.position[0]=123;
    EXPECT_EQ(run.Step().status,Code::NotInitialized); EXPECT_EQ(run.Capture(sentinel).status,Code::NotInitialized);
    EXPECT_EQ(sentinel.position[0],123); EXPECT_EQ(run.Initialize(empty).status,Code::InvalidInput);
    for (const GuidedPlateConfig config:{GuidedPlateConfig{0,20},{3,20},{1,0},{1,65}}) {
        EXPECT_EQ(run.Initialize(wall,config).status,Code::InvalidInput); EXPECT_EQ(run.metrics(),nullptr);
        EXPECT_EQ(run.output(),nullptr); EXPECT_EQ(run.state_allocations().device_bytes,0u);
        EXPECT_EQ(run.element_allocations().device_bytes,0u); EXPECT_EQ(run.contact_allocations().device_bytes,0u);
    }
    auto r=run.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    Frame before,after; r=run.Capture(before); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    for (double limit:{0.,-.001,.005,std::numeric_limits<double>::quiet_NaN()})
        EXPECT_EQ(run.Step({limit}).status,Code::InvalidInput);
    r=run.Capture(after); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; ExpectAcceptedEqual(after,before,true);
    EXPECT_EQ(run.Initialize(wall).status,Code::AlreadyInitialized);
}

TEST_F(GuidedPlate, HundredActualStepsPreserveAllocationsGuidesAndAcceptedMeshCadence) {
    std::size_t free_before=0,total_before=0,free_initialized=0,total_initialized=0,free_after=0,total_after=0;
    ASSERT_EQ(cudaMemGetInfo(&free_before,&total_before),cudaSuccess);
    GuidedPlateCase run; auto r=run.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    ASSERT_EQ(cudaMemGetInfo(&free_initialized,&total_initialized),cudaSuccess);
    Frame initial,after; r=run.Capture(initial); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const Allocations allocations(run); ASSERT_GT(run.metrics()->required_steps,100u);
    const auto source=run.wall_mesh(); ASSERT_EQ(source.vertex_count,62u); ASSERT_EQ(source.triangle_count,100u);
    EXPECT_EQ(run.wall_provenance()->wall_sha256,wall.provenance().wall_sha256);
    const auto start=std::chrono::steady_clock::now();
    for (unsigned step=1;step<=100;++step) {
        r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic<<" step="<<step;
        EXPECT_EQ(run.metrics()->stamp.epoch,step); EXPECT_EQ(run.metrics()->shell.base_epoch,step-1);
        EXPECT_EQ(run.metrics()->contact.base_epoch,step-1); EXPECT_EQ(run.metrics()->contact.attempt,run.metrics()->shell.attempt);
        EXPECT_EQ(run.metrics()->contact.wall_binding_id,kGuidedPlateWallBinding);
        EXPECT_EQ(run.metrics()->shell.configuration_id,kGuidedPlateQualification);
        EXPECT_EQ(run.metrics()->contact.potential.value,0); EXPECT_EQ(run.metrics()->full_state_audit_reads,0u);
        EXPECT_EQ(run.output()->surface().frame()->epoch,0u);
    }
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    ASSERT_EQ(cudaMemGetInfo(&free_after,&total_after),cudaSuccess);
    r=run.Capture(after); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; ExpectGuides(after,initial); allocations.Check(run);
    EXPECT_GT(Tip(after),Tip(initial)+1e-9); EXPECT_NE(after.rotation,initial.rotation);
    EXPECT_GT(after.metrics.work.kinetic_energy,0); EXPECT_LE(after.metrics.maximum_relative_energy_error,.01);
    EXPECT_NEAR(after.stamp.time,100*after.stamp.fixed_dt,1e-14);
    const auto& mesh=run.output()->surface(); ASSERT_EQ(mesh.frame()->epoch,100u); ASSERT_EQ(mesh.binding()->triangles.size(),4u);
    for (unsigned n=0;n<ref::kCouponNodes;++n) ExpectVector(mesh.mesh()->GetCoordsVertices()[n],Read(after.position,n),0,0);
    for (unsigned e=0;e<ref::kCouponElements;++e) {
        EXPECT_EQ(mesh.binding()->triangles[2*e].element,run.guided_data()->parents[e].parent_element_id);
        EXPECT_EQ(mesh.binding()->triangles[2*e+1].subtriangle,1u);
    }
    RecordProperty("hundred_steps_wall_seconds",Precise(elapsed)); RecordProperty("time_step",Precise(after.stamp.fixed_dt));
    RecordProperty("executed_steps",100);
    RecordProperty("explicit_device_bytes",std::to_string(allocations.state.device_bytes+allocations.element.device_bytes+allocations.contact.device_bytes));
    RecordProperty("explicit_device_allocations",std::to_string(allocations.state.device_allocations+allocations.element.device_allocations+allocations.contact.device_allocations));
    // Device-wide samples include runtime/driver and unrelated allocations;
    // these are sampled free bytes, not an attributed module peak measurement.
    RecordProperty("device_free_bytes_before_setup",std::to_string(free_before));
    RecordProperty("device_free_bytes_after_setup",std::to_string(free_initialized));
    RecordProperty("device_free_bytes_after_100_steps",std::to_string(free_after));
    EXPECT_EQ(total_initialized,total_before); EXPECT_EQ(total_after,total_before);
    RecordProperty("device_total_bytes",std::to_string(total_after));
    RecordProperty("maximum_energy_error",Precise(after.metrics.maximum_relative_energy_error));
}

TEST_F(GuidedPlate, LateRejectionRefreshesBothAcceptedResultsAndPreservesRetry) {
    GuidedPlateCase run,clean; auto r=run.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    for (unsigned step=0;step<4;++step) {
        r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
        r=clean.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    }
    Frame before,recovered,retried,baseline;
    r=run.Capture(before); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto shown=run.output()->surface().mesh()->GetCoordsVertices(); const Allocations allocations(run);
    r=run.Step({1e-9}); EXPECT_EQ(r.status,Code::AdmissionFailure)<<r.diagnostic;
    EXPECT_NE(r.diagnostic.find("displacement"),std::string::npos);
    EXPECT_EQ(run.output()->surface().mesh()->GetCoordsVertices(),shown);
    r=run.Capture(recovered); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; ExpectAcceptedEqual(recovered,before,true);
    EXPECT_EQ(recovered.element_association.phase,shell::ShellBatchPhase::kAcceptedBase);
    EXPECT_EQ(recovered.contact_association.phase,contact::Q4PlanarContactPhase::AcceptedBase);
    EXPECT_EQ(recovered.element_association.base_epoch,before.stamp.epoch);
    EXPECT_EQ(recovered.contact_association.attempt,recovered.element_association.attempt);
    EXPECT_GT(recovered.element_association.attempt,before.element_association.attempt);
    EXPECT_EQ(recovered.metrics.contact.phase,contact::Q4PlanarContactPhase::PreparedCandidate);
    r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=run.Capture(retried); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Capture(baseline); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; ExpectAcceptedEqual(retried,baseline,false);
    allocations.Check(run);
}

TEST_F(GuidedPlate, CoupledImpulseGuideReactionsAndPhysicalArtificialEnergyUseOneMassSpace) {
    GuidedPlateCase run; auto r=run.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    Frame initial,before,after; r=run.Capture(initial); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; before=initial;
    for (unsigned step=0;step<12;++step) {
        r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
        r=run.Capture(after); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
        CheckInterval(before,after,*run.model_data()); ExpectGuides(after,initial); before=after;
    }
    const auto energy=Measure(after,*run.model_data());
    EXPECT_NEAR(energy.translation+energy.physical+energy.drilling,
                after.metrics.shell_midpoint_work+after.metrics.contact_midpoint_work,2e-15);
    EXPECT_GT(energy.translation,0); EXPECT_GT(energy.physical,0);
}

TEST_F(GuidedPlate, FullStateOperatorReadOccursOnlyAtDeclaredAuditBoundary) {
    GuidedPlateCase run; auto r=run.Initialize(wall,{1,64}); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto stride=(run.modal()->step_count+63)/64; ASSERT_LE(stride,2048u);
    for (std::uint64_t step=1;step<stride;++step) {
        r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
        EXPECT_EQ(run.metrics()->full_state_audit_reads,0u);
    }
    r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    EXPECT_EQ(run.metrics()->full_state_audit_reads,1u); EXPECT_EQ(run.metrics()->last_operator_epoch,stride);
    EXPECT_LE(run.metrics()->last_operator_norm,run.modal()->monitored_structural_norm_limit);
    EXPECT_EQ(run.output()->surface().frame()->epoch,0u);
}

// Opt in only after the guarded 100-step cost gate. Stop at the first certified
// nonzero applied contact, capped at 3/4 of the declared horizon (about 0.15s),
// then exercise only one rejected/retried contact interval. This is not D3.
TEST_F(GuidedPlate, DISABLED_FirstPartialContactKeepsBaseImpulseAndRollbackCorrect) {
    GuidedPlateCase run,clean; auto r=run.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Initialize(wall); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    const auto cap=3*run.metrics()->required_steps/4; bool found=false;
    const Allocations allocations(run); const auto start=std::chrono::steady_clock::now();
    for (std::uint64_t step=0;step<cap && !found;++step) {
        r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic<<" epoch="<<run.metrics()->stamp.epoch;
        r=clean.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
        const auto& applied=run.metrics()->applied_contact;
        found=applied.force_on_surface.x < -applied.force_error.x;
    }
    ASSERT_TRUE(found)<<"No certified applied contact before the frozen prefix cap="<<cap;
    Frame before,recovered,after,baseline;
    r=run.Capture(before); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    EXPECT_GT(before.metrics.wall_impulse.x,0); EXPECT_GT(before.metrics.contact.potential.value,0);
    bool partial=false,nonuniform=false;
    for (const auto& parent:before.parent) if (parent.integration.resultant.value>0) {
        partial|=parent.integration.active_area.upper<.01;
        nonuniform|=std::abs(parent.integration.force[0].value-parent.integration.force[1].value)>
                    parent.integration.force[0].error+parent.integration.force[1].error;
    }
    EXPECT_TRUE(partial); EXPECT_TRUE(nonuniform);
    r=run.Step({1e-9}); EXPECT_EQ(r.status,Code::AdmissionFailure)<<r.diagnostic;
    r=run.Capture(recovered); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic; ExpectAcceptedEqual(recovered,before,true);
    r=run.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Step(); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=run.Capture(after); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    r=clean.Capture(baseline); ASSERT_EQ(r.status,Code::Ok)<<r.diagnostic;
    CheckInterval(before,after,*run.model_data()); ExpectAcceptedEqual(after,baseline,false); allocations.Check(run);
    RecordProperty("first_applied_contact_epoch",std::to_string(before.stamp.epoch));
    RecordProperty("first_applied_contact_time",Precise(before.stamp.time));
    RecordProperty("first_contact_prefix_seconds",Precise(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()));
    RecordProperty("peak_penetration",Precise(after.metrics.peak_penetration));
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=2) { std::cerr<<"Required canonical wall manifest argument\n"; return 2; }
    asset=argv[1]; return RUN_ALL_TESTS();
}
