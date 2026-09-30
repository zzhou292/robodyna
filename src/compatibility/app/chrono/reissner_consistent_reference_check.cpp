// Connected force qualification. Primitive mean derivatives alone cannot
// qualify the existing DRot/Elle, ANS, section-resultant and local-couple path.
#include "ReissnerReferenceFixture.h"
#include <exception>
#ifndef CH_REISSNER_CONSISTENT_FRAME_REFERENCE
#error "This test requires the isolated opt-in consistent-force Chrono core"
#endif

namespace crash::qualification {
namespace {
Frames Noncoaxial() {
    auto frames=Bending(.12,.002);
    for(int n=0;n<4;++n) {
        const auto p=Neutral().x[n];
        frames.x[n].y() += .008*p.x()*p.y();
        frames.x[n].z() += .015*p.x()*p.y();
        frames.q[n]=frames.q[n]*Rotation(-.08*p.y(),Vec(1,0,0))*Rotation(.025*p.x()*p.y(),Vec(0,0,1));
    }
    return frames;
}

// A second independent system, initialized exactly once. Reuse the original
// fixture's setup/measurement instead of duplicating an element harness or
// recapturing the reference of the test's already initialized baseline.
class OffsetReference final : public ReissnerReference {
  public:
    explicit OffsetReference(const Frames& reference) : ReissnerReference(reference) {}
    void Initialize() { ReissnerReference::SetUp(); }
    Evaluation Sample(const Frames& frames) { return Evaluate(frames); }
    void CheckReference() { ReferenceUnchanged(); }

  private:
    void TestBody() override {}
};
}  // namespace

TEST_F(ReissnerReference, NoncoaxialBendingAndTwistRemainObjectiveUnderLargeCommonRotations) {
    const auto frames=Noncoaxial();const auto before=Evaluate(frames);
    ASSERT_TRUE(before.finite);ASSERT_GT(before.bending_energy,1e-4);
    for (const auto& q : {Rotation(1.7,Vec(1,2,-1)),Rotation(2.8,Vec(-2,1,.3)),Rotation(-2.2,Vec(.4,-1,2))}) {
        const auto moved=Transform(frames,q,Vec(2.1,-1.3,.7));const auto after=Evaluate(moved);
        ASSERT_TRUE(after.finite);Covariant(before,after,q,"noncoaxial_superposition");
        Balance(after,moved,"noncoaxial_superposition");
    }
    const auto restored=Evaluate(frames);Covariant(before,restored,chrono::QUNIT,"noncoaxial_restored");
    ReferenceUnchanged();
}

TEST_F(ReissnerReference, AllTwentyFourLocalDofForcesDifferentiateElasticEnergyBeforeAndAfterRotation) {
    const auto initial=Noncoaxial();
    for (const auto& frames : {initial,Transform(initial,Rotation(1.7,Vec(1,2,-1)),Vec(2.1,-1.3,.7))}) {
        const auto base=Evaluate(frames);ASSERT_TRUE(base.finite);
        double largest_error=0,largest_limit=0;
        for (int node=0;node<4;++node) for (int dof=0;dof<6;++dof) {
            SCOPED_TRACE(node);
            SCOPED_TRACE(dof);
            Vec axis(0,0,0);axis[dof%3]=1;
            const double restoring=dof<3 ? base.force[node][dof] : base.couple[node].Dot(frames.q[node].Rotate(axis));
            double errors[2];int level=0;
            for (double h : {2e-5,1e-5}) {
                double energy[2];
                for (int sign=0;sign<2;++sign) {
                    const double delta=sign ? h : -h;auto perturbed=frames;
                    if (dof<3) perturbed.x[node][dof]+=delta;
                    else perturbed.q[node]=frames.q[node]*Rotation(delta,axis); // BODY perturbation, right action.
                    const auto sample=Evaluate(perturbed);ASSERT_TRUE(sample.finite);energy[sign]=sample.energy;
                }
                const double derivative=(energy[1]-energy[0])/(2*h);
                const double limit=1e-7+2e-6*std::max(std::abs(derivative),std::abs(restoring));
                errors[level++]=std::abs(derivative+restoring);
                EXPECT_LE(errors[level-1],limit);
                largest_error=std::max(largest_error,errors[level-1]);largest_limit=std::max(largest_limit,limit);
            }
            // O(h²) truncation shrinks by four; the independent roundoff floor
            // permits already-converged directions without a false ratio gate.
            EXPECT_LE(errors[1],.35*errors[0]+2e-7*(1+std::abs(restoring)));
        }
        Metric("all_24_dof_max_energy_gradient_error",largest_error,largest_limit);
        const auto restored=Evaluate(frames);Covariant(base,restored,chrono::QUNIT,"after_all_dof_probes");
        Balance(restored,frames,"all_dof_reference");ReferenceUnchanged();
    }
}

TEST_F(ReissnerReference, DistinctInitialDirectorOffsetsPreservePhysicalResponseAndBodyWork) {
    auto offset_reference=Neutral();
    offset_reference.q={{Rotation(1.4,Vec(1,0,0)),Rotation(-1.6,Vec(1,2,-1)),
                         Rotation(2.3,Vec(-2,.5,1)),Rotation(-2.2,Vec(.4,-1,2))}};
    OffsetReference offset(offset_reference);
    offset.Initialize();
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    StressFree(offset.Sample(offset_reference),"offset_initial");
    offset.CheckReference();

    const auto physical=Noncoaxial();
    const auto common_rotation=Rotation(1.7,Vec(1,2,-1));
    const std::array<Frames,2> physical_states{{physical,
        Transform(physical,common_rotation,Vec(2.1,-1.3,.7))}};
    Evaluation first_offset;
    for (std::size_t state=0;state<physical_states.size();++state) {
        SCOPED_TRACE(state);
        const auto& frames=physical_states[state];
        const auto baseline=Evaluate(frames);
        ASSERT_TRUE(baseline.finite);
        auto reparameterized=frames;
        for (int node=0;node<4;++node)
            reparameterized.q[node]=frames.q[node]*offset_reference.q[node];
        // Setup gives iTa_n=S_n^T. Thus (R_n*S_n)*iTa_n=R_n: positions,
        // physical directors, world forces/couples and elastic energy coincide.
        const auto value=offset.Sample(reparameterized);
        ASSERT_TRUE(value.finite);
        const auto prefix="offset_state_"+std::to_string(state);
        Covariant(baseline,value,chrono::QUNIT,prefix+"_same_physical_state");
        Balance(value,reparameterized,prefix+"_balance");
        if (state==0)
            first_offset=value;
        else
            Covariant(first_offset,value,common_rotation,prefix+"_superposition");

        double largest_error=0,largest_limit=0;
        for (int node=0;node<4;++node) for (int axis_index=0;axis_index<3;++axis_index) {
            SCOPED_TRACE(node);
            SCOPED_TRACE(axis_index);
            Vec axis(0,0,0);axis[axis_index]=1;
            const double restoring=value.couple[node].Dot(reparameterized.q[node].Rotate(axis));
            double errors[2];int level=0;
            for (double h : {2e-5,1e-5}) {
                double energy[2];
                for (int sign=0;sign<2;++sign) {
                    const double delta=sign ? h : -h;
                    auto perturbed=reparameterized;
                    // BODY increments act on the right of R_n*S_n. The
                    // corresponding world spin is (R_n*S_n)*axis, not R_n*axis.
                    perturbed.q[node]=reparameterized.q[node]*Rotation(delta,axis);
                    const auto sample=offset.Sample(perturbed);
                    ASSERT_TRUE(sample.finite);energy[sign]=sample.energy;
                }
                const double derivative=(energy[1]-energy[0])/(2*h);
                const double limit=1e-7+2e-6*std::max(std::abs(derivative),std::abs(restoring));
                errors[level++]=std::abs(derivative+restoring);
                EXPECT_LE(errors[level-1],limit);
                largest_error=std::max(largest_error,errors[level-1]);
                largest_limit=std::max(largest_limit,limit);
            }
            EXPECT_LE(errors[1],.35*errors[0]+2e-7*(1+std::abs(restoring)));
        }
        Metric(prefix+"_12_body_dof_energy_gradient_error",largest_error,largest_limit);
        const auto restored=offset.Sample(reparameterized);
        ASSERT_TRUE(restored.finite);
        Covariant(value,restored,chrono::QUNIT,prefix+"_after_probes");
        ReferenceUnchanged();offset.CheckReference();
    }
    StressFree(offset.Sample(offset_reference),"offset_restored_reference");
    ReferenceUnchanged();offset.CheckReference();
}

TEST_F(ReissnerReference, RejectedChartOrLateArithmeticPreservesForceOutputAndCleanRetry) {
    const auto frames=Noncoaxial();const auto before=Evaluate(frames);ASSERT_TRUE(before.finite);
    nodes[1]->SetRot(Rotation(2.2,Vec(1,0,0))*frames.q[1]);
    chrono::ChVectorDynamic<> force(24);force.setConstant(17.25);
    EXPECT_THROW(element->ComputeInternalForces(force),std::exception);
    for(int i=0;i<24;++i) EXPECT_DOUBLE_EQ(force[i],17.25);
    const auto after=Evaluate(frames);ASSERT_TRUE(after.finite);
    Covariant(before,after,chrono::QUNIT,"after_rejected_chart");ReferenceUnchanged();
    // All supplied coordinates remain finite, but existing strain/force
    // arithmetic overflows after valid director admission. No unsafe memory
    // access or CUDA context fault is injected.
    nodes[2]->SetPos(Vec(1e308,-1e308,1e308));force.setConstant(-11.5);
    EXPECT_THROW(element->ComputeInternalForces(force),std::exception);
    for(int i=0;i<24;++i) EXPECT_DOUBLE_EQ(force[i],-11.5);
    const auto retried=Evaluate(frames);ASSERT_TRUE(retried.finite);
    Covariant(before,retried,chrono::QUNIT,"after_late_force_overflow");ReferenceUnchanged();
}

TEST_F(ReissnerReference, UnqualifiedStiffnessAndDampingRequestsFailBeforeChangingCallerMatrix) {
    ASSERT_TRUE(Evaluate(Noncoaxial()).finite);
    chrono::ChMatrixDynamic<> matrix(24,24);matrix.setConstant(7.75);
    EXPECT_THROW(element->ComputeKRMmatricesGlobal(matrix,1,0,0),std::exception);
    EXPECT_TRUE((matrix.array()==7.75).all());
    EXPECT_THROW(element->ComputeKRMmatricesGlobal(matrix,0,1,0),std::exception);
    EXPECT_TRUE((matrix.array()==7.75).all());
}
}  // namespace crash::qualification
