#include "QephReference.h"

#include "lib_src/math/Quaternion.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>

namespace {
namespace native = tl::qualification::qeph;
using native::Kinematics;
using native::Matrix3;
using native::PrescribedInterval;
using native::Reference;
using native::ReferenceInput;
using native::Status;
using native::Vec3;
using Quaternion = tl::math::Quaternion;

// Frozen before the first native execution. These are arithmetic/covariance
// tolerances for the bounded <=2 m fixtures, not mechanics admission budgets.
constexpr double kAnalytic = 2e-12;
constexpr double kProperFrame = 5e-13;
constexpr double kCovariance = 2e-11;
constexpr std::array<double,3> kRigidSteps{{.04,.02,.01}};
constexpr double kZSpinHalving = .15;    // this analytic special case is cubic

Vec3 Add(Vec3 a, Vec3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vec3 Scale(Vec3 a, double scale) { return {scale*a.x,scale*a.y,scale*a.z}; }
double Dot(Vec3 a, Vec3 b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
Vec3 Cross(Vec3 a, Vec3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
double Norm(Vec3 a) { return std::hypot(a.x,a.y,a.z); }
Vec3 Unit(Vec3 a) { return Scale(a,1/Norm(a)); }
Vec3 Column(const Matrix3& a, unsigned j) { return {a.v[j],a.v[3+j],a.v[6+j]}; }
Vec3 Apply(const Matrix3& a, Vec3 x) {
    return Add(Add(Scale(Column(a,0),x.x),Scale(Column(a,1),x.y)),Scale(Column(a,2),x.z));
}
Quaternion Rotation(Vec3 vector) {
    const double increment[]{vector.x,vector.y,vector.z};
    Quaternion rotation;
    EXPECT_TRUE(tl::math::IncrementWorldRotation({},increment,rotation));
    return rotation;
}
Vec3 Rotate(Quaternion q, Vec3 x) {
    // Reuse owning TL quaternion product; no native frame/projection formula.
    const auto value=tl::math::Product(tl::math::Product(q,{0,x.x,x.y,x.z}),{q.w,-q.x,-q.y,-q.z});
    return {value.x,value.y,value.z};
}
Quaternion CommonRotation() {
    return tl::math::Product(Rotation({.4,-.7,.3}),Rotation({-.2,.25,1.1}));
}
void Near(Vec3 a, Vec3 b, double tolerance=kAnalytic) {
    EXPECT_NEAR(a.x,b.x,tolerance); EXPECT_NEAR(a.y,b.y,tolerance); EXPECT_NEAR(a.z,b.z,tolerance);
}
void Proper(const Matrix3& frame) {
    for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j)
        EXPECT_NEAR(Dot(Column(frame,i),Column(frame,j)),i==j?1.:0.,kProperFrame);
    EXPECT_NEAR(Dot(Cross(Column(frame,0),Column(frame,1)),Column(frame,2)),1.,kProperFrame);
}
template<std::size_t N>
void Near(const std::array<double,N>& a,const std::array<double,N>& b,double tolerance=kAnalytic) {
    for (unsigned i=0;i<N;++i) { SCOPED_TRACE(i); EXPECT_NEAR(a[i],b[i],tolerance); }
}
template<class T>
auto Bytes(const T& object) {
    static_assert(std::is_trivially_copyable_v<T>);
    std::array<unsigned char,sizeof(T)> bytes;
    std::memcpy(bytes.data(),&object,sizeof(T)); return bytes;
}
ReferenceInput Rectangle(double a=1,double b=.5,double warp=0) {
    ReferenceInput input;
    input.position={{{-a,-b,warp},{a,-b,-warp},{a,b,warp},{-a,b,-warp}}};
    input.node_ids={{17,3,1001,51}}; // Deliberately not dense or sorted.
    input.density=10; input.young_modulus=2e6; input.poisson_ratio=.25; input.thickness=.1;
    return input;
}
ReferenceInput Transform(ReferenceInput input,Quaternion rotation,Vec3 translation) {
    for (auto& position:input.position) position=Add(Rotate(rotation,position),translation);
    return input;
}
PrescribedInterval Static(const ReferenceInput& input,double h=.02) {
    PrescribedInterval interval; interval.position_endpoint=input.position; interval.dt=h;
    interval.base_time=.125; interval.sample_index=73; return interval;
}
PrescribedInterval Transform(PrescribedInterval input,Quaternion rotation,Vec3 translation) {
    for (unsigned n=0;n<4;++n) {
        input.position_endpoint[n]=Add(Rotate(rotation,input.position_endpoint[n]),translation);
        input.velocity_midpoint[n]=Rotate(rotation,input.velocity_midpoint[n]);
        input.omega_midpoint[n]=Rotate(rotation,input.omega_midpoint[n]);
    }
    return input;
}
PrescribedInterval RigidPath(const ReferenceInput& input,Vec3 omega,double h) {
    auto interval=Static(input,h);
    const auto endpoint=Rotation(Scale(omega,h)),midpoint=Rotation(Scale(omega,.5*h));
    for (unsigned n=0;n<4;++n) {
        interval.position_endpoint[n]=Rotate(endpoint,input.position[n]);
        interval.velocity_midpoint[n]=Cross(omega,Rotate(midpoint,input.position[n]));
        interval.omega_midpoint[n]=omega;
    }
    return interval;
}
void ZeroRates(const Kinematics& result,double tolerance=kAnalytic) {
    for (double rate:result.regular_rate) EXPECT_NEAR(rate,0.,tolerance);
    for (double rate:result.hourglass_rate) EXPECT_NEAR(rate,0.,tolerance);
}
void SameLocal(const Kinematics& a,const Kinematics& b,double tolerance=kCovariance) {
    EXPECT_EQ(a.planar,b.planar);
    EXPECT_NEAR(a.area,b.area,tolerance); EXPECT_NEAR(a.reciprocal_area,b.reciprocal_area,tolerance);
    EXPECT_NEAR(a.characteristic_length,b.characteristic_length,tolerance);
    EXPECT_NEAR(a.raw_warpage_abs,b.raw_warpage_abs,tolerance);
    EXPECT_NEAR(a.effective_warpage,b.effective_warpage,tolerance);
    Near(a.nodal_factors,b.nodal_factors,tolerance);
    Near(a.projection_inverse,b.projection_inverse,tolerance);
    Near(a.projected_omega,b.projected_omega,tolerance);
    Near(a.regular_rate,b.regular_rate,tolerance);
    Near(a.hourglass_rate,b.hourglass_rate,tolerance);
    for (unsigned n=0;n<4;++n) {
        Near(a.local_position[n],b.local_position[n],tolerance);
        Near(a.local_normals[n],b.local_normals[n],tolerance);
        Near(a.projection_columns[n],b.projection_columns[n],tolerance);
    }
}
double NormalizedRate(const Kinematics& result,double length,double angular_speed=1) {
    double maximum=0;
    for (unsigned i=0;i<8;++i)
        maximum=std::max(maximum,std::abs(result.regular_rate[i])*(i<5?1:length)/angular_speed);
    for (unsigned i=0;i<6;++i)
        maximum=std::max(maximum,std::abs(result.hourglass_rate[i])/((i==2||i==3)?angular_speed:length*angular_speed));
    return maximum;
}
std::string ExactText(double value) {
    std::ostringstream text; text<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
    return text.str();
}
std::array<double,8> PlanarRigidRateTruth(Vec3 unit_omega,double h) {
    // Independent exact rigid-path velocity gradient in the endpoint frame:
    // H=Omega*exp(-h*Omega/2)=cos(d)*Omega+sin(d)*(I-w*w^T), |w|=1.
    // Express the native time correction as operations on that gradient,
    // independent of native nodal/diagonal packing and element dimensions.
    const double w[3]={unit_omega.x,unit_omega.y,unit_omega.z},d=.5*h;
    const double spin[3][3]={{0,-w[2],w[1]},{w[2],0,-w[0]},{-w[1],w[0],0}};
    double gradient[3][3]{},corrected[2][3]{};
    for (unsigned i=0;i<3;++i) for (unsigned j=0;j<3;++j)
        gradient[i][j]=std::cos(d)*spin[i][j]+std::sin(d)*((i==j?1.:0.)-w[i]*w[j]);
    for (unsigned j=0;j<3;++j) {
        corrected[0][j]=gradient[0][j]-d*gradient[2][0]*gradient[2][j]
                                         -d*gradient[1][0]*gradient[1][j];
        corrected[1][j]=gradient[1][j]-d*gradient[2][1]*gradient[2][j]
                                         -d*gradient[0][1]*gradient[0][j];
    }
    // The native correction leaves transverse velocity unchanged. The flat
    // rotation projection is simply the two tangential components of omega.
    return {{corrected[0][0],corrected[1][1],corrected[0][1]+corrected[1][0],
             gradient[2][0]+w[1],gradient[2][1]-w[0],0,0,0}};
}

TEST(QephKinematics, RectangleStartupUsesActualAreaDerivativesAndSeparateNativeInertia) {
    for (const auto half_sides: {std::array<double,2>{1,.5},std::array<double,2>{.2,.1}}) {
        SCOPED_TRACE(half_sides[0]);
        const auto input=Rectangle(half_sides[0],half_sides[1]); Reference reference;
        ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess); ASSERT_TRUE(reference.prepared());
        const auto& data=reference.data(); Proper(data.frame);
        const double a=half_sides[0],b=half_sides[1],area=4*a*b;
        EXPECT_NEAR(data.area,area,kAnalytic*area);
        for (unsigned axis=0;axis<3;++axis)
            Near(Column(data.frame,axis),{axis==0?1.:0.,axis==1?1.:0.,axis==2?1.:0.});
        Near(data.derivative_x,std::array<double,4>{{-b,b,b,-b}});
        Near(data.derivative_y,std::array<double,4>{{-a,-a,a,a}});
        const std::array<Vec3,4> relative{{{0,0,0},{2*a,0,0},{2*a,2*b,0},{0,2*b,0}}};
        for (unsigned n=0;n<4;++n) {
            EXPECT_EQ(data.input.node_ids[n],input.node_ids[n]); Near(data.local_position[n],relative[n]);
            const double mass=input.density*input.thickness*area/4;
            const double physical=mass*input.thickness*input.thickness/12;
            const double added=mass*area/12;
            EXPECT_NEAR(data.nodal_mass[n],mass,kAnalytic*mass);
            EXPECT_NEAR(data.physical_inertia[n],physical,kAnalytic*physical);
            EXPECT_NEAR(data.added_inertia[n],added,kAnalytic*added);
            EXPECT_NEAR(data.isotropic_inertia[n],physical+added,kAnalytic*(physical+added));
            EXPECT_GT(data.added_inertia[n],data.physical_inertia[n]);
        }
    }
}

TEST(QephKinematics, PlanarAndSaddleStaticProjectionMatchIndependentBilinearNormals) {
    for (const double warp:{0.,.05}) {
        SCOPED_TRACE(warp);
        const auto input=Rectangle(1,.5,warp); Reference reference;
        ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
        Kinematics result;
        ASSERT_EQ(native::EvaluatePrescribed(reference,Static(input),result),Status::kSuccess);
        Proper(result.frame); EXPECT_NEAR(result.area,2.,kAnalytic);
        EXPECT_NEAR(result.reciprocal_area,.5,kAnalytic); EXPECT_GT(result.characteristic_length,0.);
        EXPECT_EQ(result.planar,warp==0); EXPECT_NEAR(result.raw_warpage_abs,warp,kAnalytic);
        EXPECT_NEAR(result.effective_warpage,warp,kAnalytic);
        Near(result.nodal_factors,std::array<double,2>{{1,1}});
        constexpr double xi[]{-1,1,1,-1},eta[]{-1,-1,1,1};
        for (unsigned n=0;n<4;++n) {
            // x(xi,eta)=(a*xi,b*eta,warp*xi*eta); cross of its two
            // analytic tangents is (-b*warp*eta,-a*warp*xi,a*b).
            const Vec3 normal=Unit({-.5*warp*eta[n],-warp*xi[n],.5});
            Near(Apply(result.frame,result.local_normals[n]),normal);
            Near(result.local_position[n],input.position[n]);
        }
        ZeroRates(result);
        if (warp==0) {
            for (double value:result.projection_inverse) EXPECT_DOUBLE_EQ(value,0.);
            for (auto column:result.projection_columns) Near(column,{},0.);
        } else {
            EXPECT_GT(*std::max_element(result.projection_inverse.begin(),result.projection_inverse.end()),0.);
        }
    }
}

TEST(QephKinematics, StaticAndNonzeroPrescribedKinematicsHaveProperCommonRotationCovariance) {
    const auto rotation=CommonRotation(); const Vec3 translation{1.25,-.75,.5};
    for (const double warp:{0.,.05}) {
        const auto original=Rectangle(1,.5,warp),transformed=Transform(original,rotation,translation);
        Reference base,changed;
        ASSERT_EQ(native::Initialize(original,base),Status::kSuccess);
        ASSERT_EQ(native::Initialize(transformed,changed),Status::kSuccess);
        for (unsigned axis=0;axis<3;++axis)
            Near(Column(changed.data().frame,axis),Rotate(rotation,Column(base.data().frame,axis)),kCovariance);
        Near(base.data().nodal_mass,changed.data().nodal_mass,kCovariance);
        Near(base.data().isotropic_inertia,changed.data().isotropic_inertia,kCovariance);
        for (bool stationary:{true,false}) {
            SCOPED_TRACE(warp);
            SCOPED_TRACE(stationary);
            auto interval=Static(original);
            if (!stationary) for (unsigned n=0;n<4;++n) {
                const auto p=original.position[n];
                interval.velocity_midpoint[n]={.01*p.x+.02*p.y,-.015*p.y,.005*p.x};
                interval.omega_midpoint[n]={.007*p.y,-.006*p.x,.004};
            }
            Kinematics a,b;
            ASSERT_EQ(native::EvaluatePrescribed(base,interval,a),Status::kSuccess);
            ASSERT_EQ(native::EvaluatePrescribed(changed,Transform(interval,rotation,translation),b),Status::kSuccess);
            Proper(a.frame); Proper(b.frame); SameLocal(a,b);
            for (unsigned axis=0;axis<3;++axis)
                Near(Column(b.frame,axis),Rotate(rotation,Column(a.frame,axis)),kCovariance);
            if (stationary) { ZeroRates(a); ZeroRates(b); }
            else EXPECT_GT(NormalizedRate(a,2),1e-4);
        }
    }
}

TEST(QephKinematics, NativePlanarSwitchIsExposedInsteadOfCallingSmallWarpExactlyPlanar) {
    // IRESP=2 selects TOL=1e-8. On this rectangle LM=a^2+b^2=1.25;
    // the complete native CZCORP5 switch tests Z1^2 < LM*TOL.
    // Both fixtures are far from that boundary, so no tolerance is fitted.
    for (const double warp:{1e-6,1e-3}) {
        SCOPED_TRACE(warp); const auto input=Rectangle(1,.5,warp); Reference reference;
        ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
        Kinematics result;
        ASSERT_EQ(native::EvaluatePrescribed(reference,Static(input),result),Status::kSuccess);
        EXPECT_NEAR(result.raw_warpage_abs,warp,kAnalytic);
        EXPECT_GT(result.raw_warpage_abs,0.);
        const bool native_flat=warp*warp < 1.25e-8;
        EXPECT_EQ(result.planar,native_flat);
        EXPECT_NEAR(result.effective_warpage,native_flat?0.:warp,kAnalytic);
        ZeroRates(result);
    }
}

TEST(QephKinematics, CyclicNodePermutationPreservesMassAndPhysicalWarpedNormals) {
    const auto input=Rectangle(1,.5,.05); Reference baseline;
    ASSERT_EQ(native::Initialize(input,baseline),Status::kSuccess);
    Kinematics original;
    ASSERT_EQ(native::EvaluatePrescribed(baseline,Static(input),original),Status::kSuccess);
    for (unsigned shift=1;shift<4;++shift) {
        SCOPED_TRACE(shift); auto permuted=input;
        for (unsigned n=0;n<4;++n) {
            permuted.position[n]=input.position[(n+shift)%4];
            permuted.node_ids[n]=input.node_ids[(n+shift)%4];
        }
        Reference reference; ASSERT_EQ(native::Initialize(permuted,reference),Status::kSuccess);
        Kinematics result; ASSERT_EQ(native::EvaluatePrescribed(reference,Static(permuted),result),Status::kSuccess);
        Proper(result.frame); EXPECT_NEAR(result.area,original.area,kAnalytic); EXPECT_FALSE(result.planar);
        for (unsigned n=0;n<4;++n) {
            EXPECT_NEAR(reference.data().nodal_mass[n],baseline.data().nodal_mass[(n+shift)%4],kAnalytic);
            Near(Apply(result.frame,result.local_normals[n]),
                 Apply(original.frame,original.local_normals[(n+shift)%4]),kAnalytic);
        }
        ZeroRates(result);
    }
}

TEST(QephKinematics, SmallAffineMembraneShearAndBendingRatesUseDeclaredComponentOrder) {
    const auto input=Rectangle(); Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    constexpr double amplitude=1e-4;
    for (unsigned mode=0;mode<6;++mode) {
        SCOPED_TRACE(mode); auto interval=Static(input,1e-6);
        for (unsigned n=0;n<4;++n) {
            const auto p=input.position[n];
            if (mode==0) interval.velocity_midpoint[n].x=amplitude*p.x;
            if (mode==1) interval.velocity_midpoint[n].y=amplitude*p.y;
            if (mode==2) interval.velocity_midpoint[n].x=amplitude*p.y;
            if (mode==3) interval.omega_midpoint[n].y=amplitude*p.x;
            if (mode==4) interval.omega_midpoint[n].x=-amplitude*p.y;
            if (mode==5) interval.omega_midpoint[n]={-.5*amplitude*p.x,.5*amplitude*p.y,0};
        }
        Kinematics result; ASSERT_EQ(native::EvaluatePrescribed(reference,interval,result),Status::kSuccess);
        std::array<double,8> expected{}; expected[mode<3?mode:mode+2]=amplitude;
        // The selected shear's temporal quadratic term is at most 5e-15/s,
        // below the frozen arithmetic tolerance; no finite-strain law claimed.
        Near(result.regular_rate,expected,kAnalytic);
    }
}

TEST(QephKinematics, EndpointGeometryAndMidpointZSpinMatchAnalyticCubicResidual) {
    const auto input=Rectangle(); Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    double previous=std::numeric_limits<double>::infinity();
    for (unsigned i=0;i<kRigidSteps.size();++i) {
        const double h=kRigidSteps[i],a=.5*h;
        SCOPED_TRACE(h); Kinematics result;
        ASSERT_EQ(native::EvaluatePrescribed(reference,RigidPath(input,{0,0,1},h),result),Status::kSuccess);
        // For this centered planar rectangle and omega_z=1, endpoint-frame
        // velocity gradients are sin(a)I+cos(a)[ez]x. Applying the source's
        // declared time correction gives this closed scalar remainder.
        // Its leading term is 5*h^3/48; finite-step zero is NOT expected.
        const double residual=std::sin(a)-a*std::cos(a)*std::cos(a);
        std::array<double,8> expected{}; expected[0]=expected[1]=residual;
        Near(result.regular_rate,expected,kAnalytic);
        for (double value:result.hourglass_rate) EXPECT_NEAR(value,0.,kAnalytic);
        const double measured=NormalizedRate(result,2);
        EXPECT_GT(measured,1e-9);
        EXPECT_LE(measured,.12*h*h*h+kAnalytic);
        if (i) EXPECT_LE(measured,kZSpinHalving*previous+kAnalytic);
        RecordProperty("z_spin_residual_h"+std::to_string(i),ExactText(measured));
        previous=measured;
    }
}

TEST(QephKinematics, GeneralRigidPhaseMatchesPlanarTruthAndRecordsWarpedDiagnostics) {
    // The first execution's all-component O(h^2) rate assumption failed and
    // is preserved verbatim in crash-work/checkpoints/qeph-q1-first-execution-1.
    // This replacement characterizes faithful native arithmetic; it does not
    // relax or qualify a dynamics order requirement. In particular, the exact
    // planar gamma_yz=-a*(1-cos(h/2))-b*c*sin(h/2) is O(h) for this axis.
    // Independent full warped force/work/time refinement remains a later gate.
    const Vec3 omega=Unit({1,2,3});
    const auto common=CommonRotation(); const Vec3 translation{1.25,-.75,.5};
    RecordProperty("source_dynamics_qualified","false");
    RecordProperty("warped_temporal_order_qualified","false");
    for (const double warp:{0.,.05}) {
        SCOPED_TRACE(warp); const auto input=Rectangle(1,.5,warp); Reference reference;
        ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
        Reference transformed_reference;
        ASSERT_EQ(native::Initialize(Transform(input,common,translation),transformed_reference),Status::kSuccess);
        for (unsigned i=0;i<kRigidSteps.size();++i) {
            const double h=kRigidSteps[i]; const auto interval=RigidPath(input,omega,h); Kinematics result;
            ASSERT_EQ(native::EvaluatePrescribed(reference,interval,result),Status::kSuccess);
            Proper(result.frame); EXPECT_EQ(result.planar,warp==0);
            if (warp==0) {
                Near(result.regular_rate,PlanarRigidRateTruth(omega,h),kAnalytic);
                for (double rate:result.hourglass_rate) EXPECT_NEAR(rate,0.,kAnalytic);
            }
            Kinematics transformed;
            ASSERT_EQ(native::EvaluatePrescribed(transformed_reference,Transform(interval,common,translation),
                                                 transformed),Status::kSuccess);
            SameLocal(result,transformed,kCovariance);
            for (unsigned axis=0;axis<3;++axis)
                Near(Column(transformed.frame,axis),Rotate(common,Column(result.frame,axis)),kCovariance);
            const auto prefix=std::string(warp==0?"planar":"warped")+"_h"+std::to_string(i);
            RecordProperty(prefix+"_normalized_max",ExactText(NormalizedRate(result,2)));
            for (unsigned component=0;component<8;++component) {
                EXPECT_TRUE(std::isfinite(result.regular_rate[component]));
                RecordProperty(prefix+"_regular_"+std::to_string(component),ExactText(result.regular_rate[component]));
            }
            for (unsigned component=0;component<6;++component) {
                EXPECT_TRUE(std::isfinite(result.hourglass_rate[component]));
                RecordProperty(prefix+"_hourglass_"+std::to_string(component),ExactText(result.hourglass_rate[component]));
            }
        }
    }
}

TEST(QephKinematics, InvalidStartupPreservesPreparedReferenceAndPermitsRetry) {
    const auto input=Rectangle(); Reference output;
    ASSERT_EQ(native::Initialize(input,output),Status::kSuccess);
    const auto accepted=Bytes(output);
    for (unsigned fault=0;fault<8;++fault) {
        SCOPED_TRACE(fault); auto invalid=input;
        if (fault==0) invalid.node_ids[3]=invalid.node_ids[1];
        if (fault==1) invalid.position[3].z=std::numeric_limits<double>::quiet_NaN();
        if (fault==2) invalid.density=-1;
        if (fault==3) invalid.thickness=0;
        if (fault==4) invalid.poisson_ratio=.5;
        if (fault==5) invalid.young_modulus=std::numeric_limits<double>::infinity();
        if (fault==6) for (auto& p:invalid.position) p.y=0;
        if (fault==7) invalid.poisson_ratio=-.1;
        EXPECT_NE(native::Initialize(invalid,output),Status::kSuccess);
        EXPECT_EQ(Bytes(output),accepted);
    }
    ASSERT_EQ(native::Initialize(input,output),Status::kSuccess);
    EXPECT_NEAR(output.data().area,2.,kAnalytic);
}

TEST(QephKinematics, InvalidIntervalsPreserveAllOutputAndReferenceWithCleanRetry) {
    const auto input=Rectangle(1,.5,.05); Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    const auto accepted=Bytes(reference); const auto valid=Static(input);
    Kinematics result;
    ASSERT_EQ(native::EvaluatePrescribed(reference,valid,result),Status::kSuccess);
    const auto old=Bytes(result); const auto clean=result;
    Reference unprepared;
    EXPECT_EQ(native::EvaluatePrescribed(unprepared,valid,result),Status::kInvalidReference);
    EXPECT_EQ(Bytes(result),old);
    for (unsigned fault=0;fault<7;++fault) {
        SCOPED_TRACE(fault); auto interval=valid; const auto before=Bytes(result);
        if (fault==0) interval.dt=0;
        if (fault==1) interval.dt=-.01;
        if (fault==2) interval.base_time=std::numeric_limits<double>::infinity();
        if (fault==3) interval.position_endpoint[3].z=std::numeric_limits<double>::quiet_NaN();
        if (fault==4) interval.velocity_midpoint[3].x=std::numeric_limits<double>::infinity();
        if (fault==5) for (auto& p:interval.position_endpoint) p.y=0;
        if (fault==6) for (unsigned n=0;n<4;++n)
            interval.omega_midpoint[n]={1e308,n%2?-1e308:1e308,1e308};
        EXPECT_NE(native::EvaluatePrescribed(reference,interval,result),Status::kSuccess);
        EXPECT_EQ(Bytes(result),before); EXPECT_EQ(Bytes(reference),accepted);
        ASSERT_EQ(native::EvaluatePrescribed(reference,valid,result),Status::kSuccess);
        SameLocal(result,clean,0.);
        for (unsigned i=0;i<9;++i) EXPECT_DOUBLE_EQ(result.frame.v[i],clean.frame.v[i]);
    }
}

TEST(QephKinematics, RepeatedCallsKeepCallerSequenceAndNoHiddenGeometryHistory) {
    const auto input=Rectangle(1,.5,.05); Reference reference;
    ASSERT_EQ(native::Initialize(input,reference),Status::kSuccess);
    const auto accepted=Bytes(reference);
    auto interval=RigidPath(input,Unit({1,-2,3}),.01); interval.sample_index=101; interval.base_time=.5;
    Kinematics first,other,repeated;
    ASSERT_EQ(native::EvaluatePrescribed(reference,interval,first),Status::kSuccess);
    ASSERT_EQ(native::EvaluatePrescribed(reference,Static(input),other),Status::kSuccess);
    ASSERT_EQ(native::EvaluatePrescribed(reference,interval,repeated),Status::kSuccess);
    SameLocal(first,repeated,0.);
    for (unsigned i=0;i<9;++i) EXPECT_DOUBLE_EQ(first.frame.v[i],repeated.frame.v[i]);
    EXPECT_EQ(repeated.sample_index,101u); EXPECT_EQ(repeated.base_time,.5); EXPECT_EQ(repeated.dt,.01);
    EXPECT_EQ(Bytes(reference),accepted);
}
} // namespace
