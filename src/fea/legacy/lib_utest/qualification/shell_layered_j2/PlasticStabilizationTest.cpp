#include "LayeredJ2Fixture.h"
#include "lib_src/elements/qeph/QephPlasticStabilization.h"
#include <gtest/gtest.h>
#include <array>
#include <algorithm>
#include <cmath>

extern "C" void qeph_plastic_stabilization(const double*,const double*,const double*,
    const double*,const double*,const double*,const double*,const double*,double*,int*);

namespace layered_j2_test {
namespace detail=q::detail;
std::array<double,38> Pack(const q::HistoryValues& h) {
    std::array<double,38> out{};
    std::copy_n(h.stress,5,out.begin());
    std::copy_n(h.material_stress,5,out.begin()+5);
    std::copy_n(h.bending_stress,3,out.begin()+10);
    std::copy_n(h.stabilization,12,out.begin()+13);
    std::copy_n(h.strain_curvature,8,out.begin()+25);
    out[33]=h.thickness; out[34]=h.internal_work[0]; out[35]=h.internal_work[1];
    out[36]=h.hourglass_viscous_work; out[37]=h.active;
    return out;
}
void CheckNative(double mean,double minimum,double yield,bool unloading,bool global_yield) {
    auto f=Families(Parameters());
    auto& in=f.qi; in.dt=1e-7; in.position_endpoint[2].z=2e-4;
    for(unsigned n=0;n<4;++n) {
        const double alternating=(n%2?-1.:1.);
        in.velocity_midpoint[n].x+=25*alternating;
        in.velocity_midpoint[n].y-=12*alternating;
        in.omega_midpoint[n]={70*alternating,-35*alternating,12*alternating};
    }
    detail::GeometryWork geometry;
    ASSERT_EQ(detail::PrepareGeometry(f.qr,in,geometry),q::Status::kSuccess);
    detail::MaterialWork material;
    ASSERT_TRUE(detail::PrepareMaterial(f.qr.input,geometry.values.area,in.dt,material));
    q::HistoryValues base;
    base.thickness=material.thickness;
    detail::StabilizationWork probe;
    detail::UpdateStabilization(geometry,material,base,probe);
    base={}; base.thickness=material.thickness;
    for(unsigned i=0;i<12;++i) base.stabilization[i]=(unloading?-10.:10.)*probe.delta[i];
    base.stress[0]=global_yield?400e6:100e6;
    base.stress[1]=-40e6; base.stress[2]=20e6;
    base.bending_stress[0]=15e6; base.bending_stress[1]=-5e6;
    std::copy_n(base.stress,5,base.material_stress);
    base.internal_work[0]=.07; base.internal_work[1]=.01;
    double x[12]{},v[12]{},omega[12]{};
    for(unsigned n=0;n<4;++n) {
        const auto xx=in.position_endpoint[n],vv=in.velocity_midpoint[n],ww=in.omega_midpoint[n];
        x[3*n]=xx.x; x[3*n+1]=xx.y; x[3*n+2]=xx.z;
        v[3*n]=vv.x; v[3*n+1]=vv.y; v[3*n+2]=vv.z;
        omega[3*n]=ww.x; omega[3*n+1]=ww.y; omega[3*n+2]=ww.z;
    }
    // Existing native QE_PREPARE_MATERIAL packet order: rho, E, nu, thickness.
    const double parameters[]{material.rho,material.young,material.nu,material.thickness};
    const double factors[]{mean,minimum}; const auto packed=Pack(base);
    std::array<double,62> native{}; int status=-1;
    qeph_plastic_stabilization(x,v,omega,parameters,packed.data(),&in.dt,factors,&yield,native.data(),&status);
    ASSERT_EQ(status,0);
    auto actual=base;
    detail::LocalForceWork local;
    detail::ElasticForces(geometry,material,actual,local);
    detail::StabilizationWork work;
    detail::UpdateStabilization(geometry,material,actual,work);
    const auto elastic=actual;
    ASSERT_TRUE(detail::CorrectPlasticStabilization(material,mean,minimum,yield,actual,work));
    if(unloading) {
        ASSERT_LT(work.membrane_loading,0);
        ASSERT_LT(work.bending_loading,0);
        for(unsigned i=0;i<12;++i) EXPECT_EQ(actual.stabilization[i],elastic.stabilization[i]);
    } else if(yield<9.*(1e20*1e9)&&(mean<1||global_yield)) {
        bool changed=false;
        for(unsigned i=0;i<12;++i) changed|=actual.stabilization[i]!=elastic.stabilization[i];
        EXPECT_TRUE(changed);
    }
    detail::StabilizationForces(geometry,material,actual,work,local);
    tl::math::Vec3 force[4]{},couple[4]{};
    detail::ProjectForces(geometry,local,force,couple);
    const auto actual_history=Pack(actual);
    const auto compare=[](double a,double b) {
        EXPECT_NEAR(a,b,2e-10+3e-12*std::max(std::abs(a),std::abs(b)));
    };
    for(unsigned i=0;i<38;++i) {SCOPED_TRACE(i);compare(actual_history[i],native[i]);}
    for(unsigned n=0;n<4;++n) {
        const double ff[]{force[n].x,force[n].y,force[n].z};
        const double cc[]{couple[n].x,couple[n].y,couple[n].z};
        for(unsigned a=0;a<3;++a) {compare(ff[a],native[38+3*n+a]);compare(cc[a],native[50+3*n+a]);}
    }
}
TEST(QephPlasticStabilization, NativePlasticLoadingAndElasticUnloading) {
    CheckNative(.3,.01,270e6,false,false);
    CheckNative(.3,.01,270e6,true,false);
}
TEST(QephPlasticStabilization, NativeGlobalCriterionAndElasticLimit) {
    CheckNative(1,1,270e6,false,true);
    CheckNative(1,1,1e12,false,false);
    CheckNative(0,0,1e30,false,false);
}
}
