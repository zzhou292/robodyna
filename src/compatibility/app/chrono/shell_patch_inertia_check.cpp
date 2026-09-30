#include "ShellPatchInertia.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>

namespace crash::qualification {
namespace {
using namespace reference;
namespace tlr=tl::fea::reissner;
using Status=ElasticCouponStatus;
using Nodes=std::array<tlr::Vec3,kCouponNodes>;
using Masses=std::array<tlr::ShellMass,kCouponElements>;
using Connectivity=std::array<std::array<std::size_t,4>,kCouponElements>;
constexpr Connectivity connectivity{{{0,1,2,3},{4,0,3,5}}};

// Only the mass-bearing tables are needed by the owning TL operation. These
// exact constant shape rows prescribe independent row-sum fractions; they do
// not claim to be a complete elastic Q4 setup or geometry admission.
tlr::ShellMass Mass(double area,double thickness=.5,double density=12,
                    std::array<double,4> fraction={{.25,.25,.25,.25}}) {
    tlr::ShellReference reference;
    reference.prepared=true;
    for (auto& point:reference.gauss) {
        point.area_weight=area/4;
        for (std::size_t i=0;i<4;++i) point.shape[i]=fraction[i];
    }
    tlr::ElasticSection section;
    section.prepared=true; section.thickness=thickness; section.density=density;
    tlr::ShellMass result;
    EXPECT_EQ(tlr::ComputeShellMass(reference,section,
              tlr::ShellDrillingInertiaPolicy::kEqualPhysicalTangential,result),tlr::ShellMassStatus::kSuccess);
    return result;
}
ShellPatchInertia Assemble(const Masses& mass) {
    ShellPatchInertia result;
    std::string diagnostic;
    EXPECT_EQ(AssembleShellPatchInertia(mass,connectivity,result,diagnostic),Status::kSuccess)<<diagnostic;
    EXPECT_TRUE(diagnostic.empty());
    return result;
}
template<class T> std::array<unsigned char,sizeof(T)> Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> result;
    std::memcpy(result.data(),&value,sizeof(T));
    return result;
}
void Near(double value,double expected) {
    EXPECT_NEAR(value,expected,64*std::numeric_limits<double>::epsilon()*std::abs(expected));
}
void SameNode(const ShellPatchInertia& a,std::size_t i,const ShellPatchInertia& b,std::size_t j) {
    EXPECT_EQ(a.original[i].area,b.original[j].area);
    EXPECT_EQ(a.original[i].mass,b.original[j].mass);
    EXPECT_EQ(a.original[i].physical_tangential_inertia,b.original[j].physical_tangential_inertia);
    EXPECT_EQ(a.original[i].artificial_drilling_inertia,b.original[j].artificial_drilling_inertia);
    EXPECT_EQ(a.added_tangential_inertia[i],b.added_tangential_inertia[j]);
    EXPECT_EQ(a.added_drilling_inertia[i],b.added_drilling_inertia[j]);
    EXPECT_EQ(a.counterfactual.mass[i],b.counterfactual.mass[j]);
    EXPECT_EQ(a.counterfactual.total_isotropic_inertia[i],b.counterfactual.total_isotropic_inertia[j]);
}

TEST(ShellPatchInertia, ActualTLSetupUniformPhysicalAndWholeAreaAnalyticLedger) {
    ElasticCouponParameters parameters;
    parameters.length=.02; parameters.width=.01; parameters.thickness=.001648;
    parameters.young_modulus=2e11; parameters.density=7890;
    ElasticCouponModel model(parameters);
    const auto& data=model.data();
    ShellPatchInertia result;
    std::string diagnostic;
    ASSERT_EQ(AssembleShellPatchInertia(data.element_mass,data.connectivity,result,diagnostic),Status::kSuccess)<<diagnostic;
    const double whole_area=parameters.length*parameters.width/2;
    double sum_mass=0,sum_physical=0,sum_added=0;
    for (const double area:result.element_area) Near(area,whole_area);
    for (std::size_t n=0;n<kCouponNodes;++n) {
        const double pieces=(n==0||n==3)?2:1;
        const double mass=pieces*parameters.density*parameters.thickness*whole_area/4;
        const double physical=mass*parameters.thickness*parameters.thickness/12;
        const double added=mass*whole_area/12;
        Near(result.original[n].mass,mass);
        Near(result.original[n].physical_tangential_inertia,physical);
        EXPECT_EQ(result.original[n].physical_tangential_inertia,result.original[n].artificial_drilling_inertia);
        EXPECT_EQ(result.original[n].mass,data.nodal_mass[n].mass);
        EXPECT_EQ(result.original[n].physical_tangential_inertia,data.nodal_mass[n].physical_tangential_inertia);
        Near(result.added_tangential_inertia[n],added);
        EXPECT_EQ(result.added_drilling_inertia[n],result.added_tangential_inertia[n]);
        Near(result.counterfactual.total_isotropic_inertia[n],physical+added);
        EXPECT_NE(result.counterfactual.total_isotropic_inertia[n],2*physical+added);
        sum_mass+=result.original[n].mass; sum_physical+=result.original[n].physical_tangential_inertia;
        sum_added+=result.added_tangential_inertia[n];
    }
    Near(sum_mass,parameters.density*parameters.thickness*parameters.length*parameters.width);
    Near(sum_physical,sum_mass*parameters.thickness*parameters.thickness/12);
    Near(sum_added,sum_mass*whole_area/12);
    // Constrained nodes remain positive contributors; this diagnostic does not
    // overwrite the existing owner policy or its constrained inverse masses.
    EXPECT_GT(result.counterfactual.mass[1],0);
    EXPECT_EQ(data.inverse_mass[1],0);
}

TEST(ShellPatchInertia, UnequalElementAreasAreReducedBeforeSharedNodeAssembly) {
    const auto result=Assemble({Mass(1),Mass(4)});
    EXPECT_EQ(result.element_area,(std::array<double,2>{{1,4}}));
    // Each element has rho*t=6. Local masses are 1.5 and 6, respectively.
    // Shared DeltaJ = 1.5*1/12 + 6*4/12 = 2.125; neither a nodal-area
    // substitution nor multiplying assembled mass by total patch area agrees.
    for (std::size_t n=0;n<kCouponNodes;++n) {
        const bool shared=n==0||n==3,first=n==1||n==2;
        const double mass=shared?7.5:(first?1.5:6);
        const double added=shared?2.125:(first?.125:2);
        EXPECT_EQ(result.original[n].mass,mass);
        EXPECT_EQ(result.original[n].physical_tangential_inertia,mass/48);
        EXPECT_EQ(result.added_tangential_inertia[n],added);
        EXPECT_EQ(result.counterfactual.total_isotropic_inertia[n],mass/48+added);
    }
}

TEST(ShellPatchInertia, ElementLocalAndGlobalPermutationsPreserveUnequalRows) {
    const Masses mass{{Mass(1,.5,12,{{.125,.25,.375,.25}}),Mass(4)}};
    const auto baseline=Assemble(mass);
    const std::array<std::size_t,6> node_map{{5,3,1,4,0,2}};
    const std::array<std::size_t,4> local_map{{2,0,3,1}};
    Masses transformed;
    Connectivity map;
    for (std::size_t e=0;e<2;++e) {
        transformed[e].drilling_policy=mass[1-e].drilling_policy;
        for (std::size_t i=0;i<4;++i) {
            transformed[e].node[i]=mass[1-e].node[local_map[i]];
            map[e][i]=node_map[connectivity[1-e][local_map[i]]];
        }
    }
    ShellPatchInertia result;
    std::string diagnostic;
    ASSERT_EQ(AssembleShellPatchInertia(transformed,map,result,diagnostic),Status::kSuccess)<<diagnostic;
    for (std::size_t n=0;n<6;++n) SameNode(result,node_map[n],baseline,n);
    EXPECT_EQ(result.element_area[0],baseline.element_area[1]);
    EXPECT_EQ(result.element_area[1],baseline.element_area[0]);
}

TEST(ShellPatchInertia, InvalidContributionsAndLateOverflowPreserveCompleteOutputAndRetry) {
    const Masses good{{Mass(1),Mass(4)}};
    const auto saved=Assemble(good);
    for (unsigned variant=0;variant<12;++variant) {
        auto mass=good; auto map=connectivity; auto result=saved;
        const auto saved_bytes=Bytes(result);
        switch (variant) {
            case 0: map[1][3]=6; break;
            case 1: map[1][3]=map[1][2]; break;
            case 2: map[1]=map[0]; break;  // unused nodes 4 and 5
            case 3: mass[1].drilling_policy=tlr::ShellDrillingInertiaPolicy::kNone; break;
            case 4: mass[1].node[3].artificial_drilling_inertia*=2; break;
            case 5: mass[1].node[3].area=0; break;
            case 6: mass[1].node[3].mass=std::numeric_limits<double>::quiet_NaN(); break;
            case 7: mass[1].node[3].physical_tangential_inertia=-1; break;
            case 8: for (auto& n:mass[1].node) n.area=1e308; break;
            case 9: for (auto& n:mass[1].node) { n.area=1e307; n.mass=1e307; } break;
            case 10:
                for (auto& e:mass) for (auto& n:e.node) {
                    n.area=1e-307; n.mass=1e308;
                }
                break;  // local DeltaJ finite; second contribution overflows shared mass
            case 11:
                for (auto& n:mass[1].node) { n.area=1e-300; n.mass=1e-300; }
                break;  // unrepresentable positive DeltaJ
        }
        std::string diagnostic;
        EXPECT_NE(AssembleShellPatchInertia(mass,map,result,diagnostic),Status::kSuccess)<<variant;
        EXPECT_FALSE(diagnostic.empty()); EXPECT_EQ(Bytes(result),saved_bytes);
        ASSERT_EQ(AssembleShellPatchInertia(good,connectivity,result,diagnostic),Status::kSuccess)<<diagnostic;
        for (std::size_t n=0;n<6;++n) SameNode(result,n,saved,n);
        EXPECT_TRUE(diagnostic.empty());
    }
}

TEST(ShellPatchInertia, FiveKineticPartitionsMatchAnalyticAndOwningTLElementLedgers) {
    const Masses mass{{Mass(1),Mass(4)}};
    const auto inertia=Assemble(mass);
    Nodes director,velocity,omega;
    director.fill({0,0,1}); velocity.fill({1,2,3}); omega.fill({2,3,4});
    ShellPatchKineticEnergy result;
    std::string diagnostic;
    ASSERT_EQ(ComputeShellPatchKineticEnergy(inertia,director,velocity,omega,result,diagnostic),Status::kSuccess);
    // Whole masses 6 and 24, physical J_total = 30*.5^2/12 = .625;
    // area-added J_total = (6*1 + 24*4)/12 = 8.5.
    EXPECT_EQ(result.translation,.5*30*14);
    EXPECT_EQ(result.physical_rotation,.5*.625*13);
    EXPECT_EQ(result.original_artificial_drilling,.5*.625*16);
    EXPECT_EQ(result.added_tangential,.5*8.5*13);
    EXPECT_EQ(result.added_drilling,.5*8.5*16);
    tlr::ShellKineticEnergy donor_sum;
    for (std::size_t e=0;e<2;++e) {
        tlr::Vec3 d[4],v[4],w[4];
        for (std::size_t i=0;i<4;++i) {
            d[i]=director[connectivity[e][i]]; v[i]=velocity[connectivity[e][i]]; w[i]=omega[connectivity[e][i]];
        }
        tlr::ShellKineticEnergy donor;
        ASSERT_EQ(tlr::ComputeShellKineticEnergy(mass[e],d,v,w,donor),tlr::ShellMassStatus::kSuccess);
        donor_sum.translation+=donor.translation; donor_sum.physical_rotation+=donor.physical_rotation;
        donor_sum.artificial_drilling+=donor.artificial_drilling;
    }
    EXPECT_EQ(result.translation,donor_sum.translation);
    EXPECT_EQ(result.physical_rotation,donor_sum.physical_rotation);
    EXPECT_EQ(result.original_artificial_drilling,donor_sum.artificial_drilling);
    EXPECT_EQ(result.physical_rotation+result.original_artificial_drilling+
              result.added_tangential+result.added_drilling,.5*(.625+8.5)*29);
}

TEST(ShellPatchInertia, WorldDirectorEnergyCovarianceAndPureDrillingRemainSeparate) {
    const auto inertia=Assemble({Mass(1),Mass(4)});
    Nodes director,velocity,omega;
    for (std::size_t n=0;n<6;++n) {
        director[n]={.6,0,.8}; velocity[n]={double(n),.5,1}; omega[n]={.25,double(n)+1,-.5};
    }
    ShellPatchKineticEnergy before,after;
    std::string diagnostic;
    ASSERT_EQ(ComputeShellPatchKineticEnergy(inertia,director,velocity,omega,before,diagnostic),Status::kSuccess);
    // Exact proper cyclic rotation (x,y,z)->(z,x,y), independent of TL frame operations.
    const auto rotate=[](tlr::Vec3 v) { return tlr::Vec3{v.z,v.x,v.y}; };
    for (std::size_t n=0;n<6;++n) {
        director[n]=rotate(director[n]); velocity[n]=rotate(velocity[n]); omega[n]=rotate(omega[n]);
    }
    ASSERT_EQ(ComputeShellPatchKineticEnergy(inertia,director,velocity,omega,after,diagnostic),Status::kSuccess);
    Near(after.translation,before.translation); Near(after.physical_rotation,before.physical_rotation);
    Near(after.original_artificial_drilling,before.original_artificial_drilling);
    Near(after.added_tangential,before.added_tangential); Near(after.added_drilling,before.added_drilling);
    director.fill({0,1,0}); velocity.fill({}); omega.fill({0,2,0});
    ASSERT_EQ(ComputeShellPatchKineticEnergy(inertia,director,velocity,omega,after,diagnostic),Status::kSuccess);
    EXPECT_EQ(after.translation,0); EXPECT_EQ(after.physical_rotation,0); EXPECT_EQ(after.added_tangential,0);
    EXPECT_EQ(after.original_artificial_drilling,1.25); EXPECT_EQ(after.added_drilling,17);
}

TEST(ShellPatchInertia, InvalidReportAndLateKinematicsPreserveFiveOutputs) {
    const auto good=Assemble({Mass(1),Mass(4)});
    Nodes director,velocity,omega;
    director.fill({0,0,1}); velocity.fill({1,2,3}); omega.fill({2,3,4});
    const ShellPatchKineticEnergy saved{11,12,13,14,15};
    for (unsigned variant=0;variant<10;++variant) {
        auto inertia=good; auto d=director,v=velocity,w=omega; auto result=saved;
        const auto saved_bytes=Bytes(result);
        switch (variant) {
            case 0: inertia.prepared=false; break;
            case 1: inertia.counterfactual.mass[5]*=2; break;
            case 2: inertia.counterfactual.total_isotropic_inertia[5]+=inertia.original[5].artificial_drilling_inertia; break;
            case 3: inertia.added_drilling_inertia[5]*=2; break;
            case 4: inertia.original[5].artificial_drilling_inertia=0; break;
            case 5: d[5]={0,0,.99}; break;
            case 6: v[5].z=std::numeric_limits<double>::quiet_NaN(); break;
            case 7: w[5].x=std::numeric_limits<double>::infinity(); break;
            case 8: v[5].z=1e308; break;
            case 9: w[5].x=1e308; break;
        }
        std::string diagnostic;
        EXPECT_NE(ComputeShellPatchKineticEnergy(inertia,d,v,w,result,diagnostic),Status::kSuccess)<<variant;
        EXPECT_EQ(Bytes(result),saved_bytes); EXPECT_FALSE(diagnostic.empty());
        ASSERT_EQ(ComputeShellPatchKineticEnergy(good,director,velocity,omega,result,diagnostic),Status::kSuccess);
        EXPECT_TRUE(diagnostic.empty()); EXPECT_EQ(result.translation,210);
    }
}

}  // namespace
}  // namespace crash::qualification
