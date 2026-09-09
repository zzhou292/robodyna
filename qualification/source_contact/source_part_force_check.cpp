#include "SourcePartContactFixture.h"

#include "case/CanonicalWallArtifacts.h"
#include "case/WallTessellation.h"
#include "collision/PrescribedSurfaceContact.h"
#include "output/ArtifactIO.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <sstream>

namespace {
namespace sf=crash::qualification::source_contact;
namespace sc=tlfea::contact;
namespace cw=crash::case_data;
namespace io=crash::output;
std::filesystem::path readiness_path,wall_path;
constexpr double Stiffness=4e5,Depth=.00025,Cap=.0005,Clearance=1e-6;
constexpr double ForceBudget=5e-7,EnergyBudget=1.2500000000000005e-12;
constexpr const char* MassPolicy="qualification-e2a16-equal-native-node-lump";
using Coordinates=std::array<double,3*sf::NodeCount>;

std::string Exact(double x) {std::ostringstream s;s<<std::setprecision(17)<<x;return s.str();}
sc::VectorView View(const Coordinates& a) {return {a.data(),sf::NodeCount,3,1};}
sc::Vec3 Cross(sc::Vec3 a,sc::Vec3 b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> bytes;std::memcpy(bytes.data(),&value,sizeof(T));return bytes;
}
struct LongVector {
    long double x=0,y=0,z=0;
    LongVector operator+(LongVector b) const {return {x+b.x,y+b.y,z+b.z};}
    LongVector operator-(LongVector b) const {return {x-b.x,y-b.y,z-b.z};}
    LongVector operator*(long double b) const {return {x*b,y*b,z*b};}
};
LongVector Long(sc::Vec3 a) {return {a.x,a.y,a.z};}
long double NormCross(LongVector a,LongVector b) {
    return std::hypot(a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x);
}
long double ReferenceArea(const sf::SourcePartContactFixture& source,unsigned p) {
    const auto& parent=source.parents()[p];LongVector x[4];
    for (unsigned n=0;n<parent.arity;++n) x[n]=Long(source.positions().at(parent.local_node_indices[n]));
    if (parent.arity==3) return .5L*NormCross(x[1]-x[0],x[2]-x[0]);
    // Independent center derivative directly from original opposite edges.
    return 4*NormCross(((x[0]-x[1])+(x[3]-x[2]))*.25L,
                       ((x[0]-x[3])+(x[1]-x[2]))*.25L);
}
void Contains(sc::Q4CertifiedIntegral c,long double value) {
    EXPECT_LE(static_cast<long double>(c.lower),value);EXPECT_GE(static_cast<long double>(c.upper),value);
    EXPECT_LE(std::abs(static_cast<long double>(c.value)-value),static_cast<long double>(c.error));
}
void Same(sc::Vec3 a,sc::Vec3 b) {EXPECT_EQ(a.x,b.x);EXPECT_EQ(a.y,b.y);EXPECT_EQ(a.z,b.z);}
void Same(sc::Q4CertifiedIntegral a,sc::Q4CertifiedIntegral b) {
    EXPECT_EQ(a.value,b.value);EXPECT_EQ(a.lower,b.lower);EXPECT_EQ(a.upper,b.upper);EXPECT_EQ(a.error,b.error);
}

// This is the explicitly chosen PRESCRIBED-TEST lumping of authenticated
// per-parent midsurface proxy masses. It is neither a new production mass
// operation nor the contact A0 measure, source solver mass or rotary inertia.
struct FixtureMass {
    std::array<double,sf::NodeCount> mass{},inverse{};
    std::array<std::uint8_t,sf::NodeCount> fixed{};
    bool Initialize(const sf::SourcePartContactFixture& source) {
        FixtureMass next;
        for (unsigned p=0;p<sf::ParentCount;++p) {
            const auto& parent=source.parents()[p];
            const double share=source.surface_mass().parent_mass_kg[p]/parent.arity;
            if (!std::isfinite(share)||share<=0) return false;
            for (unsigned n=0;n<parent.arity;++n) next.mass[parent.local_node_indices[n]]+=share;
        }
        for (unsigned n=0;n<sf::NodeCount;++n) {
            if (!std::isfinite(next.mass[n])||next.mass[n]<=0) return false;
            next.inverse[n]=1/next.mass[n];
            if (!std::isfinite(next.inverse[n])||next.inverse[n]<=0) return false;
        }
        *this=next;return true;
    }
    sc::LumpedTranslationMassView view() const {
        return {inverse.data(),fixed.data(),sf::NodeCount,17,sc::TranslationMassModel::kIsotropicLumped};
    }
};
struct ParentForce {
    std::uint64_t source_id=0,feature_id=0,base_epoch=0,attempt=0;
    unsigned arity=0;
    std::array<std::uint32_t,4> nodes{}; // Capacity only: native T3 uses exactly3.
    std::array<sc::Vec3,4> force{};
    std::array<sc::Q4CertifiedIntegral,4> magnitude{};
    sc::Q4CertifiedIntegral area,resultant,potential;
    sc::Q4IntegralInterval active_area;
    unsigned cells=0,visits=0,depth_u=0,depth_v=0;
    bool valid=false;
};
struct Aggregate {
    std::array<ParentForce,sf::ParentCount> parents{};
    std::array<sc::Vec3,sf::NodeCount> forces{};
    sc::Vec3 wall_reaction,wall_moment;
    double power=0;
    bool valid=false;
};
struct Scratch {
    std::array<sc::Q4RectangularCell,sc::MaxQ4IntegrationLeaves> cells;
    std::array<std::uint32_t,sc::MaxQ4IntegrationLeaves> heap;
    sc::Q4RectangularScratch view() {return {cells.data(),heap.data(),sc::MaxQ4IntegrationLeaves,sc::MaxQ4IntegrationLeaves};}
};
static_assert(sizeof(Scratch)+sizeof(Aggregate)+sizeof(sf::SourcePartContactFixture)<1024*1024,
              "Source qualification storage must stay below1MiB excluding input parser/wall");

struct Harness {
    const sf::SourcePartContactFixture& source;
    FixtureMass mass;
    std::unique_ptr<Scratch> scratch=std::make_unique<Scratch>();
    std::string diagnostic;
    unsigned failed_parent=UINT32_MAX;
    explicit Harness(const sf::SourcePartContactFixture& source):source(source) {}
    bool EvaluateParent(unsigned,const Coordinates&,const Coordinates&,const Coordinates&,
                        const sc::PlanarWallGeometry&,ParentForce*);
    bool EvaluateAll(const Coordinates& base,const Coordinates& endpoint,const Coordinates& velocity,
                     const sc::PlanarWallGeometry& wall,Aggregate* output) {
        Aggregate next;
        for (unsigned p=0;p<sf::ParentCount;++p) {
            failed_parent=p;
            if (!EvaluateParent(p,base,endpoint,velocity,wall,&next.parents[p])) return false;
            const auto& parent=next.parents[p];next.wall_reaction.x+=parent.resultant.value;
            for (unsigned n=0;n<parent.arity;++n) {
                const auto node=parent.nodes[n];const auto force=parent.force[n];
                next.forces[node]=sc::Add(next.forces[node],force);
                next.wall_moment=sc::Subtract(next.wall_moment,Cross(View(endpoint).at(node),force));
                next.power+=sc::Dot(View(velocity).at(node),force);
            }
        }
        next.valid=true;*output=next;failed_parent=UINT32_MAX;return true;
    }
};

// One original parent at a time through the owning finite-wall dispatcher.
// No triangle preflight/coverage or force integration is copied into this test.
bool Harness::EvaluateParent(unsigned p,const Coordinates& base,const Coordinates& endpoint,
                             const Coordinates& velocity,const sc::PlanarWallGeometry& wall,ParentForce* output) {
    diagnostic.clear();
    if (!output || p>=sf::ParentCount) return false;
    const auto& binding=source.parents()[p];
    sc::Q4ParametricReference quad_reference;sc::T3MaterialMeasure triangle_reference;
    sc::SurfaceQ4 quad;sc::SurfaceTriangle triangle;sc::PrescribedSurfaceParent request;
    ParentForce next;next.arity=binding.arity;
    if (binding.arity==4) {
        if (!source.q4_parent(p,quad)) return false;
        const auto prepared=quad_reference.Initialize(source.positions(),&quad,1);
        if (prepared.status!=sc::Q4ParametricStatus::Ok) {diagnostic=prepared.message;return false;}
        next.area=quad_reference.parent(0).area;
        request.family=sc::PrescribedSurfaceFamily::Q4CenterAreaUniformNatural;
        request.q4={&quad_reference,0,&quad,&quad};
    } else {
        if (!source.t3_parent(p,triangle)) return false;
        if (sc::PrepareT3MaterialMeasure(source.positions(),triangle,&triangle_reference)!=sc::SurfaceMeasureStatus::Ok ||
            !sc::q4_bounds::Certify(.5*triangle_reference.density().value,triangle_reference.area_enclosure(),&next.area)) {
            diagnostic="Source native triangle reference preparation failed";return false;
        }
        request.family=sc::PrescribedSurfaceFamily::T3NativeLinear;
        request.t3={&triangle_reference,&triangle,&triangle};
    }
    sc::PrescribedSurfaceConfig config;
    config.stiffness_per_area=Stiffness;config.maximum_penetration=Cap;config.exposed_clearance=Clearance;
    config.q4.force_error=ForceBudget;config.q4.energy_error=EnergyBudget;
    config.t3={ForceBudget,EnergyBudget};
    const sc::PrescribedSurfaceInput input{View(base),View(velocity),View(endpoint),View(velocity),mass.view(),&request,1};
    sc::PrescribedSurfaceResult result;
    const auto report=sc::IntegratePrescribedSurfaceContact(wall,input,config,31,scratch->view(),&result);
    if (report.status!=sc::PrescribedSurfaceStatus::Ok) {
        std::ostringstream message;message<<report.message<<"; source="<<binding.source_id
            <<" status="<<static_cast<int>(report.status)<<" q4="<<static_cast<int>(report.q4.status)
            <<" t3="<<static_cast<int>(report.t3.status);diagnostic=message.str();return false;
    }
    if (!result.valid || result.parent_count!=1 || result.parents[0].family!=request.family ||
        !result.parents[0].coverage.covered) {diagnostic="Incomplete owning dispatcher result";return false;}
    if (binding.arity==4) {
        const auto& rectangular=result.parents[0].q4;const auto& integral=rectangular.integration;
        for (unsigned n=0;n<4;++n) {
            next.nodes[n]=integral.nodal.nodes[n];next.force[n]=integral.nodal.forces[n];next.magnitude[n]=integral.force[n];
            const auto c=integral.nodal.couples[n];if (c.x!=0||c.y!=0||c.z!=0) return false;
        }
        next.source_id=integral.parent_element_id;next.feature_id=integral.feature_id;
        next.base_epoch=integral.base_epoch;next.attempt=integral.attempt;
        next.resultant=integral.resultant;next.potential=integral.potential;next.active_area=integral.active_area;
        next.cells=integral.leaf_count;next.visits=integral.visited;
        next.depth_u=rectangular.deepest_u;next.depth_v=rectangular.deepest_v;next.valid=integral.valid;
    } else {
        const auto& integral=result.parents[0].t3;
        for (unsigned n=0;n<3;++n) {
            next.nodes[n]=integral.nodal.nodes[n];next.force[n]=integral.nodal.forces[n];next.magnitude[n]=integral.force[n];
        }
        if (integral.parent_face_id!=triangle.parent_face_id) return false;
        next.source_id=integral.parent_element_id;next.feature_id=integral.feature_id;
        next.base_epoch=integral.base_epoch;next.attempt=integral.attempt;
        next.resultant=integral.resultant;next.potential=integral.potential;next.active_area=integral.active_area;
        next.cells=integral.subtriangle_count;next.visits=integral.sample_count;next.valid=integral.valid;
    }
    if (!next.valid) {diagnostic="Invalid native integration result";return false;}
    *output=next;return true;
}

Coordinates Shift(const sf::SourcePartContactFixture& source,double translation) {
    auto x=source.coordinates();
    for (unsigned n=0;n<sf::NodeCount;++n) x[3*n]+=translation;
    return x;
}
Coordinates Velocity(double translation) {
    Coordinates v{};
    // Explicit straight prescribed path over1second, not a solver clock/step.
    for (unsigned n=0;n<sf::NodeCount;++n) v[3*n]=translation;
    return v;
}
double ParentShift(const sf::SourcePartContactFixture& source,unsigned p,double wall_x) {
    double maximum=-std::numeric_limits<double>::infinity();const auto& parent=source.parents()[p];
    for (unsigned n=0;n<parent.arity;++n) maximum=std::max(maximum,source.positions().at(parent.local_node_indices[n]).x);
    return (wall_x-maximum)+Depth;
}
double WholeShift(const sf::SourcePartContactFixture& source,double wall_x) {
    double maximum=-std::numeric_limits<double>::infinity();
    for (unsigned n=0;n<sf::NodeCount;++n) maximum=std::max(maximum,source.positions().at(n).x);
    return (wall_x-maximum)+Depth;
}
void CheckParent(const sf::SourcePartContactFixture& source,unsigned p,const ParentForce& actual) {
    const auto& parent=source.parents()[p];ASSERT_TRUE(actual.valid);EXPECT_EQ(actual.arity,parent.arity);
    EXPECT_EQ(actual.source_id,parent.source_id);EXPECT_EQ(actual.feature_id,parent.source_id);
    EXPECT_EQ(actual.base_epoch,17u);EXPECT_EQ(actual.attempt,31u);
    Contains(actual.area,ReferenceArea(source,p));
    EXPECT_LE(actual.resultant.error,ForceBudget);EXPECT_LE(actual.potential.error,EnergyBudget);
    for (unsigned n=0;n<actual.arity;++n) {
        EXPECT_EQ(actual.nodes[n],parent.local_node_indices[n]);EXPECT_EQ(actual.force[n].x,-actual.magnitude[n].value);
        EXPECT_EQ(actual.force[n].y,0);EXPECT_EQ(actual.force[n].z,0);EXPECT_LE(actual.magnitude[n].error,ForceBudget);
    }
    if (parent.arity==4) {
        EXPECT_LE(actual.cells,sc::MaxQ4IntegrationLeaves);EXPECT_LE(actual.visits,sc::MaxQ4IntegrationVisits);
        EXPECT_LE(actual.depth_u,sc::MaxQ4IntegrationDepth);EXPECT_LE(actual.depth_v,sc::MaxQ4IntegrationDepth);
    } else {EXPECT_LE(actual.cells,2u);EXPECT_LE(actual.visits,6u);}
}
void SameParentForce(const ParentForce& a,const ParentForce& b) {
    EXPECT_EQ(a.source_id,b.source_id);EXPECT_EQ(a.arity,b.arity);EXPECT_EQ(a.nodes,b.nodes);
    Same(a.area,b.area);Same(a.resultant,b.resultant);Same(a.potential,b.potential);
    for (unsigned n=0;n<a.arity;++n) {Same(a.force[n],b.force[n]);Same(a.magnitude[n],b.magnitude[n]);}
    EXPECT_EQ(a.active_area.lower,b.active_area.lower);EXPECT_EQ(a.active_area.upper,b.active_area.upper);
    EXPECT_EQ(a.cells,b.cells);EXPECT_EQ(a.visits,b.visits);EXPECT_EQ(a.depth_u,b.depth_u);EXPECT_EQ(a.depth_v,b.depth_v);
}
class SourcePartForceCheck:public ::testing::Test {
  protected:
    sf::SourcePartContactFixture source;
    cw::CanonicalWall wall;
    std::string wall_bytes;
    void SetUp() override {
        const auto loaded=sf::LoadPinnedSourcePartContact(readiness_path,&source);
        ASSERT_EQ(loaded.status,sf::FixtureStatus::Ok)<<loaded.diagnostic;
        wall_bytes=cw::ReadPinnedWallManifest(wall_path.string());std::istringstream input(wall_bytes);
        ASSERT_EQ(wall.Load(input).status,cw::WallStatus::Ok);
    }
};

TEST_F(SourcePartForceCheck, AuthenticatedParentMassesAreLumpedOnceIntoTheRealSharedNodeSpace) {
    Harness harness(source);ASSERT_TRUE(harness.mass.Initialize(source));
    std::array<long double,sf::NodeCount> independent{};long double parents=0,nodes=0;
    for (unsigned p=0;p<sf::ParentCount;++p) {
        const auto& binding=source.parents()[p];const long double mass=source.surface_mass().parent_mass_kg[p];parents+=mass;
        for (unsigned n=0;n<binding.arity;++n) independent[binding.local_node_indices[n]]+=mass/binding.arity;
    }
    for (unsigned n=0;n<sf::NodeCount;++n) {
        nodes+=harness.mass.mass[n];ASSERT_GT(independent[n],0);
        EXPECT_LE(std::abs(static_cast<long double>(harness.mass.mass[n])-independent[n]),64*std::numeric_limits<double>::epsilon()*independent[n]);
        EXPECT_NEAR(harness.mass.mass[n]*harness.mass.inverse[n],1,2*std::numeric_limits<double>::epsilon());
        EXPECT_EQ(harness.mass.fixed[n],0);
    }
    const auto allowance=64*std::numeric_limits<double>::epsilon()*parents;
    EXPECT_LE(std::abs(nodes-parents),allowance);
    EXPECT_LE(std::abs(parents-source.surface_mass().total_mass_kg),allowance);
    const auto range=std::minmax_element(harness.mass.mass.begin(),harness.mass.mass.end());
    RecordProperty("minimum_supplied_node_mass_kg",Exact(*range.first));RecordProperty("maximum_supplied_node_mass_kg",Exact(*range.second));
    RecordProperty("physical_node_count",117);RecordProperty("mass_lumping_policy",MassPolicy);
    RecordProperty("source_mass_equivalence_qualified","false");RecordProperty("mechanics_mass_admitted","false");
}

TEST_F(SourcePartForceCheck, AllOriginalParentsExecuteSeparatedAndShapePreservingPartialContact) {
    const auto source_before=Bytes(source);Harness harness(source);ASSERT_TRUE(harness.mass.Initialize(source));
    cw::WallTessellation prepared;
    ASSERT_EQ(prepared.Initialize(wall,wall_bytes,cw::WallTessellationKind::Original).status,cw::WallTessellationStatus::Ok);
    const auto& geometry=*prepared.geometry();Coordinates zero{};
    unsigned quads=0,triangles=0,unresolved_positive=0;
    std::set<std::uint64_t> seen,mixed_seen;
    for (unsigned p=0;p<sf::ParentCount;++p) {
        const auto& parent=source.parents()[p];SCOPED_TRACE(parent.source_id);
        ParentForce separated;
        ASSERT_TRUE(harness.EvaluateParent(p,source.coordinates(),source.coordinates(),zero,geometry,&separated))<<harness.diagnostic;
        CheckParent(source,p,separated);EXPECT_EQ(separated.resultant.value,0);EXPECT_EQ(separated.potential.value,0);
        EXPECT_EQ(separated.resultant.upper,0);EXPECT_EQ(separated.potential.upper,0);
        const double translation=ParentShift(source,p,geometry.wall_x());const auto endpoint=Shift(source,translation);
        const auto velocity=Velocity(translation);ParentForce result;
        ASSERT_TRUE(harness.EvaluateParent(p,source.coordinates(),endpoint,velocity,geometry,&result))<<harness.diagnostic;
        CheckParent(source,p,result);EXPECT_TRUE(seen.insert(parent.source_id).second);
        if (parent.arity==4) ++quads;else ++triangles;
        if (parent.source_id==2126280 || parent.source_id==2126284 || parent.source_id==2213539) mixed_seen.insert(parent.source_id);
        long double minimum=1,maximum=-1;
        for (unsigned n=0;n<parent.arity;++n) {
            const auto i=parent.local_node_indices[n];
            EXPECT_EQ(io::Bits(endpoint[3*i+1]),io::Bits(source.coordinates()[3*i+1]));
            EXPECT_EQ(io::Bits(endpoint[3*i+2]),io::Bits(source.coordinates()[3*i+2]));
            const long double gap=static_cast<long double>(endpoint[3*i])-geometry.wall_x();
            minimum=std::min(minimum,gap);maximum=std::max(maximum,gap);
        }
        ASSERT_GT(maximum,0);EXPECT_LE(maximum,Cap);const long double area=ReferenceArea(source,p);
        // Independent positive-corner patch lower bound. For Q4, Ncorner >=
        // (1-l)^2 on a corner square; for T3, Ncorner >=1-l on a corner simplex.
        // The chosen l ensures g>=gmax/2 there; physical area fraction is l².
        const long double length=minimum>0 ? 1 : maximum/((parent.arity==4 ? 4 : 2)*(maximum-minimum));
        const long double depth_floor=minimum>0 ? minimum : maximum*.5L;
        const long double lower_force=Stiffness*area*length*length*depth_floor;
        const long double lower_energy=.5L*Stiffness*area*length*length*depth_floor*depth_floor;
        EXPECT_GE(static_cast<long double>(result.resultant.upper),lower_force);
        EXPECT_GE(static_cast<long double>(result.potential.upper),lower_energy);
        EXPECT_LE(static_cast<long double>(result.resultant.lower),Stiffness*area*maximum);
        EXPECT_LE(static_cast<long double>(result.potential.lower),.5L*Stiffness*area*maximum*maximum);
        if (result.resultant.lower==0) ++unresolved_positive;
        const std::string key="parent_"+std::to_string(parent.source_id);
        RecordProperty(key+"_force_error_N",Exact(result.resultant.error));RecordProperty(key+"_potential_error_J",Exact(result.potential.error));
        RecordProperty(key+"_cells",static_cast<int>(result.cells));RecordProperty(key+"_visits",static_cast<int>(result.visits));
    }
    EXPECT_EQ(quads,88u);EXPECT_EQ(triangles,6u);EXPECT_EQ(seen.size(),94u);EXPECT_EQ(mixed_seen.size(),3u);
    EXPECT_EQ(Bytes(source),source_before);RecordProperty("independent_parent_experiments",94);
    RecordProperty("positive_pressure_not_separated_from_zero_by_force_certificate",unresolved_positive);
    RecordProperty("one_consistent_wholepart_trajectory","false");RecordProperty("source_formulation_equivalence","false");
}

TEST_F(SourcePartForceCheck, OneCoherentWholepartProfileAssemblesRealSharedForcesAndRejectsTransactionally) {
    Harness harness(source);ASSERT_TRUE(harness.mass.Initialize(source));cw::WallTessellation prepared;
    ASSERT_EQ(prepared.Initialize(wall,wall_bytes,cw::WallTessellationKind::Original).status,cw::WallTessellationStatus::Ok);
    const auto& geometry=*prepared.geometry();const double translation=WholeShift(source,geometry.wall_x());
    const auto endpoint=Shift(source,translation),velocity=Velocity(translation);Aggregate result;
    ASSERT_TRUE(harness.EvaluateAll(source.coordinates(),endpoint,velocity,geometry,&result))<<harness.diagnostic;
    ASSERT_TRUE(result.valid);ASSERT_GT(result.wall_reaction.x,0);
    long double sum=0,power=0,moment_y=0,moment_z=0,scale=0;
    for (unsigned p=0;p<sf::ParentCount;++p) CheckParent(source,p,result.parents[p]);
    for (unsigned n=0;n<sf::NodeCount;++n) {
        long double independent=0;
        for (const auto& parent:result.parents) for (unsigned i=0;i<parent.arity;++i)
            if (parent.nodes[i]==n) independent+=parent.force[i].x;
        EXPECT_LE(std::abs(static_cast<long double>(result.forces[n].x)-independent),
                  64*std::numeric_limits<double>::epsilon()*std::abs(independent));
        EXPECT_EQ(result.forces[n].y,0);EXPECT_EQ(result.forces[n].z,0);
        const long double force=result.forces[n].x;sum+=force;scale+=std::abs(force);
        power+=velocity[3*n]*force;moment_y+=endpoint[3*n+2]*force;moment_z-=endpoint[3*n+1]*force;
    }
    const auto allowance=256*std::numeric_limits<double>::epsilon()*scale;
    EXPECT_LE(std::abs(sum+result.wall_reaction.x),allowance);
    EXPECT_LE(std::abs(moment_y+result.wall_moment.y),allowance);
    EXPECT_LE(std::abs(moment_z+result.wall_moment.z),allowance);
    EXPECT_LE(std::abs(power-result.power),allowance);
    EXPECT_LE(std::abs(power+translation*result.wall_reaction.x),allowance);
    const auto before=Bytes(result);std::array<unsigned,sf::NodeCount> first;first.fill(sf::ParentCount);
    for (unsigned p=0;p<sf::ParentCount;++p) for (unsigned n=0;n<source.parents()[p].arity;++n) {
        const auto i=source.parents()[p].local_node_indices[n];first[i]=std::min(first[i],p);
    }
    const auto late=static_cast<unsigned>(std::max_element(first.begin(),first.end())-first.begin());
    ASSERT_GT(first[late],0u);const double saved=harness.mass.inverse[late];harness.mass.inverse[late]=0;
    EXPECT_FALSE(harness.EvaluateAll(source.coordinates(),endpoint,velocity,geometry,&result));
    EXPECT_EQ(harness.failed_parent,first[late]);EXPECT_EQ(Bytes(result),before);
    harness.mass.inverse[late]=saved;Aggregate retry;
    ASSERT_TRUE(harness.EvaluateAll(source.coordinates(),endpoint,velocity,geometry,&retry))<<harness.diagnostic;
    for (unsigned p=0;p<sf::ParentCount;++p) SameParentForce(result.parents[p],retry.parents[p]);
    for (unsigned n=0;n<sf::NodeCount;++n) Same(result.forces[n],retry.forces[n]);
    RecordProperty("coherent_prescribed_profile","one shared rigid X translation over1second; no dynamics");
    RecordProperty("wholepart_shift_m",Exact(translation));RecordProperty("wall_reaction_N",Exact(result.wall_reaction.x));
}

TEST_F(SourcePartForceCheck, EveryParentKeepsTheSameCertifiedForceAcrossActualWallTessellations) {
    Harness harness(source);ASSERT_TRUE(harness.mass.Initialize(source));std::array<ParentForce,sf::ParentCount> baseline;
    const cw::WallTessellationKind kinds[]={cw::WallTessellationKind::Original,
        cw::WallTessellationKind::FlipConvexPairs,cw::WallTessellationKind::UniformFour};
    for (unsigned variant=0;variant<3;++variant) {
        SCOPED_TRACE(variant);cw::WallTessellation prepared;
        ASSERT_EQ(prepared.Initialize(wall,wall_bytes,kinds[variant]).status,cw::WallTessellationStatus::Ok);
        const auto& geometry=*prepared.geometry();
        for (unsigned p=0;p<sf::ParentCount;++p) {
            SCOPED_TRACE(source.parents()[p].source_id);
            const double translation=ParentShift(source,p,geometry.wall_x());const auto endpoint=Shift(source,translation);
            const auto velocity=Velocity(translation);ParentForce actual;
            ASSERT_TRUE(harness.EvaluateParent(p,source.coordinates(),endpoint,velocity,geometry,&actual))<<harness.diagnostic;
            CheckParent(source,p,actual);
            if (variant==0) baseline[p]=actual;else SameParentForce(baseline[p],actual);
        }
    }
    RecordProperty("wall_variants",3);RecordProperty("native_source_parents_per_variant",94);
    RecordProperty("physical_shell_mesh_refinement_qualified","false");
}
} // namespace

int main(int argc,char** argv) {
    ::testing::InitGoogleTest(&argc,argv);
    if (argc!=3) {std::cerr<<"Usage: source_part_force_check pinned-readiness.json canonical-wall.manifest.json [gtest options]\n";return 2;}
    readiness_path=argv[1];wall_path=argv[2];
    if (!std::filesystem::is_regular_file(readiness_path)||!std::filesystem::is_regular_file(wall_path)) {
        std::cerr<<"Both authenticated source fixtures are required\n";return 2;
    }
    return RUN_ALL_TESTS();
}
