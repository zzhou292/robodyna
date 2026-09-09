#include "SourcePartContactFixture.h"
#include "lib_utest/qualification/qeph/QephKinematicsFixture.h"

#include <filesystem>
#include <iostream>
#include <set>
#include <sstream>

namespace {
namespace source=crash::qualification::source_contact;
namespace port=tl::fea::qeph;
namespace native=tl::qualification::qeph;
namespace startup=qeph_startup_test;
namespace rates=qeph_kinematics_test;
std::filesystem::path readiness_path;
std::string Exact(double value) { std::ostringstream text; text<<std::setprecision(17)<<value; return text.str(); }

port::ReferenceInput Input(const source::SourcePartContactFixture& fixture,unsigned p) {
  port::ReferenceInput input;
  const auto& parent=fixture.parents()[p];
  for(unsigned n=0;n<4;++n) {
    const auto index=parent.local_node_indices[n];
    const auto x=fixture.positions().at(index);
    input.position[n]={x.x,x.y,x.z}; input.node_ids[n]=fixture.nodes()[index].source_id;
  }
  input.density=fixture.surface_mass().density_kg_m3;
  input.thickness=fixture.surface_mass().thickness_m;
  input.young_modulus=200e9; input.poisson_ratio=.3;
  return input;
}

// Independent mean-plane area from the two world diagonals. This is neither
// the display triangle area nor an exact variable bilinear surface measure.
long double Area(const port::ReferenceInput& in) {
  const auto& p=in.position;
  const long double a[]{static_cast<long double>(p[2].x)-p[0].x,
                        static_cast<long double>(p[2].y)-p[0].y,
                        static_cast<long double>(p[2].z)-p[0].z};
  const long double b[]{static_cast<long double>(p[3].x)-p[1].x,
                        static_cast<long double>(p[3].y)-p[1].y,
                        static_cast<long double>(p[3].z)-p[1].z};
  return .5L*std::hypot(a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]);
}

class SourcePartQephGeometryCheck:public ::testing::Test {
 protected:
  source::SourcePartContactFixture fixture;
  void SetUp() override {
    const auto report=source::LoadPinnedSourcePartContact(readiness_path,&fixture);
    ASSERT_EQ(report.status,source::FixtureStatus::Ok)<<report.diagnostic;
  }
};

TEST_F(SourcePartQephGeometryCheck, AllOriginalQuadsMatchNativeStartupWithoutCoordinateRepair) {
  std::set<std::uint64_t> visited;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=4) continue;
    SCOPED_TRACE(parent.source_id);
    const auto input=Input(fixture,p); port::ReferenceData actual; native::Reference expected;
    ASSERT_EQ(native::Initialize(startup::NativeInput(input),expected),native::Status::kSuccess);
    ASSERT_EQ(port::InitializeReference(input,actual),port::Status::kSuccess);
    startup::Agreement(actual,expected.data()); startup::SameInput(actual.input,input);
    startup::Near(actual.area,static_cast<double>(Area(input)),startup::Scale(input)*startup::Scale(input));
    for(unsigned n=0;n<4;++n) EXPECT_EQ(actual.input.node_ids[n],parent.raw_record[2+n]);
    ASSERT_TRUE(visited.insert(parent.source_id).second);
  }
  EXPECT_EQ(visited.size(),source::Q4Count);
  RecordProperty("source_readiness_sha256",source::ReadinessSha256);
  RecordProperty("original_q4_count",static_cast<int>(source::Q4Count));
  RecordProperty("material_experiment","QEPH startup E=200GPa nu=0.3; source rho/thickness unchanged; MAT024 not consumed");
}

TEST_F(SourcePartQephGeometryCheck, OriginalSharedNodesReceiveEachNativeMassAndInertiaContributionOnce) {
  std::array<long double,source::NodeCount> mass{},inertia{},mass_truth{},inertia_truth{};
  long double total_mass=0,total_inertia=0; unsigned visited=0;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=4) continue;
    SCOPED_TRACE(parent.source_id);
    const auto in=Input(fixture,p); port::ReferenceData result;
    ASSERT_EQ(port::InitializeReference(in,result),port::Status::kSuccess);
    const long double area=Area(in),t=in.thickness,m=in.density*t*area/4;
    const long double physical=m*t*t/12,added=m*area/12,total=physical+added;
    for(unsigned n=0;n<4;++n) {
      const auto i=parent.local_node_indices[n];
      startup::Near(result.physical_inertia[n],static_cast<double>(physical),static_cast<double>(physical));
      startup::Near(result.added_inertia[n],static_cast<double>(added),static_cast<double>(added));
      mass[i]+=result.nodal_mass[n]; inertia[i]+=result.isotropic_inertia[n];
      mass_truth[i]+=m; inertia_truth[i]+=total;
    }
    total_mass+=4*m; total_inertia+=4*total; ++visited;
  }
  EXPECT_EQ(visited,source::Q4Count);
  long double assembled_mass=0,assembled_inertia=0;
  for(unsigned i=0;i<source::NodeCount;++i) {
    if(mass_truth[i]>0) {
      startup::Near(static_cast<double>(mass[i]),static_cast<double>(mass_truth[i]),static_cast<double>(mass_truth[i]));
      startup::Near(static_cast<double>(inertia[i]),static_cast<double>(inertia_truth[i]),static_cast<double>(inertia_truth[i]));
    } else { EXPECT_EQ(mass[i],0); EXPECT_EQ(inertia[i],0); }
    assembled_mass+=mass[i]; assembled_inertia+=inertia[i];
  }
  startup::Near(static_cast<double>(assembled_mass),static_cast<double>(total_mass),static_cast<double>(total_mass));
  startup::Near(static_cast<double>(assembled_inertia),static_cast<double>(total_inertia),static_cast<double>(total_inertia));
  RecordProperty("q4_assembled_mass_kg",Exact(static_cast<double>(assembled_mass)));
  RecordProperty("q4_assembled_isotropic_inertia_kg_m2",Exact(static_cast<double>(assembled_inertia)));
  RecordProperty("full_part_structural_mass_admitted","false");
}

TEST_F(SourcePartQephGeometryCheck, EveryOriginalQuadMatchesAllNativeCurrentFieldsForThreePrescribedPatterns) {
  unsigned visited=0;
  for(unsigned p=0;p<source::ParentCount;++p) {
    const auto& parent=fixture.parents()[p]; if(parent.arity!=4) continue;
    SCOPED_TRACE(parent.source_id);
    const auto input=Input(fixture,p); port::ReferenceData ref; native::Reference native_ref;
    ASSERT_EQ(port::InitializeReference(input,ref),port::Status::kSuccess);
    ASSERT_EQ(native::Initialize(startup::NativeInput(input),native_ref),native::Status::kSuccess);
    const auto saved=startup::Bytes(ref);
    for(unsigned pattern=0;pattern<3;++pattern) {
      SCOPED_TRACE(pattern);
      const auto interval=rates::Pattern(input,pattern); port::Kinematics actual; native::Kinematics expected;
      ASSERT_EQ(native::EvaluatePrescribed(native_ref,rates::NativeInterval(interval),expected),native::Status::kSuccess);
      ASSERT_EQ(port::EvaluatePrescribed(ref,interval,actual),port::Status::kSuccess);
      rates::Agreement(actual,expected,interval);
      if(pattern==0) rates::ZeroRates(actual,interval);
      EXPECT_EQ(startup::Bytes(ref),saved); ++visited;
    }
  }
  EXPECT_EQ(visited,3*source::Q4Count);
  RecordProperty("native_parity_configurations",static_cast<int>(visited));
  RecordProperty("force_or_dynamics_qualified","false");
}
} // namespace

int main(int argc,char** argv) {
  ::testing::InitGoogleTest(&argc,argv);
  if(argc!=2) { std::cerr<<"Usage: source_part_qeph_geometry_check READINESS.json\n"; return 2; }
  readiness_path=argv[1];
  return RUN_ALL_TESTS();
}
