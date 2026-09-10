#include "YarisLocalizedSpinFixture.h"
#include <string>

namespace layered_j2_test::recurrence {
namespace pair=source_pair;
namespace {
struct Cell {
  q::ReferenceInput input;q::ReferenceData port;nq::Reference native;
  q::LayeredJ2History accepted;oracle::QephHistory expected;
  q::PrescribedInterval held;
};
void Advance(Cell& c,const q::PrescribedInterval& in,const sec::PointParameters& material,
             oracle::QephTrial& expected) {
  q::LayeredJ2ForceTrial actual;
  ASSERT_EQ(q::EvaluateLayeredJ2Force(c.port,material,c.accepted,in,actual),q::Status::kSuccess);
  ASSERT_EQ(oracle::Evaluate(c.native,c.expected,qeph_kinematics_test::NativeInterval(in),pair::Law(),expected),nq::Status::kSuccess);
  qeph_force_port_test::ForceAgreement(actual.force,expected.shell,c.input,in,cv::Tolerance);
  Sections(actual.proposed_section,expected.points);
  Diagnostics(actual.section_diagnostics,expected.section,expected.shell.kinematics.area,expected.shell.diagnostics.effective_thickness);
  ASSERT_FALSE(::testing::Test::HasFailure());
  c.accepted={actual.force.proposed_history,actual.proposed_section};
  c.expected={expected.shell.proposed_history,expected.points};
}
void Initialize(Cell (&cells)[2],double dt,const sec::PointParameters& material) {
  for(unsigned p=0;p<2;++p) {
    auto& c=cells[p];c.input=pair::Reference(p);
    ASSERT_EQ(q::InitializeReference(c.input,c.port),q::Status::kSuccess);
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(c.input),c.native),nq::Status::kSuccess);
    ASSERT_EQ(q::InitializeLayeredJ2History(c.port,material,{},c.accepted),q::Status::kSuccess);
    ASSERT_EQ(nq::InitializeHistory(c.native,{},c.expected.shell),nq::Status::kSuccess);
  }
  pair::Preload path{cells[0].native.data().frame,cells[0].input.position[1]};
  const unsigned steps=static_cast<unsigned>(64*BaseDt/dt);
  for(unsigned step=0;step<steps;++step) for(unsigned p=0;p<2;++p) {
    auto& c=cells[p];oracle::QephTrial next;c.held=path.Interval(c.input,step,dt);Advance(c,c.held,material,next);
    ASSERT_FALSE(::testing::Test::HasFailure());
  }
  for(unsigned p=0;p<2;++p) {
    const auto& points=cells[p].expected.points;
    ASSERT_GT(std::max(points[0][5],points[2][5]),1e-4);
    ASSERT_GT(std::abs(points[0][5]-points[2][5]),1e-6);
    ASSERT_GT(points[0][6],0.);
    for(unsigned n=0;n<4;++n) cells[p].held.velocity_midpoint[n]=cells[p].held.omega_midpoint[n]={};
  }
}
Vec3 Normal(Cell& c,unsigned local) {
  auto in=c.held;in.base_time=c.accepted.shell.stamp().time;in.sample_index=c.accepted.shell.stamp().sample_index+1;
  nq::Kinematics k;EXPECT_EQ(nq::EvaluatePrescribed(c.native,qeph_kinematics_test::NativeInterval(in),k),nq::Status::kSuccess);
  EXPECT_FALSE(k.planar);const auto n=k.local_normals[local];return pair::World(k.frame,{n.x,n.y,n.z});
}
q::PrescribedInterval Hold(const Cell& c,Vec3 w,unsigned local) {
  auto in=c.held;in.base_time=c.accepted.shell.stamp().time;in.sample_index=c.accepted.shell.stamp().sample_index+1;
  in.omega_midpoint[local]=w;return in;
}
double PointStressGap(const oracle::Points& a,const oracle::Points& b) {
  double gap=0;for(unsigned p=0;p<3;++p)for(unsigned c=0;c<5;++c)gap=std::max(gap,std::abs(a[p][c]-b[p][c]));return gap;
}
}

TEST(ShellLayeredNativeLargeSpin,ActualWarpedParentsDistinguishOwnNormalFromCommonNodeAxis) {
  const auto material=pair::Material();
  constexpr double angular_speed=7800,horizon=2048*0x1p-23;
  for(unsigned refinement:{1u,2u}) {
    SCOPED_TRACE(refinement);const double dt=0x1p-23/refinement;
    Cell cells[2];Initialize(cells,dt,material);ASSERT_FALSE(::testing::Test::HasFailure());
    const Vec3 normals[]{Normal(cells[0],1),Normal(cells[1],0)};
    const auto sum=Add(normals[0],normals[1]);const auto common=Scale(sum,1/std::sqrt(Dot(sum,sum)));
    const double normal_separation=std::acos(std::clamp(Dot(normals[0],normals[1]),-1.,1.));
    ASSERT_GT(normal_separation,1e-3);ASSERT_GT(angular_speed*horizon,1.9);
    RecordNumber("normal_separation_rad_h"+std::to_string(refinement),normal_separation);
    RecordNumber("localized_director_rotation_rad",angular_speed*horizon);
    double common_gap=0,own_gap=0,common_power=0,common_hg=0,common_strain=0,common_curvature=0;
    double min_area=1,max_area=1,min_thickness=1,max_thickness=1,max_dt_fraction=0,max_displacement=0;
    for(unsigned p=0;p<2;++p) {
      SCOPED_TRACE(p);auto own=cells[p],shared=cells[p],zero=cells[p];
      const auto local=pair::SelectedLocal[p];
      const unsigned steps=static_cast<unsigned>(horizon/dt);
      for(unsigned step=0;step<steps;++step) {
        SCOPED_TRACE(step);oracle::QephTrial a,b,z;
        Advance(own,Hold(own,Scale(normals[p],angular_speed),local),material,a);
        Advance(shared,Hold(shared,Scale(common,angular_speed),local),material,b);
        Advance(zero,Hold(zero,{},local),material,z);
        ASSERT_FALSE(::testing::Test::HasFailure());
        const auto& values=b.shell.proposed_history.data();
        const double ar=b.shell.kinematics.area/shared.native.data().area,tr=values.thickness/shared.input.thickness;
        min_area=std::min(min_area,ar);max_area=std::max(max_area,ar);
        min_thickness=std::min(min_thickness,tr);max_thickness=std::max(max_thickness,tr);
        max_dt_fraction=std::max(max_dt_fraction,dt/b.shell.diagnostics.unscaled_element_dt);
        for(unsigned n=0;n<4;++n) {
          const auto x=shared.held.position_endpoint[n],X=shared.input.position[n];
          max_displacement=std::max(max_displacement,std::hypot(std::hypot(x.x-X.x,x.y-X.y),x.z-X.z));
        }
        for(unsigned c=0;c<5;++c) common_strain=std::max(common_strain,std::abs(values.strain_curvature[c]));
        for(unsigned c=5;c<8;++c) common_curvature=std::max(common_curvature,
          std::abs(values.strain_curvature[c])*b.shell.diagnostics.effective_thickness);
        own_gap=std::max(own_gap,PointStressGap(a.points,z.points));
        common_gap=std::max(common_gap,PointStressGap(b.points,z.points));
        const auto c=b.shell.internal_couple[local];
        common_power=std::max(common_power,std::abs(Dot({c.x,c.y,c.z},Scale(common,angular_speed))));
        common_hg=std::max(common_hg,std::abs(b.shell.proposed_history.data().hourglass_viscous_work-
                                           z.shell.proposed_history.data().hourglass_viscous_work));
      }
    }
    // Each parent's own fixed normal is its native IDRIL0 near-null direction;
    // the common shared-node axis has tangential components in both elements.
    // Neither native match nor small own-normal effect makes the common axis
    // harmless. These are prescribed states, not a coupled nodal trajectory.
    EXPECT_LT(own_gap,1.);EXPECT_GT(common_gap,1e5);EXPECT_GT(common_power,1e-3);EXPECT_GT(common_hg,1e-8);
    // Record this fixture's non-orientation domain; these assertions neither
    // change case guards nor establish that an actual coupled impact stays here.
    EXPECT_LT(common_strain,.2);EXPECT_LT(common_curvature,.2);EXPECT_LT(max_displacement,.02);
    EXPECT_GT(min_area,.5);EXPECT_LT(max_area,1.5);EXPECT_GT(min_thickness,.5);EXPECT_LT(max_thickness,1.5);
    EXPECT_LT(max_dt_fraction,.5);
    RecordNumber("common_axis_min_area_ratio_h"+std::to_string(refinement),min_area);
    RecordNumber("common_axis_max_area_ratio_h"+std::to_string(refinement),max_area);
    RecordNumber("common_axis_min_thickness_ratio_h"+std::to_string(refinement),min_thickness);
    RecordNumber("common_axis_max_thickness_ratio_h"+std::to_string(refinement),max_thickness);
    RecordNumber("common_axis_max_dt_fraction_h"+std::to_string(refinement),max_dt_fraction);
    RecordNumber("common_axis_max_displacement_m_h"+std::to_string(refinement),max_displacement);
    RecordNumber("common_axis_max_abs_strain_h"+std::to_string(refinement),common_strain);
    RecordNumber("common_axis_max_thickness_curvature_h"+std::to_string(refinement),common_curvature);
    RecordNumber("own_normal_stress_gap_Pa_h"+std::to_string(refinement),own_gap);
    RecordNumber("common_axis_stress_gap_Pa_h"+std::to_string(refinement),common_gap);
    RecordNumber("common_axis_carried_power_abs_W_h"+std::to_string(refinement),common_power);
    RecordNumber("common_axis_signed_HOURG_work_difference_abs_J_h"+std::to_string(refinement),common_hg);
  }
}
} // namespace layered_j2_test::recurrence
