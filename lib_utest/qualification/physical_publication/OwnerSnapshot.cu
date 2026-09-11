// SPDX-License-Identifier: AGPL-3.0-or-later
#include "OwnerFixture.h"
#include "../qt_mapped/OwnerFixture.h"
#include "../qbat_resident/ResultValues.h"
#include "../type25/EvaluationValues.h"
#include "../type13_recurrence/Fixture.h"
#include "../solid_resident/ResultValues.h"

namespace physical_publication_test {
namespace {
template<class Values> void Append(Snapshot& output,const Values& values) {
  for (double value : values) output.values.push_back(Bits(value));
}
void Vector(Snapshot& output,tl::math::Vec3 value) {
  output.values.insert(output.values.end(),{Bits(value.x),Bits(value.y),Bits(value.z)});
}
}
bool Rig::Read(Snapshot& output) {
  Snapshot next;
  next.stamp = owner.accepted();
  if (!Good(publication.CopyAcceptedPhysicalDiagnostics(next.stamp,&next.diagnostics))) return false;
  const auto n = fixture.domain.node_count();
  std::vector<double> fields(19*n),coefficients(2*n+3);
  fe::NodalStamp copied;
  if (!Good(owner.CopyAccepted({fields.data(),fields.data()+3*n,n,fields.data()+6*n,
      fields.data()+10*n,fields.data()+13*n,fields.data()+16*n},&copied)) ||
      !Good(owner.CopyAcceptedCin({coefficients.data(),coefficients.data()+n,
          coefficients.data()+2*n,coefficients.data()+2*n+1,coefficients.data()+2*n+2,n,1},&copied))) return false;
  Append(next,fields);
  Append(next,coefficients);
  std::vector<fe::NodalRigidGroupSnapshot> groups(fixture.rigid.groups().size());
  if (!Good(owner.CopyAcceptedRigidGroups({groups.data(),groups.size()},&copied))) return false;
  for (const auto& group : groups) {
    next.values.push_back(group.source_group_id);
    next.values.push_back(group.source_node_set_id);
    next.values.push_back(static_cast<unsigned>(group.source_kind));
    Vector(next,group.state.center);
    Vector(next,group.state.velocity);
    Vector(next,group.state.omega);
    Append(next,group.state.principal_axes.v);
  }
  std::vector<fe::qeph::ForceTrial> q(2);
  std::vector<fe::t3::ForceTrial> t(1);
  fe::qeph::BatchDiagnostics qd;
  fe::t3::BatchDiagnostics td;
  if (!Good(qeph.CopyAcceptedResults(next.stamp,q.data(),q.size(),&qd)) ||
      !Good(t3.CopyAcceptedResults(next.stamp,t.data(),t.size(),&td))) return false;
  Append(next,qt_mapped_test::Values(q));
  Append(next,qt_mapped_test::Values(t));
  std::vector<fe::ShellBatchLayeredSection> qsection(2),tsection(1);
  std::vector<fe::ShellBatchFailureState> qfailure(2);
  if (!Good(qeph.CopyAcceptedLayeredSectionHistory(next.stamp,qsection.data(),2,&qd)) ||
      !Good(t3.CopyAcceptedLayeredSectionHistory(next.stamp,tsection.data(),1,&td)) ||
      !Good(qeph.CopyAcceptedFailureHistory(next.stamp,qfailure.data(),2,&qd))) return false;
  for (const auto& value : qsection) qt_mapped_test::Add(next.values,value);
  for (const auto& value : tsection) qt_mapped_test::Add(next.values,value);
  for (const auto& value : qfailure) qt_mapped_test::Add(next.values,value);
  std::uint8_t qa[2]{},ta[1]{},ba[1]{};
  fe::qbat::BatchDiagnostics bd;
  if (!Good(qeph.CopyAcceptedParentActivity(next.stamp,qa,2,&qd)) ||
      !Good(t3.CopyAcceptedParentActivity(next.stamp,ta,1,&td)) ||
      !Good(qbat.CopyAcceptedParentActivity(next.stamp,ba,1,&bd))) return false;
  next.values.insert(next.values.end(),{qa[0],qa[1],ta[0],ba[0]});
  fe::qbat::BatchResult b;
  if (!Good(qbat.CopyAcceptedResults(next.stamp,&b,1,&bd))) return false;
  const auto bvalues = qbat_resident_test::ResultValues(b);
  next.values.insert(next.values.end(),bvalues.begin(),bvalues.end());
  fe::type25::Evaluation weld[2];
  fe::type25::BatchDiagnostics wd;
  if (!Good(welds.CopyAcceptedResults(next.stamp,weld,2,&wd))) return false;
  for (const auto& value : weld) {
    Append(next,type25_test::EvaluationValues(value));
    next.values.push_back(value.history.active);
  }
  fe::type13::Evaluation beam;
  fe::type13::BatchDiagnostics beam_diagnostics;
  if (!Good(beams.CopyAcceptedResults(next.stamp,&beam,1,&beam_diagnostics))) return false;
  Append(next,type13_recurrence_test::Values(beam));
  solid_resident_test::Results result;
  fe::solids::BatchDiagnostics solid_diagnostics;
  if (!Good(solids.CopyAcceptedResults(next.stamp,result.Buffers(),&solid_diagnostics))) return false;
  Append(next,solid_resident_test::Values(result.a));
  Append(next,solid_resident_test::Values(result.b));
  Append(next,solid_resident_test::Values(result.c));
  next.values.insert(next.values.end(),{result.a.stamp.sample_index,result.b.stamp.sample_index,result.c.stamp.sample_index,
      Bits(result.a.stamp.time_s),Bits(result.b.stamp.time_s),Bits(result.c.stamp.time_s)});
  output = std::move(next);
  return true;
}
} // namespace physical_publication_test
