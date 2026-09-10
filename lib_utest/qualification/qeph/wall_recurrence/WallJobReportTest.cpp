#include "WallJobReportTestFixture.h"
#include <cmath>

namespace tl::qualification::qeph::wall_recurrence {
namespace test=report_test;
namespace io=crash::output;
TEST(QephWallJobReport, LateContextIsWrittenOnceAndOperatorsUseBoundHashes) {
  test::Fixture fixture; auto job=fixture.Job(); fixture.Prepare(job,1); fixture.Prepare(job,2);
  auto& populated=job.steps[0].amplitudes[1].branches.branches[1];
  populated.spectrum.complete=true; populated.spectrum.matrix_norm=2; populated.spectrum.spectral_radius=1.108;
  for(unsigned i=0;i<109;++i) populated.spectrum.eigenvalues.emplace_back(1+.001*i,.0001*i);
  auto& gram=populated.continuous.raw; gram.complete=true; gram.within_gain_budget=true;
  gram.ordinary_steps=1; gram.state_count=2; gram.eigenvalues=Eigen::VectorXd::LinSpaced(109,1,109);
  gram.minimum_eigenvalue=1; gram.maximum_eigenvalue=109; gram.gram_norm=gram.eigenvalues.norm();
  gram.mean_gain=std::sqrt(109./2); gram.individual_power_bound=std::sqrt(109.); gram.controlling_coordinate=108;
  gram.controlling_direction=Eigen::VectorXd::Zero(109); gram.controlling_direction[108]=1;
  const auto path=fixture.directory.path/"derived";
  WallJobReportWriter writer(path,fixture.raw,fixture.binding,test::rt::Provenance(),"synthetic-analysis.json",ScreenSetByteCap);
  writer(job,{WallJobProgressKind::Amplitude,0,0});
  EXPECT_FALSE(test::rt::Read(path/"amplitude-h0-a0.json")["context_available"].GetBool());
  EXPECT_FALSE(std::filesystem::exists(path/"context-h0.json"));
  writer(job,{WallJobProgressKind::Amplitude,0,1}); const auto context=io::ReadBounded(path/"context-h0.json",RawFileByteCap);
  writer(job,{WallJobProgressKind::Amplitude,0,2});
  EXPECT_EQ(io::ReadBounded(path/"context-h0.json",RawFileByteCap),context);
  unsigned contexts=0; for(const auto& f:writer.receipt().files) contexts+=f.name=="context-h0.json"; EXPECT_EQ(contexts,1u);
  const auto amplitude=test::rt::Read(path/"amplitude-h0-a1.json");
  EXPECT_EQ(amplitude["raw_index_sha256"].GetString(),fixture.binding.index_sha256);
  EXPECT_EQ(amplitude["raw_provenance_sha256"].GetString(),fixture.binding.provenance_sha256);
  EXPECT_EQ(amplitude["context_sha256"].GetString(),io::Sha256(context));
  EXPECT_EQ(amplitude["raw_native_matrix"]["sha256"].GetString(),io::Sha256(io::ReadBounded(fixture.directory.path/"raw/native-h0-a1.json",RawFileByteCap)));
  const auto& branch=amplitude["analysis"]["branches"]["constant_branches"][0];
  EXPECT_FALSE(branch["full_operator"].HasMember("values_rows"));
  EXPECT_EQ(branch["full_operator"]["sha256"].GetString(),WallDerivedMatrixHash(job.steps[0].amplitudes[1].branches.branches[0].full));
  EXPECT_EQ(branch["raw_spectrum"]["matrix_norm"]["nonfinite"].GetString(),std::string("positive-infinity"));
  const auto& actual=amplitude["analysis"]["branches"]["constant_branches"][1];
  EXPECT_EQ(actual["raw_spectrum"]["eigenvalues_real_imaginary"].Size(),109u);
  EXPECT_EQ(actual["continuous_gram"]["raw"]["eigenvalues"].Size(),109u);
  EXPECT_EQ(actual["continuous_gram"]["raw"]["controlling_direction"].Size(),109u);
  for(unsigned i=0;i<109;++i) {
    EXPECT_EQ(actual["raw_spectrum"]["eigenvalues_real_imaginary"][i][0].GetDouble(),populated.spectrum.eigenvalues[i].real());
    EXPECT_EQ(actual["raw_spectrum"]["eigenvalues_real_imaginary"][i][1].GetDouble(),populated.spectrum.eigenvalues[i].imag());
    EXPECT_EQ(actual["continuous_gram"]["raw"]["eigenvalues"][i].GetDouble(),gram.eigenvalues[i]);
    EXPECT_EQ(actual["continuous_gram"]["raw"]["controlling_direction"][i].GetDouble(),gram.controlling_direction[i]);
  }
  // Independent sparse B construction and explicit row/column similarity.
  const auto& model=fixture.raw.job.model; const auto& m=model.native(); Eigen::MatrixXd kick=Eigen::MatrixXd::Identity(109,109);
  for(unsigned n=0;n<m.nodes;++n) {
    const auto x=model.coordinate(recurrence::Group::Position,n,0),v=model.coordinate(recurrence::Group::Velocity,n,0);
    kick(v,x)=-static_cast<double>(static_cast<long double>(job.steps[0].h)*model.touching().nodes[n].stiffness.value/m.mass[n]*m.dictionary[x].scale/m.dictionary[v].scale);
  }
  const Eigen::MatrixXd full=(fixture.raw.job.steps[0].native[1].derivative.full*kick).eval(); Eigen::MatrixXd weighted(109,109);
  const auto& diagonal=job.steps[0].amplitudes[1].branches.metric.diagonal;
  for(unsigned r=0;r<109;++r) for(unsigned c=0;c<109;++c) weighted(r,c)=diagonal[r]*full(r,c)/diagonal[c];
  EXPECT_EQ(actual["full_operator"]["sha256"].GetString(),WallDerivedMatrixHash(full));
  EXPECT_EQ(actual["weighted_operator"]["sha256"].GetString(),WallDerivedMatrixHash(weighted));
  writer(job,{WallJobProgressKind::Finished});
  EXPECT_TRUE(writer.receipt().final_index_present); EXPECT_FALSE(writer.receipt().analysis_complete); EXPECT_FALSE(writer.receipt().analysis_passed);
  test::CheckFiles(path,writer.receipt());
}
TEST(QephWallJobReport, LateSerializationAndContextMismatchPreserveEarlierEvidence) {
  test::Fixture fixture; auto job=fixture.Job(); fixture.Prepare(job,1); fixture.Prepare(job,2);
  const auto path=fixture.directory.path/"derived";
  WallJobReportWriter writer(path,fixture.raw,fixture.binding,test::rt::Provenance(),"synthetic-analysis.json",ScreenSetByteCap);
  writer(job,{WallJobProgressKind::Amplitude,0,0}); writer(job,{WallJobProgressKind::Amplitude,0,1});
  const auto held=writer.receipt().total_bytes; auto& a=job.steps[0].amplitudes[2];
  a.branches.branches[0].spectrum.complete=true;
  a.branches.branches[0].spectrum.eigenvalues.resize(109,{1,0});
  EXPECT_THROW(writer(job,{WallJobProgressKind::Amplitude,0,2}),std::runtime_error); // Completed nonfinite norm.
  EXPECT_EQ(writer.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(path/"amplitude-h0-a2.json"));
  a.branches.branches[0].spectrum.complete=false; a.branches.metric.eta*=1.01;
  EXPECT_THROW(writer(job,{WallJobProgressKind::Amplitude,0,2}),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(path/"index.json"));
  test::CheckFiles(path,writer.receipt());
}
TEST(QephWallJobReport, IdentityBudgetAndCreateOnlyFailuresDoNotPublishFinalIndex) {
  test::Fixture fixture; auto job=fixture.Job(); fixture.Prepare(job,1); fixture.Prepare(job,2);
  const auto invalid=fixture.directory.path/"invalid"; auto wrong=fixture.binding; wrong.index_sha256=std::string(64,'0');
  EXPECT_THROW(WallJobReportWriter(invalid,fixture.raw,wrong,test::rt::Provenance(),"synthetic.json",ScreenSetByteCap),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(invalid));
  const auto path=fixture.directory.path/"full";
  WallJobReportWriter full(path,fixture.raw,fixture.binding,test::rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  full(job,{WallJobProgressKind::Amplitude,0,0}); full(job,{WallJobProgressKind::Amplitude,0,1});
  const auto held=full.receipt().total_bytes; job.normal_velocity=8;
  EXPECT_THROW(full(job,{WallJobProgressKind::Amplitude,0,2}),std::runtime_error);
  job.normal_velocity=0; EXPECT_EQ(full.receipt().total_bytes,held);
  const auto bounded=fixture.directory.path/"bounded";
  WallJobReportWriter limited(bounded,fixture.raw,fixture.binding,test::rt::Provenance(),"synthetic.json",held+256);
  limited(job,{WallJobProgressKind::Amplitude,0,0}); limited(job,{WallJobProgressKind::Amplitude,0,1});
  const auto limited_held=limited.receipt().total_bytes;
  EXPECT_THROW(limited(job,{WallJobProgressKind::Amplitude,0,2}),std::runtime_error);
  EXPECT_EQ(limited.receipt().total_bytes,limited_held); EXPECT_FALSE(std::filesystem::exists(bounded/"index.json"));
  io::WriteBytes(path/"amplitude-h0-a2.json","retained conflict");
  EXPECT_THROW(full(job,{WallJobProgressKind::Amplitude,0,2}),std::runtime_error);
  EXPECT_EQ(io::ReadBounded(path/"amplitude-h0-a2.json",100),"retained conflict");
  EXPECT_EQ(full.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(path/"index.json"));
}
TEST(QephWallJobReport, ContactStepLinksAndFalseFinalFlagsAreExplicit) {
  test::Fixture fixture; auto job=fixture.Job(); const auto path=fixture.directory.path/"derived";
  fixture.Prepare(job,1); fixture.Prepare(job,2);
  WallJobReportWriter writer(path,fixture.raw,fixture.binding,test::rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  for(unsigned a=0;a<3;++a) writer(job,{WallJobProgressKind::Amplitude,0,a});
  auto& contact=job.steps[0].contact[1]; WallDirectionRecheck direction;
  direction.name="synthetic failed direction"; direction.quotients[0]=Eigen::VectorXd::Constant(109,std::numeric_limits<double>::quiet_NaN());
  contact.directions.push_back(direction);
  writer(job,{WallJobProgressKind::Contact,0,0}); writer(job,{WallJobProgressKind::Contact,0,1});
  job.steps[0].complete=true; job.steps[0].passed=true; job.completed_steps=1;
  const auto held=writer.receipt().total_bytes;
  EXPECT_THROW(writer(job,{WallJobProgressKind::Step,0}),std::runtime_error);
  EXPECT_EQ(writer.receipt().total_bytes,held); EXPECT_FALSE(std::filesystem::exists(path/"step-h0.json"));
  job.steps[0].complete=false; job.steps[0].passed=false; job.completed_steps=0;
  writer(job,{WallJobProgressKind::Step,0});
  const auto step=test::rt::Read(path/"step-h0.json"); EXPECT_EQ(step["derived_amplitudes"].Size(),3u); EXPECT_EQ(step["derived_contact"].Size(),2u);
  const auto checked=test::rt::Read(path/"contact-h0-b1.json");
  EXPECT_EQ(checked["analysis"]["directions"][0]["quotients"][0][0]["nonfinite"].GetString(),std::string("nan"));
  EXPECT_FALSE(checked["raw_contact_branch"]["present"].GetBool());
  job.complete=true; job.passed=true;
  EXPECT_THROW(writer(job,{WallJobProgressKind::Finished}),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path/"index.json")); job.complete=false; job.passed=false;
  writer(job,{WallJobProgressKind::Finished}); test::CheckFiles(path,writer.receipt());
  const auto final=test::rt::Read(path/"index.json"); EXPECT_FALSE(final["analysis_passed"].GetBool());
  EXPECT_FALSE(final["six_job_screen_decision_included"].GetBool());
}
} // namespace tl::qualification::qeph::wall_recurrence
