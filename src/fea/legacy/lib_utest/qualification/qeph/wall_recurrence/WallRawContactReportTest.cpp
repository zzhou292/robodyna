#include "WallRawReportTestFixture.h"
#include <limits>
#include <utility>

namespace tl::qualification::qeph::wall_recurrence {
namespace rt=raw_test;
namespace io=crash::output;
TEST(QephWallRawReport, ActualContactCertificatesAndFailedQuotientMarkersRemainSeparate) {
  rt::Directory directory; const auto path=directory.path/"raw"; auto job=rt::Fixture();
  RawJobWriter writer(path,1,0,rt::Provenance(),"synthetic.json",ScreenSetByteCap);
  writer(job,{RawProgressKind::Model});
  // The synthetic identity matrix is explicitly not an admitted native matrix.
  // It provides only a complete serialization prerequisite in this partial job.
  rt::Native(job,0,2,true); writer(job,{RawProgressKind::NativeMatrix,0,2});
  auto& p=job.steps[0].contact[1]; job.steps[0].contact_attempted[1]=true;
  const auto dimension=job.model.native().dictionary.size(); const double h=job.steps[0].h;
  p.h=h; p.normal_velocity=0; p.branch=ContactBranch::Active; p.full=Eigen::MatrixXd::Identity(dimension,dimension);
  std::string error;
  ASSERT_TRUE(EvaluateContactNativeMap(job.model,h,0,Eigen::VectorXd::Zero(dimension),p.baseline,error))<<error;
  p.baseline_complete=true; std::vector<ContactDirection> directions;
  ASSERT_TRUE(SignConeDirections(job.model,ContactBranch::Active,directions,error));
  ContactDirectionProbe d; d.direction=directions.front();
  for(unsigned a=0;a<3;++a) {
    ASSERT_TRUE(EvaluateContactNativeMap(job.model,h,0,recurrence::Amplitudes[a]*d.direction.value,d.samples[a],error))<<error;
    ++d.completed_samples;
  }
  d.quotients[0]=Eigen::VectorXd::Constant(dimension,std::numeric_limits<double>::infinity());
  d.diagnostic="Synthetic failed quotient after three actual physical samples";
  p.directions.push_back(std::move(d)); p.diagnostic="Synthetic incomplete direction collection";
  writer(job,{RawProgressKind::ContactBranch,0,1});
  const auto report=rt::Read(path/"contact-h0-b1.json"); const auto& direction=report["probe"]["directions"][0];
  EXPECT_EQ(report["native_matrix_file"].GetString(),std::string("native-h0-a2.json"));
  EXPECT_EQ(report["native_matrix_sha256"].GetString(),io::Sha256(io::ReadBounded(path/"native-h0-a2.json",RawFileByteCap)));
  EXPECT_FALSE(report["probe"]["local_derivative_checks_complete"].GetBool());
  EXPECT_EQ(direction["one_sided_quotients"][0][0]["nonfinite"].GetString(),std::string("positive-infinity"));
  const auto& sample=direction["samples"][0]; const auto& actual=p.directions[0].samples[0];
  EXPECT_EQ(sample["nodes"].Size(),4u); EXPECT_EQ(sample["parents"].Size(),1u);
  for(unsigned n=0;n<4;++n) {
    const auto& node=sample["nodes"][n]; const auto& point=actual.nodes[n];
    EXPECT_EQ(node["force_n"]["value"].GetDouble(),point.force.value);
    EXPECT_EQ(node["force_n"]["lower"].GetDouble(),point.force.lower);
    EXPECT_EQ(node["force_n"]["upper"].GetDouble(),point.force.upper);
    EXPECT_EQ(node["force_n"]["error"].GetDouble(),point.force.error);
    EXPECT_EQ(node["source_row_diagnostic"]["attempt"].GetUint64(),point.row.attempt);
  }
  EXPECT_EQ(sample["potential_j"]["upper"].GetDouble(),actual.potential.upper);
  const auto bytes=io::ReadBounded(path/"contact-h0-b1.json",RawFileByteCap);
  EXPECT_THROW(writer(job,{RawProgressKind::ContactBranch,0,1}),std::runtime_error);
  EXPECT_EQ(io::ReadBounded(path/"contact-h0-b1.json",RawFileByteCap),bytes);
  job.collection_complete=true;
  EXPECT_THROW(writer(job,{RawProgressKind::Finished}),std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path/"index.json")); job.collection_complete=false;
  writer(job,{RawProgressKind::Finished}); rt::CheckFiles(path,writer.receipt());
  EXPECT_FALSE(writer.receipt().collection_complete);
}
} // namespace tl::qualification::qeph::wall_recurrence
