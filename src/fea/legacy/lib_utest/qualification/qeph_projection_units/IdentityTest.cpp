// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/elements/qeph/QephStartup.h"
#include "lib_src/elements/qeph/QephForce.h"
#include "lib_src/elements/qbat/QbatReference.h"
#include "lib_src/elements/ShellBatchBinding.h"
#include <gtest/gtest.h>
#include <array>
#include <cstring>
#include <limits>
namespace {
namespace fe=tl::fea;namespace q=fe::qeph;
template<class T> auto Bytes(const T& object){std::array<unsigned char,sizeof(T)> b;std::memcpy(b.data(),&object,sizeof object);return b;}
q::ReferenceInput Input(){q::ReferenceInput x;x.position[0]={0,0,0};x.position[1]={.02,0,0};x.position[2]={.02,.01,0};x.position[3]={0,.01,0};return x;}
TEST(QephProjectionIdentity,ReferenceAndHistoryAuthenticateMetricWithoutChangingMass) {
  auto input=Input();q::ReferenceData a,b;ASSERT_EQ(q::InitializeReference(input,a),q::Status::kSuccess);
  input.projection_working_length_m=.001;ASSERT_EQ(q::InitializeReference(input,b),q::Status::kSuccess);
  q::History h;ASSERT_EQ(q::InitializeHistory(a,{0,0},h),q::Status::kSuccess);
  EXPECT_TRUE(h.matches_reference(a));EXPECT_FALSE(h.matches_reference(b));
  EXPECT_EQ(a.area,b.area);EXPECT_EQ(Bytes(a.frame),Bytes(b.frame));
  for(unsigned n=0;n<4;++n){EXPECT_EQ(a.nodal_mass[n],b.nodal_mass[n]);EXPECT_EQ(a.isotropic_inertia[n],b.isotropic_inertia[n]);}
  q::History native;ASSERT_EQ(q::InitializeHistory(b,{0,0},native),q::Status::kSuccess);
  EXPECT_TRUE(native.matches_reference(b));EXPECT_FALSE(native.matches_reference(a));
  q::PrescribedInterval interval;interval.dt=1e-7;interval.sample_index=1;
  for(unsigned n=0;n<4;++n)interval.position_endpoint[n]=a.input.position[n];
  q::ForceTrial trial;const auto old=Bytes(trial);
  EXPECT_EQ(q::EvaluateForce(b,h,interval,trial),q::Status::kInvalidReference);EXPECT_EQ(Bytes(trial),old);
  EXPECT_EQ(q::EvaluateForce(a,native,interval,trial),q::Status::kInvalidReference);EXPECT_EQ(Bytes(trial),old);
}
TEST(QephProjectionIdentity,InvalidScaleRejectsBeforeReferenceOrHistoryPublication) {
  const double invalid[]{0,-1,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN(),1e-300,1e300};
  q::ReferenceData accepted;ASSERT_EQ(q::InitializeReference(Input(),accepted),q::Status::kSuccess);const auto before=Bytes(accepted);
  q::History history;ASSERT_EQ(q::InitializeHistory(accepted,{0,0},history),q::Status::kSuccess);const auto old=Bytes(history);
  for(double scale:invalid){auto in=Input();in.projection_working_length_m=scale;
    EXPECT_EQ(q::InitializeReference(in,accepted),q::Status::kInvalidInput);EXPECT_EQ(Bytes(accepted),before);
    auto bad=accepted;bad.input.projection_working_length_m=scale;
    EXPECT_EQ(q::InitializeHistory(bad,{0,0},history),q::Status::kInvalidReference);EXPECT_EQ(Bytes(history),old);}
}
TEST(QephProjectionIdentity,CompleteCollectionInventoryDistinguishesMetricAndRetainsOwnedCopy) {
  fe::ShellQephBindingInput first{Input(),{0,1,2,3},21};fe::ShellBatchCollectionInput source{&first,nullptr,1,0,4};
  fe::ShellBatchBinding a,b;ASSERT_EQ(a.Initialize(source).status,fe::ShellBindingStatus::Success);
  first.reference.projection_working_length_m=.001;ASSERT_EQ(b.Initialize(source).status,fe::ShellBindingStatus::Success);
  EXPECT_NE(a.inventory(),b.inventory());ASSERT_EQ(b.inventory().words().size(),33u);EXPECT_EQ(b.inventory().words()[0],7u);
  std::uint64_t bits;std::memcpy(&bits,&first.reference.projection_working_length_m,sizeof bits);EXPECT_EQ(b.inventory().words()[32],bits);
  first.reference.projection_working_length_m=7;EXPECT_EQ(b.qeph_reference(0).input.projection_working_length_m,.001);
  EXPECT_EQ(a.totals().mass,b.totals().mass);EXPECT_EQ(a.totals().isotropic_inertia,b.totals().isotropic_inertia);
  fe::ShellBatchBinding copy(b);EXPECT_EQ(copy.inventory(),b.inventory());EXPECT_EQ(copy.qeph_reference(0).input.projection_working_length_m,.001);
}
TEST(QephProjectionIdentity,QbatExplicitlyRejectsUnusedNondefaultQephMetric) {
  fe::qbat::ReferenceInput in;in.quadrilateral=Input();in.initial_a11_pa=in.quadrilateral.young_modulus/(1-in.quadrilateral.poisson_ratio*in.quadrilateral.poisson_ratio);
  fe::qbat::Reference reference;ASSERT_EQ(fe::qbat::InitializeReference(in,reference),q::Status::kSuccess);const auto before=Bytes(reference);
  in.quadrilateral.projection_working_length_m=.001;
  EXPECT_EQ(fe::qbat::InitializeReference(in,reference),q::Status::kInvalidInput);EXPECT_EQ(Bytes(reference),before);
}
} // namespace
