#include "CudaFixture.h"
#include "Fingerprint.h"
#include "../extended_solid_resident/CudaFixture.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iomanip>
namespace solid_parallel_test {
namespace e=extended_resident_test;
std::size_t Setting(const char* key,std::size_t fallback,std::size_t cap) {
  const auto* text=std::getenv(key);
  if (!text) return fallback;
  char* end=nullptr;const auto value=std::strtoull(text,&end,10);
  if (!*text || *end || !value || value>cap) throw std::runtime_error("Benchmark option");
  return value;
}
template<class Traits,class Rows>
void CopyCaches(Packet& packet,const Rows& rows) {
  auto& family=d::FamilyStorage<Traits>(packet.State());
  if (rows.empty()) throw std::runtime_error("Missing completed cache family");
  for (std::size_t i=0;i<family.count;++i) family.slab[0][i].cache=rows[i%rows.size()].cache;
}
TEST(SolidParallelAssemblyBenchmark, CompletedFiveFamilyCachesWithFullAssemblyReadControlAndParity) {
  const auto repeats=Setting("SOLID_ASSEMBLY_REPEATS",100,10000);
  const auto parents=Setting("SOLID_ASSEMBLY_PARENTS_PER_FAMILY",64,1000);
  const auto nodes=Setting("SOLID_ASSEMBLY_NODES",4096,524288);
  ASSERT_GE(nodes,8u);
  const std::string mode=std::getenv("SOLID_ASSEMBLY_MODE")?std::getenv("SOLID_ASSEMBLY_MODE"):"current";
  ASSERT_TRUE(mode=="serial" || mode=="current");
  e::Rig owning;
  ASSERT_TRUE(owning.Initialize());
  for (unsigned step=0;step<8;++step) {
    fe::NodalTrialToken token;fe::NodalAssemblyView assembly;fe::NodalPreparedView prepared;
    ASSERT_TRUE(owning.Begin(token,assembly));ASSERT_TRUE(owning.Prepare(token,assembly,prepared));
    s::BatchDiagnostics candidate;
    ASSERT_TRUE(e::Good(owning.batch.EvaluateCandidate(owning.owner,token,prepared,&candidate)));
    ASSERT_TRUE(e::Good(e::Peer::Commit(owning.batch,owning.owner,token,prepared,candidate)));
  }
  e::Results completed(owning.fixture.model);s::BatchDiagnostics diagnostics;
  ASSERT_TRUE(owning.Read(completed,diagnostics));ASSERT_EQ(diagnostics.epoch,8u);
  Packet packet(nodes,parents);
  CopyCaches<d::Traits18>(packet,completed.old18);CopyCaches<d::Traits24>(packet,completed.old24);
  CopyCaches<d::Traits6z>(packet,completed.old6z);CopyCaches<d::Traits18Law44>(packet,completed.rear);
  CopyCaches<d::Traits18Law90>(packet,completed.foam);
  double nonzero=0;
  Fingerprint inputs;
  inputs.Integer(nodes);inputs.Integer(parents);
  for (std::size_t i=0;i<packet.State().assembly.occurrences;++i) {
    d::AssemblyOccurrence value;ASSERT_TRUE(d::ReadAssemblyOccurrence<true>(packet.State(),0,i,value));
    inputs.Integer(value.node);inputs.Real(value.force.x);inputs.Real(value.force.y);inputs.Real(value.force.z);inputs.Real(value.stiffness);
    nonzero=std::max(nonzero,std::abs(value.force.x)+std::abs(value.force.y)+std::abs(value.force.z));
  }
  ASSERT_GT(nonzero,0.);
  DevicePacket device(packet);
  Cuda(device.Launch(true));const auto expected=device.Read();
  ASSERT_EQ(expected.control.status,s::BatchStatus::Success);
  Fingerprint outputs;
  for (double value:expected.fields) outputs.Real(value);
  std::uint64_t total_ns=0;
  for (std::size_t i=0;i<repeats+2;++i) {
    device.Reset();
    d::Control control;
    const StreamDrain drain{device.stream};
    const auto start=std::chrono::steady_clock::now();
    Cuda(device.Launch(mode=="serial"));
    Cuda(cudaGetLastError());
    Cuda(cudaMemcpyAsync(&control,&device.storage->control,sizeof(control),cudaMemcpyDeviceToHost,device.stream));
    Cuda(cudaStreamSynchronize(device.stream));
    const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now()-start).count();
    if (i>=2) total_ns+=elapsed;
    const auto actual=device.Read();Same(expected,actual);ASSERT_FALSE(HasFailure());
    if (mode=="current") ASSERT_EQ(actual.fallback,0u);
  }
  std::cout<<std::setprecision(17)<<"{\"scope\":\"synthetic topology; actual completed owner caches; full assembly plus control readback\","
      <<"\"mode\":\""<<mode<<"\",\"repeats\":"<<repeats<<",\"parents\":"<<5*parents
      <<",\"nodes\":"<<nodes<<",\"occurrences\":"<<packet.State().assembly.occurrences
      <<",\"seconds_per_call\":"<<double(total_ns)/1e9/repeats
      <<",\"input_digest\":"<<inputs.value<<",\"field_bits_digest\":"<<outputs.value
      <<",\"full_report_and_fields_verified_each_call\":true,\"max_cache_force_l1\":"<<nonzero<<",\"device_bytes\":"<<packet.layout.bytes<<"}"<<std::endl;
}
} // namespace solid_parallel_test
