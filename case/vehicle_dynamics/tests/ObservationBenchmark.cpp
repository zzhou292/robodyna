#include "ObservationFixture.h"
#include "ObservationTransfer.h"
#include <charconv>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <locale>
#include <string_view>
#include <vector>
namespace {
std::size_t Count(const char* text) {
  const std::string_view value(text);std::size_t out=0;
  const auto result=std::from_chars(value.data(),value.data()+value.size(),out);
  if(result.ec!=std::errc{}||result.ptr!=value.data()+value.size()||!out)
    throw std::runtime_error("Invalid observation benchmark count");
  return out;
}
struct Sample {std::size_t repetition,order;bool old;double seconds;motion_transfer_probe::Counts transfers;};
}
int main(int argc,char** argv) {
  try {
    if(argc!=5||std::string_view(argv[1])!="--nodes"||std::string_view(argv[3])!="--repetitions")
      throw std::runtime_error("Usage: observation_benchmark --nodes N --repetitions R");
    const auto nodes=Count(argv[2]),repetitions=Count(argv[4]);
    if(nodes>tl::fea::MaxActiveNodalStateNodes||repetitions>64)throw std::runtime_error("Observation benchmark cap exceeded");
    using namespace crash::cases::vehicle_dynamics::motion_test;
    Fixture fixture(nodes);
    const auto expected=fixture.Old();
    for(unsigned i=0;i<4;++i)if(!Same(fixture.New(),expected)||!Same(fixture.Old(),expected))
      throw std::runtime_error("Observation warmup mismatch");
    std::vector<Sample> samples;samples.reserve(4*repetitions);
    double old_seconds=0,new_seconds=0;
    for(std::size_t repetition=0;repetition<repetitions;++repetition) {
      for(std::size_t order=0;order<4;++order) {
        const bool old=(order==0||order==3) != bool(repetition&1);
        motion_transfer_probe::Begin();
        const auto begin=std::chrono::steady_clock::now();
        const auto actual=old?fixture.Old():fixture.New();
        const auto end=std::chrono::steady_clock::now();
        const auto transfers=motion_transfer_probe::End();
        if(!Same(actual,expected))throw std::runtime_error("Timed observation changed result");
        const auto seconds=std::chrono::duration<double>(end-begin).count();
        (old?old_seconds:new_seconds)+=seconds;
        samples.push_back({repetition,order,old,seconds,transfers});
      }
    }
    std::cout.imbue(std::locale::classic());std::cout<<std::setprecision(17);
    std::cout<<"{\"schema\":\"robo_dyna.complete_motion_observation_benchmark.v1\","
      "\"scope\":\"component_only_not_solver_speedup\",\"nodes\":"<<nodes
      <<",\"accepted_epoch\":"<<fixture.owner.accepted().epoch
      <<",\"prepared_time_s\":"<<fixture.prepared.proposed_time
      <<",\"reference_device_bytes\":"<<fixture.observer.allocations().device_bytes
      <<",\"old_nodal_transfer_bytes\":"<<19*nodes*sizeof(double)
      <<",\"old_app_payload_bytes\":"<<13*nodes*sizeof(double)
      <<",\"samples\":[";
    bool first=true;
    for(const auto& row:samples) {
      if(!first)std::cout<<',';first=false;
      std::cout<<"{\"repetition\":"<<row.repetition<<",\"order\":"<<row.order
        <<",\"backend\":\""<<(row.old?"full_readback_cpu_scan":"device_complete_observation")
        <<"\",\"completed_seconds\":"<<row.seconds
        <<",\"host_read_bytes\":"<<row.transfers.host_bytes
        <<",\"host_reads\":"<<row.transfers.host_reads
        <<",\"device_allocations\":"<<row.transfers.device_allocations<<'}';
    }
    std::cout<<"],\"old_mean_seconds\":"<<old_seconds/(2*repetitions)
      <<",\"new_mean_seconds\":"<<new_seconds/(2*repetitions)
      <<",\"component_ratio\":"<<old_seconds/new_seconds<<",\"summary_bits_match\":true}\n";
    return 0;
  } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
