#include "Oracle.h"
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
namespace {
std::uint64_t Bits(double x) {
  std::uint64_t bits;
  std::memcpy(&bits,&x,sizeof(bits));
  return bits;
}
void Hex(double x) {
  std::cout << '"' << std::hex << std::setw(16) << std::setfill('0') << Bits(x)
      << std::dec << '"';
}
bool Near(double a,double b) {
  return std::isfinite(a) && std::isfinite(b) && std::abs(a-b) <=
      64*std::numeric_limits<double>::epsilon()*std::max({std::abs(a),std::abs(b),1e-300});
}
}
int main() try {
  const auto cases=pen3_test::Cases();
  if(cases.size()!=2064) throw std::runtime_error("Unexpected fixed corpus size");
  std::size_t changed_bits=0,changed_membership=0,outside_tolerance=0,nonfinite=0;
  bool packing_controls=true;
  std::cout << std::setprecision(17);
  std::cout << "{\"kind\":\"header\",\"schema\":\"pen3_cohort_probe_v1\","
      "\"modes\":[\"single_t3\",\"two_t3\",\"t3_then_q4\",\"q4_then_t3\"],"
      "\"output_order\":\"target_then_auxiliary\",\"native_nrtm\":2,"
      "\"active_rows\":[1,2,2,2],\"single_t3_second_output\":\"unused_wrapper_initialized_zero\","
      "\"controls\":{\"all_t3\":\"cohort_length_target_first\",\"mixed\":\"target_ordinal\"},"
      "\"native_has_status\":false,\"auxiliary_inputs\":[";
  for (unsigned kind=0;kind<2;++kind) {
    if(kind) std::cout << ',';
    const auto other=pen3_test::Unrelated(kind==0);
    std::cout << "{\"coordinates_bits\":[";
    for(unsigned j=0;j<15;++j) {if(j)std::cout<<',';Hex(other.coordinates[j]);}
    std::cout << "],\"flags\":[";
    for(unsigned j=0;j<6;++j) {if(j)std::cout<<',';std::cout<<other.flags[j];}
    std::cout << "],\"gap_bits\":";Hex(other.gap);std::cout<<'}';
  }
  std::cout << "]}\n";
  for(std::size_t i=0;i<cases.size();++i) {
    const auto& test=cases[i];
    std::array<std::array<double,2>,4> outputs;
    for(unsigned mode=0;mode<4;++mode) outputs[mode]=pen3_test::Evaluate(test,mode);
    const double a=outputs[0][0],b=outputs[2][0];
    bool finite=true;
    for (const auto& row : outputs) for (double value : row) finite &= std::isfinite(value);
    const bool same_bits=Bits(a)==Bits(b);
    const bool same_membership=finite&&((a!=0)==(b!=0));
    const bool near=Near(a,b);
    nonfinite+=!finite;
    changed_bits+=!same_bits;
    changed_membership+=finite&&!same_membership;
    outside_tolerance+=finite&&!near;
    // All-T3 compares cohort length with target first in both calls. Mixed
    // compares target ordinal. Failures flag packing/undefined-state concerns.
    packing_controls &= Bits(a)==Bits(outputs[1][0]) && Bits(b)==Bits(outputs[3][0]);
    std::cout << "{\"kind\":\"case\",\"index\":" << i
        << ",\"family\":\"" << test.family << "\",\"coordinates_bits\":[";
    for(unsigned k=0;k<15;++k) {if(k)std::cout<<',';Hex(test.target.coordinates[k]);}
    std::cout << "],\"flags\":[";
    for(unsigned k=0;k<6;++k) {if(k)std::cout<<',';std::cout<<test.target.flags[k];}
    std::cout << "],\"gap_bits\":";Hex(test.target.gap);
    std::cout << ",\"margin_bits\":";Hex(test.margin);
    std::cout << ",\"outputs_bits\":[";
    for(unsigned mode=0;mode<4;++mode) {
      if(mode)std::cout<<',';
      std::cout<<'[';Hex(outputs[mode][0]);std::cout<<',';Hex(outputs[mode][1]);std::cout<<']';
    }
    std::cout << "],\"finite\":" << (finite?"true":"false")
        << ",\"same_target_bits\":" << (same_bits?"true":"false")
        << ",\"same_membership\":" << (same_membership?"true":"false")
        << ",\"within_64eps\":" << (near?"true":"false") << "}\n";
  }
  std::cout << "{\"kind\":\"summary\",\"cases\":" << cases.size()
      << ",\"native_calls\":" << 4*cases.size() << ",\"changed_bits\":" << changed_bits
      << ",\"changed_membership\":" << changed_membership
      << ",\"outside_64eps\":" << outside_tolerance << ",\"nonfinite_pairs\":" << nonfinite
      << ",\"packing_controls_exact\":" << (packing_controls?"true":"false")
      << ",\"comparison_complete\":true,\"physical_qualification\":false}\n";
  // Differences are the measurement. They are not suppressed or turned into a
  // fake solver failure. Nonfinite data or failed invariant controls are errors.
  return nonfinite||!packing_controls?2:0;
} catch(const std::exception& e) {
  std::cerr << e.what() << '\n';
  return 2;
}
