// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/radioss_type25/GeometryTypes.h"
#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
namespace native_runtime_test {
struct ExpectedPacket {std::vector<int> secondary,main;};
struct ExpectedFrame {
  std::uint64_t epoch=0;double time=0,drift=0,kick=0;
  std::array<double,54> position{},velocity{},incoming_force{},outgoing_force{};
  std::array<double,18> incoming_stiffness{},outgoing_stiffness{};
  std::array<tlfea::contact::radioss_type25::NativeContactRow,18> rows;
  std::array<int,18> initial_contact{};
  std::vector<ExpectedPacket> geometry_packets;
};
// Expected observations only. This reader exposes no source or physical restart
// API. The format explicitly writes scalar little-endian words, never C++ ABI
// padding/pointers or a binary memcpy of an owner object.
class ReferenceReader {
  std::ifstream input;
  template<class T> void Bits(T& value) {
    unsigned char bytes[sizeof(T)];input.read(reinterpret_cast<char*>(bytes),sizeof(bytes));
    if(!input)throw std::runtime_error("Truncated native expected fixture");
    std::uint64_t bits=0;for(unsigned i=0;i<sizeof(T);++i)bits|=std::uint64_t(bytes[i])<<(8*i);
    if constexpr(sizeof(T)==8)std::memcpy(&value,&bits,8);
    else {const auto low=std::uint32_t(bits);std::memcpy(&value,&low,4);}
  }
  template<std::size_t N> void Doubles(std::array<double,N>& values){for(auto& value:values)Bits(value);}
 public:
  explicit ReferenceReader(const std::string& path):input(path,std::ios::binary) {
    static_assert(sizeof(double)==8&&std::numeric_limits<double>::is_iec559&&sizeof(int)==4);
    char magic[8];input.read(magic,8);if(!input||std::memcmp(magic,"T25REF01",8))throw std::runtime_error("Bad native expected fixture magic");
    std::uint32_t nodes=0,rows=0,count=0;Bits(nodes);Bits(rows);Bits(count);
    if(nodes!=18||rows!=18||count!=1001)throw std::runtime_error("Native expected fixture shape changed");
  }
  ExpectedFrame Read(std::uint64_t expected_epoch) {
    ExpectedFrame f;Bits(f.epoch);if(f.epoch!=expected_epoch)throw std::runtime_error("Native expected epoch is not sequential");
    Bits(f.time);Bits(f.drift);Bits(f.kick);Doubles(f.position);Doubles(f.velocity);
    Doubles(f.incoming_force);Doubles(f.outgoing_force);Doubles(f.incoming_stiffness);Doubles(f.outgoing_stiffness);
    for(unsigned i=0;i<18;++i){auto& row=f.rows[i];for(auto& code:row.irtlm)Bits(code);auto& h=row.history.normal;
      Bits(h.previous_penetration);Bits(h.previous_stiffness);Bits(h.staged_penetration);Bits(h.staged_stiffness);Bits(h.damping_half_force);
      Bits(row.history.previous_force.x);Bits(row.history.previous_force.y);Bits(row.history.previous_force.z);
      Bits(row.history.staged_force.x);Bits(row.history.staged_force.y);Bits(row.history.staged_force.z);
      Bits(row.penetration_auxiliary);Bits(row.penetration_offset);Bits(row.selection_metric[0]);Bits(row.selection_metric[1]);Bits(f.initial_contact[i]);}
    std::uint32_t count=0;Bits(count);if(count>16)throw std::runtime_error("Native packet bound changed");f.geometry_packets.resize(count);
    for(auto& packet:f.geometry_packets){std::uint32_t n=0;Bits(n);if(!n||n>128)throw std::runtime_error("Native packet width changed");
      packet.secondary.resize(n);packet.main.resize(n);for(auto& v:packet.secondary)Bits(v);for(auto& v:packet.main)Bits(v);}
    return f;
  }
  void Finish(){if(input.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing native expected fixture bytes");}
};
} // namespace native_runtime_test
