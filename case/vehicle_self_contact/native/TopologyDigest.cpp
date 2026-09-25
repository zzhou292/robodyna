#include "TopologyAssessmentInternal.h"
#include <algorithm>
#include <cstring>
#include <limits>
namespace crash::cases::vehicle_self_contact::native::detail {
namespace {
constexpr std::size_t ChunkWords=65536;
std::string Integers(std::initializer_list<std::uint64_t> words) {
  return output::arrays::Encode<std::uint64_t>({output::arrays::Scalar::UInt64,words.size(),1,{}},words.begin(),words.size());
}
std::uint32_t FloatBits(float value) {
  static_assert(sizeof(float)==sizeof(std::uint32_t));
  std::uint32_t bits;std::memcpy(&bits,&value,sizeof(bits));return bits;
}
class Fields {
 public:
  Fields(const std::string& binding,std::size_t cap):cap_(cap) {
    output::Require(cap_&&cap_<=1u<<20,"Invalid topology digest metadata cap");
    Append(root_,"robo_dyna.native_topology_fields.v1");
    Append(root_,Integers({binding.size()}));Append(root_,binding);
  }
  template<class T,class Read> void Add(const char* name,std::size_t rows,std::size_t columns,Read read) {
    output::Require(columns && rows<=SIZE_MAX/columns,"Topology field digest extent overflow");
    const auto count=rows*columns;
    const auto chunks=count/ChunkWords+(count%ChunkWords!=0);
    const auto type=output::arrays::detail::Type<T>::value;
    const std::string spelling=output::arrays::Dtype(type);
    std::string chain;
    Append(chain,"robo_dyna.native_topology_field.v1");
    Append(chain,Integers({std::strlen(name),spelling.size(),rows,columns,ChunkWords,chunks}));
    Append(chain,name);Append(chain,spelling);
    output::Require(chunks<=(cap_-chain.size())/64,"Topology chunk digest list exceeds cap");
    std::vector<T> values;values.reserve(std::min(count,ChunkWords));
    for(std::size_t first=0;first<count;) {
      const auto size=std::min(ChunkWords,count-first);values.resize(size);
      for(std::size_t i=0;i<size;++i)values[i]=read(first+i);
      const auto encoded=output::arrays::Encode<T>({type,size,1,{}},values.data(),size);
      Append(chain,output::Sha256(encoded));first+=size;
    }
    FieldDigest field{name,spelling,output::Sha256(chain),rows,columns,chunks};
    Append(root_,Integers({std::strlen(name),spelling.size(),rows,columns,chunks}));
    Append(root_,name);Append(root_,spelling);Append(root_,field.sha256);
    result_.fields.push_back(std::move(field));
  }
  Digest Finish(){result_.sha256=output::Sha256(root_);return std::move(result_);}
 private:
  void Append(std::string& to,const std::string& value) const {
    output::Require(to.size()<=cap_ && value.size()<=cap_-to.size(),"Topology digest metadata exceeds cap");to+=value;
  }
  std::size_t cap_;std::string root_;Digest result_;
};
}
Digest InputDigest(const Inputs& in,const Config& config,std::size_t cap) {
  output::Require(in.node_ids.size()<=SIZE_MAX/3 && in.positions.size()==3*in.node_ids.size() &&
      in.origin.size()==in.primary.size(),"Incomplete topology input digest extents");
  Fields fields("canonical-record-order:si-to-declared-native-working-units",cap);
  fields.Add<std::uint64_t>("policy",1,3,[&](std::size_t i){return std::uint64_t(i==0?int(config.selection):i==1?int(config.order):int(config.working_units));});
  fields.Add<double>("declared_unit_scale",1,3,[&](std::size_t i){return i==0?in.units.length_m:i==1?in.units.mass_kg:in.units.time_s;});
  fields.Add<std::uint64_t>("node_ids",in.node_ids.size(),1,[&](auto i){return in.node_ids[i];});
  fields.Add<double>("canonical_position_m",in.node_ids.size(),3,[&](auto i){return in.positions[i];});
  fields.Add<std::uint64_t>("primary_source_ids",in.primary.size(),1,[&](auto i){return in.primary[i].source_id;});
  fields.Add<std::uint32_t>("primary_nodes",in.primary.size(),4,[&](auto i){return in.primary[i/4].nodes[i%4];});
  fields.Add<std::uint32_t>("primary_layout",in.primary.size(),1,[&](auto i){return std::uint32_t(in.primary[i].layout);});
  fields.Add<std::uint64_t>("primary_pid",in.origin.size(),1,[&](auto i){return in.origin[i].pid;});
  fields.Add<std::uint32_t>("canonical_row",in.origin.size(),1,[&](auto i){return in.origin[i].canonical_row;});
  fields.Add<std::uint32_t>("source_line",in.origin.size(),1,[&](auto i){return in.origin[i].source_line;});
  return fields.Finish();
}
Digest TopologyDigest(const s::Snapshot& v,const std::string& input,std::size_t cap) {
  output::Require(input.size()==64,"Topology digest requires its source input binding");
  output::arrays::CheckHash(input);
  output::Require(v.mains&&v.starter.face_normals&&v.starter.references&&v.expanded_to_primary&&v.primary_to_partner&&
      v.normal_offsets&&v.normal_mains,"Incomplete successful topology digest view");
  Fields f(input,cap);const auto g=v.main_count,r=v.starter.reference_count;
  const std::uint64_t header[]{v.node_count,v.primary_count,g,r,v.normal_incidence_count,v.source_generation,std::uint64_t(v.profile),std::uint64_t(v.topology)};
  f.Add<std::uint64_t>("header",1,8,[&](auto i){return header[i];});
  f.Add<std::uint64_t>("main_source_ids",g,1,[&](auto i){return v.mains[i].source_id;});
  f.Add<std::uint32_t>("main_nodes",g,4,[&](auto i){return v.mains[i/4].nodes[i%4];});
  f.Add<std::int32_t>("main_global_ids",g,1,[&](auto i){return std::int32_t(v.mains[i].global_id);});
  f.Add<std::int32_t>("main_segment_type",g,1,[&](auto i){return std::int32_t(v.mains[i].segment_type);});
  f.Add<std::int32_t>("main_neighbors",g,4,[&](auto i){return std::int32_t(v.mains[i/4].neighbors[i%4]);});
  f.Add<std::int32_t>("main_neighbor_edges",g,4,[&](auto i){return std::int32_t(v.mains[i/4].neighbor_edges[i%4]);});
  f.Add<std::int32_t>("main_references",g,4,[&](auto i){return std::int32_t(v.mains[i/4].normal_reference[i%4]);});
  f.Add<std::uint32_t>("expanded_to_primary",g,1,[&](auto i){return v.expanded_to_primary[i];});
  f.Add<std::uint32_t>("primary_to_partner",v.primary_count,1,[&](auto i){return v.primary_to_partner[i];});
  f.Add<std::uint32_t>("normal_offsets",r+1,1,[&](auto i){return v.normal_offsets[i];});
  f.Add<std::uint32_t>("normal_mains",v.normal_incidence_count,1,[&](auto i){return v.normal_mains[i];});
  f.Add<std::uint32_t>("starter_normals_f32",4*g,3,[&](auto i){const auto& x=v.starter.face_normals[i/3];return FloatBits(i%3==0?x.x:i%3==1?x.y:x.z);});
  f.Add<std::int32_t>("starter_boundary",r,1,[&](auto i){return std::int32_t(v.starter.references[i].boundary);});
  f.Add<std::uint32_t>("starter_bisectors_f32",r,6,[&](auto i){const auto& x=v.starter.references[i/6].bisector[(i%6)/3];return FloatBits(i%3==0?x.x:i%3==1?x.y:x.z);});
  return f.Finish();
}
} // namespace crash::cases::vehicle_self_contact::native::detail
