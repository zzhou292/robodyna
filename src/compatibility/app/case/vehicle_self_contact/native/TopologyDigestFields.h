#pragma once
#include "TopologyAssessmentInternal.h"
#include <algorithm>
#include <cstring>
namespace crash::cases::vehicle_self_contact::native::detail::digest {
constexpr std::size_t ChunkWords=65536;
inline std::string Integers(std::initializer_list<std::uint64_t> words) {
  return output::arrays::Encode<std::uint64_t>({output::arrays::Scalar::UInt64,words.size(),1,{}},words.begin(),words.size());
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
} // namespace crash::cases::vehicle_self_contact::native::detail::digest
