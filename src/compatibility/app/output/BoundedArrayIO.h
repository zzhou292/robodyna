#pragma once
#include "ArtifactIO.h"
#include "ArtifactInventory.h"
#include <cstring>
#include <vector>

namespace crash::output::arrays {
enum class Scalar { UInt16, UInt32, UInt64, Int32, Float64 };
struct Layout {
    Scalar scalar=Scalar::Float64;
    std::uint64_t rows=0;
    std::size_t columns=1;
    // Empty means the canonical descriptor's explicit null fields.
    std::vector<std::string> fields;
};
struct Descriptor {
    std::string file;
    Layout layout;
    std::size_t bytes=0;
    std::string sha256;
};
struct Limits {
    std::size_t file_bytes=kArtifactFileCap;
    std::uint64_t rows=UINT32_MAX;
    std::size_t columns=64;
};
const char* Dtype(Scalar);
std::size_t ScalarBytes(Scalar);
std::size_t ByteCount(const Layout&,Limits={});
void CheckHash(const std::string&);
void CheckRelativeName(const std::string&);
void CheckDescriptor(const Descriptor&,Limits={});
Document DescriptorDocument(const Descriptor&,Limits={});
Descriptor ParseDescriptor(const Value&,Limits={});
// All path components must be existing real directories under the real root.
// Calls and files are externally serialized, like ArtifactIO. No directories,
// completion manifests, solver state or concurrent-writer guarantees are owned.
std::filesystem::path CheckedPath(const std::filesystem::path&,const std::string&,bool existing);
std::string ReadBytes(const std::filesystem::path&,const Descriptor&,Limits={});
Descriptor WriteBytes(const std::filesystem::path&,const std::string&,const Layout&,
                      const std::string& little_endian_bytes,Limits={});
namespace detail {
std::string Encode(const Layout&,const void*,std::size_t,Scalar,Limits);
std::string Decode(const Descriptor&,const std::string&,Scalar,Limits);
template<class T> struct Type; // Unsupported types fail at compile time.
template<> struct Type<std::uint16_t>{static constexpr Scalar value=Scalar::UInt16;};
template<> struct Type<std::uint32_t>{static constexpr Scalar value=Scalar::UInt32;};
template<> struct Type<std::uint64_t>{static constexpr Scalar value=Scalar::UInt64;};
template<> struct Type<std::int32_t>{static constexpr Scalar value=Scalar::Int32;};
template<> struct Type<double>{static constexpr Scalar value=Scalar::Float64;};
}
template<class T> std::string Encode(const Layout& l,const T* data,std::size_t count,Limits limits={}) {
    return detail::Encode(l,data,count,detail::Type<T>::value,limits);
}
template<class T> std::vector<T> Decode(const Descriptor& d,const std::string& bytes,Limits limits={}) {
    const auto native=detail::Decode(d,bytes,detail::Type<T>::value,limits);
    std::vector<T> result(native.size()/sizeof(T));
    if(!native.empty())std::memcpy(result.data(),native.data(),native.size());
    return result; // No caller-owned output can be partially replaced.
}
template<class T> std::vector<T> Read(const std::filesystem::path& root,const Descriptor& d,Limits limits={}) {
    // Type and extent admission precede file reads.
    CheckDescriptor(d,limits);Require(d.layout.scalar==detail::Type<T>::value,"Array scalar type mismatch");
    return Decode<T>(d,ReadBytes(root,d,limits),limits);
}
template<class T> Descriptor Write(const std::filesystem::path& root,const std::string& name,
        const Layout& l,const T* data,std::size_t count,Limits limits={}) {
    const auto bytes=Encode(l,data,count,limits);
    return WriteBytes(root,name,l,bytes,limits);
}
} // namespace crash::output::arrays
