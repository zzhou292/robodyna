#pragma once
#include "NativeContactValues.h"
#include <array>
namespace crash::output::physical_run {
enum class NativeContactRole : std::uint64_t { Unspecified=0,Self=1,MeshWall=2 };
struct NativeGroupEntry {
    NativeContactRole role=NativeContactRole::Unspecified;
    NativeContactValues publication;
};
inline constexpr std::size_t NativeGroupCapacity=2;
struct NativeGroupValues {
    std::uint64_t count=0;
    std::array<NativeGroupEntry,NativeGroupCapacity> entries{};
};
inline constexpr std::size_t NativeGroupIntegerCount=1+NativeGroupCapacity*(1+NativeContactIntegerCount);
inline constexpr std::size_t NativeGroupRealCount=NativeGroupCapacity*NativeContactRealCount;
void CheckNativeGroupValues(const NativeGroupValues&);
void CheckNativeGroupContinuation(const NativeGroupValues&,const NativeGroupValues&);
std::vector<std::string> NativeGroupIntegerFields();
std::vector<std::string> NativeGroupRealFields();
void EncodeNativeGroup(const NativeGroupValues&,std::uint64_t*,double*);
NativeGroupValues DecodeNativeGroup(const std::uint64_t*,const double*);
}
