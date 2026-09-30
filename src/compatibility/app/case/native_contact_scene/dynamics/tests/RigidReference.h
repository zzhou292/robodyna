#pragma once
#include "NativeTrajectoryAssertions.h"
#include "RigidGauge.h"
#include <fstream>
namespace crash::cases::native_scene::rigid_trajectory_test {
struct Frame {std::uint64_t epoch=0;std::array<double,186> values{};};
class Reader {
    std::ifstream stream;
    template<class T> void ReadBits(T& value) {
        unsigned char raw[sizeof(T)];stream.read(reinterpret_cast<char*>(raw),sizeof(raw));
        if(!stream)throw std::runtime_error("Truncated rigid expected fixture");
        std::uint64_t bits=0;for(unsigned i=0;i<sizeof(T);++i)bits|=std::uint64_t(raw[i])<<(8*i);
        if constexpr(sizeof(T)==8)std::memcpy(&value,&bits,8);
        else {const auto word=std::uint32_t(bits);std::memcpy(&value,&word,4);}
    }
  public:
    explicit Reader(const char* path):stream(path,std::ios::binary) {
        char magic[8];stream.read(magic,8);
        if(!stream||std::memcmp(magic,"T25RIG01",8))throw std::runtime_error("Invalid rigid expected format");
        std::uint32_t nodes=0,groups=0,frames=0;ReadBits(nodes);ReadBits(groups);ReadBits(frames);
        if(nodes!=18||groups!=1||frames!=1001)throw std::runtime_error("Rigid expected extent differs");
    }
    Frame Read(std::uint64_t epoch) {
        Frame result;ReadBits(result.epoch);if(result.epoch!=epoch)throw std::runtime_error("Rigid expected cycle is not sequential");
        for(auto& x:result.values){ReadBits(x);if(!std::isfinite(x))throw std::runtime_error("Nonfinite rigid expected scalar");}
        return result;
    }
    void Finish(){if(stream.peek()!=std::char_traits<char>::eof())throw std::runtime_error("Trailing rigid expected bytes");}
};
inline std::array<double,18> GroupValues(const tl::fea::NodalRigidGroupSnapshot& g) {
    std::array<double,18> result{};tl::fea::rigid::WriteGroupState(result.data(),g.state);return result;
}
inline void SameGroup(const tl::fea::NodalRigidGroupSnapshot& a,const tl::fea::NodalRigidGroupSnapshot& b) {
    EXPECT_EQ(a.source_kind,b.source_kind);EXPECT_EQ(a.source_group_id,b.source_group_id);EXPECT_EQ(a.source_node_set_id,b.source_node_set_id);
    const auto x=GroupValues(a),y=GroupValues(b);for(unsigned i=0;i<18;++i)EXPECT_EQ(output::Bits(x[i]),output::Bits(y[i]));
}
inline gauge::Matrix Axes(const tl::fea::NodalRigidGroupSnapshot& group) {
    gauge::Matrix result{};for(unsigned i=0;i<9;++i)result[i]=group.state.principal_axes.v[i];return result;
}
inline gauge::Matrix Axes(const Frame& frame) {
    gauge::Matrix result{};for(unsigned r=0;r<3;++r)for(unsigned c=0;c<3;++c)result[3*r+c]=frame.values[63+3*c+r];return result;
}
inline void Motion(const output::physical_frames::NativeAcceptedState& state,
    const tl::fea::NodalRigidGroupSnapshot& group,const Frame& current,const Frame& force_base,const gauge::Alignment& initial) {
    using trajectory_test::Number;
    ASSERT_EQ(state.stamp.epoch,current.epoch);ASSERT_EQ(group.source_kind,tl::fea::RigidBindingSourceKind::Part);
    ASSERT_EQ(group.source_group_id,2u); // Physical PART2 maps to reference body1/primary19.
    for(unsigned i=0;i<54;++i)Number(state.spin_xyz[i],current.values[i],2e-8);
    const auto values=GroupValues(group);
    for(unsigned i=0;i<3;++i)Number(values[i]/.001,current.values[54+i],2e-8);
    for(unsigned i=0;i<3;++i)Number(values[3+i]/.001,current.values[57+i],2e-5);
    for(unsigned i=0;i<3;++i)Number(values[6+i],current.values[60+i],2e-8);
    const auto actual_axes=Axes(group),native_axes=Axes(force_base);
    ASSERT_TRUE(gauge::Proper(actual_axes));
    ASSERT_TRUE(gauge::Proper(native_axes));
    const auto actual_rotation=gauge::Rotation(actual_axes,initial.actual_initial);
    const auto native_rotation=gauge::Rotation(native_axes,initial.native_initial);
    for(unsigned i=0;i<9;++i)Number(actual_rotation[i],native_rotation[i],gauge::Absolute,gauge::Relative);
}
}
