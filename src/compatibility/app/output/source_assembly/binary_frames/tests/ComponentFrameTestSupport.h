#pragma once
#include "output/source_assembly/binary_frames/ComponentContext.h"
#include "output/source_assembly/tests/SourceAssemblyWallArtifactTestSupport.h"
#include <gtest/gtest.h>

namespace crash::output::assembly::binary::test {
namespace fixture=crash::output::assembly::test;
inline records::FrameRecord Storage(const records::Context& c) {
    records::FrameRecord f;f.position_xyz.resize(3*c.nodes());f.plastic_points.resize(c.points());return f;
}
inline void Same(const records::FrameRecord& a,const records::FrameRecord& b) {
    EXPECT_TRUE(records::SameStamp(a.stamp,b.stamp));
    ASSERT_EQ(a.position_xyz.size(),b.position_xyz.size());ASSERT_EQ(a.plastic_points.size(),b.plastic_points.size());
    for(std::size_t i=0;i<a.position_xyz.size();++i)EXPECT_EQ(Bits(a.position_xyz[i]),Bits(b.position_xyz[i]))<<i;
    for(std::size_t i=0;i<a.plastic_points.size();++i)EXPECT_EQ(Bits(a.plastic_points[i]),Bits(b.plastic_points[i]))<<i;
}
inline void CompareLegacy(const records::Context& c,const records::FrameRecord& f,const Document& json) {
    const auto& stamp=json["stamp"];const auto& sections=json["sections"];
    EXPECT_EQ(c.identity().owner,json["owner_id"].GetUint64());
    EXPECT_EQ(c.identity().run,json["run_id"].GetUint64());EXPECT_EQ(c.identity().topology,json["topology_id"].GetUint64());
    EXPECT_EQ(c.identity().source_instance,json["source_instance_id"].GetUint64());
    EXPECT_EQ(c.identity().configuration,json["configuration_id"].GetUint64());
    EXPECT_EQ(c.identity().qualification,json["qualification_id"].GetUint64());
    EXPECT_EQ(f.stamp.epoch,stamp["epoch"].GetUint64());EXPECT_EQ(f.stamp.base_epoch,stamp["reaction_base_epoch"].GetUint64());
    EXPECT_EQ(Bits(f.stamp.time),Bits(stamp["time"].GetDouble()));
    EXPECT_EQ(Bits(f.stamp.base_time),Bits(stamp["reaction_time"].GetDouble()));
    EXPECT_EQ(Bits(f.stamp.velocity_time),Bits(stamp["velocity_time"].GetDouble()));
    EXPECT_EQ(Bits(f.stamp.kick_dt),Bits(stamp["reaction_kick_dt"].GetDouble()));
    const auto& nodes=json["nodal_fields"]["position_xyz_m"];
    ASSERT_EQ(f.position_xyz.size(),nodes.Size());ASSERT_EQ(c.parents().size(),sections["source_parents"].Size());
    for(std::size_t i=0;i<f.position_xyz.size();++i)EXPECT_EQ(Bits(f.position_xyz[i]),Bits(nodes[i].GetDouble()))<<i;
    for(std::size_t p=0;p<c.parents().size();++p) {
        const auto& id=sections["source_parents"][p];const auto& source=c.parents()[p];
        EXPECT_EQ(source.source_element,id[1u].GetUint64());EXPECT_EQ(source.source_part,id[2u].GetUint64());
        EXPECT_EQ(source.source_elform,id[6u].GetUint());
        EXPECT_EQ(source.native_family,std::string(id[7u].GetString())=="QEPH"?QephFamily:T3Family);
        ASSERT_EQ(source.native_points,3u);ASSERT_EQ(c.point_offsets()[p+1]-c.point_offsets()[p],3u);
        EXPECT_EQ(source.plastic,records::PlasticField::NativeEquivalentPlasticStrain);
        const auto& points=sections["sections"][p][9u];ASSERT_EQ(points.Size(),3u);
        for(unsigned point=0;point<3;++point)EXPECT_EQ(Bits(f.plastic_points[c.point_offsets()[p]+point]),
            Bits(points[point][5u].GetDouble()))<<p<<":"<<point;
    }
    if(f.stamp.epoch)EXPECT_EQ(f.stamp.attempt,json["contact"]["attempt"].GetUint64());
    else EXPECT_EQ(f.stamp.attempt,0u);
}
} // namespace crash::output::assembly::binary::test
