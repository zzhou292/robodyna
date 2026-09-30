#pragma once
#include "SourcePartWallSetup.h"
#include "qualification/source_contact/SourceShellCollection.h"
#include "case/CanonicalWallArtifacts.h"
#include "output/ArtifactIO.h"
#include <cstring>
#include <sstream>

namespace crash::cases::source_part_wall::check {
inline std::string SourcePath,WallPath;
inline constexpr double Step=0x1p-24;
inline constexpr std::uint64_t Configuration=0x5350574346473031ULL,Qualification=0x535057554e495431ULL,WallBinding=0x53505757414c4c31ULL;
struct Fixture {
    source::SourcePartContactFixture source;
    source::SourceShellCollection collection;
    tl::fea::ShellBatchBinding binding;
    case_data::CanonicalWall wall;
    std::string wall_bytes;
    SourcePartWallSettings settings;
    std::array<double,source::NodeCount> inverse{},inverse_j{};
    std::array<double,3*source::NodeCount> position{},velocity{},omega{};
    std::array<double,4*source::NodeCount> orientation{};
    std::array<std::uint8_t,source::NodeCount> free{};
    double kinetic=0;
    Fixture():wall_bytes(case_data::ReadPinnedWallManifest(WallPath)) {
        output::Require(source::LoadPinnedSourcePartContact(SourcePath,&source).status==source::FixtureStatus::Ok,
                        "Source wall unit fixture requires the authenticated original part");
        output::Require(collection.Initialize(source).status==source::FixtureStatus::Ok,"Source collection conversion failed");
        output::Require(binding.Initialize(collection.input()).status==tl::fea::ShellBindingStatus::Success,"Native source binding failed");
        std::istringstream stream(wall_bytes);
        output::Require(wall.Load(stream).status==case_data::WallStatus::Ok,"Original canonical wall failed its owning loader");
        settings.configuration_id=Configuration;settings.qualification_id=Qualification;settings.wall_binding_id=WallBinding;
        position=source.coordinates();
        for(unsigned n=0;n<source::NodeCount;++n) {
            const auto& native=binding.nodes()[n].native;
            inverse[n]=1/native.mass;inverse_j[n]=1/native.isotropic_inertia;
            orientation[4*n]=1;velocity[3*n]=1;kinetic+=.5*native.mass;
        }
    }
    tl::fea::NodalStamp Stamp(double dt=Step) const {
        tl::fea::NodalStamp stamp;stamp.owner_id=77;stamp.node_count=source::NodeCount;
        stamp.fixed_dt=dt;stamp.has_rotations=true;stamp.temporal_scheme=tl::fea::NodalTemporalScheme::StaggeredHalfKickStart;
        return stamp;
    }
    SourcePartWallReport Setup(SourcePartWallSetup& prepared,const SourcePartWallSettings& selected,double dt=Step) const {
        return prepared.Initialize(source,binding,Stamp(dt),inverse.data(),kinetic,wall,wall_bytes,selected);
    }
};
template<class T> auto Bytes(const T& value) {
    std::array<unsigned char,sizeof(T)> bytes;std::memcpy(bytes.data(),&value,sizeof(T));return bytes;
}
} // namespace crash::cases::source_part_wall::check
