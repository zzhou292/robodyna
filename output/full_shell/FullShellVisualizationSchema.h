#pragma once
#include "output/BoundedArrayIO.h"
#include "IntervalValues.h"
#include <memory>

namespace crash::output::full_shell {
inline constexpr const char* FrameSchema="robo_dyna.full_shell_visualization_frame.v1";
inline constexpr const char* ArchiveSchema="robo_dyna.full_shell_accepted_visualization.v1";
inline constexpr std::size_t TotalByteCap=kArtifactMaximumTotalCap;
inline constexpr std::size_t FrameMetadataByteCap=16*1024;
inline constexpr std::size_t IntervalCoreBytes=interval::RowBytes;
inline constexpr std::size_t StaticReserveBytes=192*1024*1024;
struct RecordLimits {
    std::size_t nodes=1048576,parents=1048576,points=4194304;
    std::size_t host_bytes=256*1024*1024;
    arrays::Limits arrays;
};
enum class PlasticField { NativeEquivalentPlasticStrain, NotApplicable, Unavailable };
struct ParentPoints {
    std::uint64_t source_element=0,source_part=0;
    // Actual resolved constitutive count. Rigid/nonplastic/unavailable parents
    // can have zero points; the original source NIP belongs to source authority.
    std::uint32_t source_elform=0,native_family=0,native_points=0;
    PlasticField plastic=PlasticField::Unavailable;
};
struct Identity {
    std::uint64_t owner=0,run=0,topology=0,source_instance=0,configuration=0,qualification=0;
    std::uint64_t source_inventory_bytes=0;
    std::string source_inventory_sha256,source_mapping_sha256;
};
bool SameIdentity(const Identity&,const Identity&) noexcept;
void CheckIdentity(const Identity&);
struct FrameStamp {
    std::uint64_t epoch=0,base_epoch=0,attempt=0;
    double time=0,base_time=0,velocity_time=0,kick_dt=0;
};
// Immutable record context. Identity must come from the caller's authenticated
// source/runtime mapping. This class validates declaration structure; it does
// not parse the vehicle inventory, authenticate live mechanics, or own a clock.
// Native family is an opaque declared identifier, not inferred from ELFORM.
class Context {
  public:
    Context(const Context&) noexcept=default;
    Context(Context&& other) noexcept:data_(other.data_){}
    Context& operator=(const Context&)=delete;
    Context& operator=(Context&&)=delete;
    static Context Create(Identity,std::size_t nodes,const ParentPoints*,std::size_t parents,
                          double fixed_dt,RecordLimits={});
    const Identity& identity() const noexcept;
    std::size_t nodes() const noexcept;
    std::size_t points() const noexcept;
    const std::vector<ParentPoints>& parents() const noexcept;
    const std::vector<std::size_t>& point_offsets() const noexcept;
    const std::string& point_layout_sha256() const noexcept;
    double fixed_dt() const noexcept;
    const RecordLimits& limits() const noexcept;
  private:
    struct Data;
    explicit Context(std::shared_ptr<const Data> data):data_(std::move(data)){}
    std::shared_ptr<const Data> data_;
};
void CheckStamp(const Context&,const FrameStamp&);
void CheckStamp(double fixed_dt,const FrameStamp&);
bool SameStamp(const FrameStamp&,const FrameStamp&) noexcept;
} // namespace crash::output::full_shell
