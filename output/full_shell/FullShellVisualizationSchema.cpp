#include "FullShellVisualizationSchema.h"
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace crash::output::full_shell {
bool SameIdentity(const Identity& a, const Identity& b) noexcept {
    return a.owner == b.owner && a.run == b.run && a.topology == b.topology &&
        a.source_instance == b.source_instance && a.configuration == b.configuration &&
        a.qualification == b.qualification && a.source_inventory_bytes == b.source_inventory_bytes &&
        a.source_inventory_sha256 == b.source_inventory_sha256 && a.source_mapping_sha256 == b.source_mapping_sha256;
}

void CheckIdentity(const Identity& id) {
    Require(id.owner&&id.run&&id.topology&&id.source_instance&&id.configuration&&id.qualification&&
        id.source_inventory_bytes&&id.source_inventory_bytes<=TotalByteCap,"Invalid record source identity");
    arrays::CheckHash(id.source_inventory_sha256);
    arrays::CheckHash(id.source_mapping_sha256);
}
bool SameStamp(const FrameStamp& a,const FrameStamp& b) noexcept {
    return a.epoch==b.epoch&&a.base_epoch==b.base_epoch&&a.attempt==b.attempt&&
        Bits(a.time)==Bits(b.time)&&Bits(a.base_time)==Bits(b.base_time)&&
        Bits(a.velocity_time)==Bits(b.velocity_time)&&Bits(a.kick_dt)==Bits(b.kick_dt);
}
struct Context::Data {
    Identity identity;
    std::size_t nodes=0,points=0;
    double fixed_dt=0;
    RecordLimits limits;
    std::vector<ParentPoints> parents;
    std::vector<std::size_t> offsets;
    std::string point_hash;
};

Context Context::Create(Identity id, std::size_t nodes, const ParentPoints* parents, std::size_t count,
                        double dt, RecordLimits limits) {
    Require(limits.nodes && limits.nodes <= 1048576 && limits.parents && limits.parents <= 1048576 &&
        limits.points <= 4194304 && limits.host_bytes && limits.host_bytes <= 512*1024*1024 &&
        nodes && nodes <= limits.nodes && count && count <= limits.parents && parents,
        "Record context count/capacity mismatch");
    CheckIdentity(id);
    Require(std::isfinite(dt) && dt > 0 && std::isfinite(dt * .5) && dt * .5 > 0, "Invalid fixed record timestep");
    const auto positions = arrays::ByteCount({arrays::Scalar::Float64, nodes, 3, {}}, limits.arrays);
    // This conservative startup bound includes sorted ID validation scratch,
    // immutable table/offsets, two decoded frames and two raw staging buffers.
    const auto table_per = sizeof(ParentPoints) + sizeof(std::size_t) + 7 * sizeof(std::uint64_t);
    Require(limits.host_bytes >= sizeof(Data) + sizeof(std::size_t) &&
        count <= (limits.host_bytes - sizeof(Data) - sizeof(std::size_t)) / table_per,
        "Record context table exceeds host budget");
    const auto table = sizeof(Data) + count * table_per + sizeof(std::size_t);
    Require(table <= limits.host_bytes && positions <= (limits.host_bytes - table) / 4,
        "Record positions exceed host budget");

    std::vector<std::uint64_t> ids;
    ids.reserve(count);
    std::size_t points = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const auto& p = parents[i];
        Require(p.source_element && p.source_part && p.source_elform && p.native_family &&
            p.native_points <= 64,
            "Invalid source parent or native point declaration");
        ids.push_back(p.source_element);
        switch (p.plastic) {
            case PlasticField::NativeEquivalentPlasticStrain:
                Require(p.native_points && p.native_points <= limits.points - points,
                    "Native plastic point count is zero or exceeds capacity");
                points += p.native_points;
                break;
            case PlasticField::NotApplicable:
            case PlasticField::Unavailable:
                break;
            default:
                throw std::runtime_error("Invalid plastic field applicability");
        }
    }
    std::sort(ids.begin(), ids.end());
    Require(std::adjacent_find(ids.begin(), ids.end()) == ids.end(), "Duplicate source parent");
    const auto point_bytes = arrays::ByteCount({arrays::Scalar::Float64, points, 1, {}}, limits.arrays);
    Require(point_bytes <= (limits.host_bytes - table - 4 * positions) / 4, "Record fields exceed host budget");

    auto data = std::make_shared<Data>();
    data->identity = std::move(id);
    data->nodes = nodes;
    data->points = points;
    data->fixed_dt = dt;
    data->limits = limits;
    data->parents.assign(parents, parents + count);
    data->offsets.reserve(count + 1);
    std::size_t offset = 0;
    data->offsets.push_back(offset);
    for (const auto& p : data->parents) {
        if (p.plastic == PlasticField::NativeEquivalentPlasticStrain) offset += p.native_points;
        data->offsets.push_back(offset);
    }
    // Bind ordered point applicability and source/family declarations explicitly;
    // no native struct padding or host byte order enters the identity.
    std::string point_bytes_le;
    point_bytes_le.reserve(6 * sizeof(std::uint64_t) * count);
    for (const auto& p : data->parents) {
        for (std::uint64_t value : {p.source_element, p.source_part, std::uint64_t(p.source_elform),
             std::uint64_t(p.native_family), std::uint64_t(p.native_points), std::uint64_t(p.plastic)}) {
            for (unsigned j = 0; j < 8; ++j)
                point_bytes_le.push_back(static_cast<char>((value >> (8 * j)) & 255));
        }
    }
    data->point_hash = Sha256(point_bytes_le);
    return Context(std::move(data));
}
const Identity& Context::identity() const noexcept{return data_->identity;}
std::size_t Context::nodes() const noexcept{return data_->nodes;}
std::size_t Context::points() const noexcept{return data_->points;}
const std::vector<ParentPoints>& Context::parents() const noexcept{return data_->parents;}
const std::vector<std::size_t>& Context::point_offsets() const noexcept{return data_->offsets;}
const std::string& Context::point_layout_sha256() const noexcept{return data_->point_hash;}
double Context::fixed_dt() const noexcept{return data_->fixed_dt;}
const RecordLimits& Context::limits() const noexcept{return data_->limits;}
std::size_t Context::retained_payload_bytes() const noexcept {
    std::size_t bytes=sizeof(Data);
    const auto add=[&](std::size_t count,std::size_t width) {
        if(count>(SIZE_MAX-bytes)/width)return false;bytes+=count*width;return true;
    };
    if(!add(data_->parents.capacity(),sizeof(ParentPoints))||!add(data_->offsets.capacity(),sizeof(std::size_t)))return SIZE_MAX;
    for(const auto* text:{&data_->identity.source_inventory_sha256,&data_->identity.source_mapping_sha256,&data_->point_hash})
        if(!add(text->capacity(),1)||!add(1,1))return SIZE_MAX;
    return bytes;
}

void CheckStamp(const Context& context, const FrameStamp& s) {
    CheckStamp(context.fixed_dt(),s);
}
void CheckStamp(double fixed_dt,const FrameStamp& s) {
    Require(std::isfinite(fixed_dt)&&fixed_dt>0&&fixed_dt*.5>0,"Invalid fixed record timestep");
    Require(std::isfinite(s.time) && std::isfinite(s.base_time) && std::isfinite(s.velocity_time) &&
        std::isfinite(s.kick_dt) && s.time >= 0 && s.base_time >= 0 && s.velocity_time >= 0,
        "Invalid record phase values");
    if (!s.epoch) {
        Require(!s.base_epoch && !s.attempt && Bits(s.time) == Bits(0.) && Bits(s.base_time) == Bits(0.) &&
            Bits(s.velocity_time) == Bits(0.) && Bits(s.kick_dt) == Bits(0.), "Invalid initial record phase");
        return;
    }
    const auto dt = fixed_dt;
    const double endpoint = s.base_time + dt, midpoint = s.base_time + .5 * dt;
    const double kick = s.epoch == 1 ? .5 * dt : dt;
    Require(s.base_epoch == s.epoch - 1 && s.attempt && s.time > s.base_time &&
        std::isfinite(endpoint) && std::isfinite(midpoint) && Bits(s.time) == Bits(endpoint) &&
        Bits(s.velocity_time) == Bits(midpoint) && Bits(s.kick_dt) == Bits(kick) &&
        (s.epoch != 1 || Bits(s.base_time) == Bits(0.)), "Record is not an accepted staggered interval");
}
} // namespace crash::output::full_shell
