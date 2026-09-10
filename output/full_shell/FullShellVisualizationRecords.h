#pragma once
#include "FullShellVisualizationSchema.h"
#include <optional>

namespace crash::output::full_shell {
struct FrameInput {
    FrameStamp stamp;
    const double* position_xyz=nullptr;
    std::size_t position_values=0;
    const double* plastic_points=nullptr;
    std::size_t plastic_values=0;
};
struct FrameRecord {
    FrameStamp stamp;
    std::vector<double> position_xyz,plastic_points;
};
struct RecordFile {std::string file,sha256;std::size_t bytes=0;};
struct FrameDescription {
    Identity identity;
    FrameStamp stamp;
    double fixed_dt=0;
    std::string point_layout_sha256;
    std::size_t nodes=0,parents=0,points=0;
    arrays::Descriptor positions,plastic;
};
Document FrameDocument(const Context&,const FrameDescription&);
FrameDescription ParseFrameDocument(const Context&,const Value&);
void CheckFrame(const Context&,const FrameInput&);
// A per-frame serializer, not an archive publisher or acceptance certificate.
// Caller supplies authentic accepted values/identity and serializes all calls.
// All content/destination preflight precedes the first write. I/O failure leaves
// incomplete files as evidence; only a caller-owned completion manifest can
// publish a run. Neither function changes a context or caller-owned frame.
RecordFile WriteFrame(const std::filesystem::path&,const std::string& stem,const Context&,const FrameInput&);
// Expected phase comes from the caller's authenticated frame index/interval
// record; finite but falsely associated epoch/attempt metadata also rejects.
FrameRecord ReadFrame(const std::filesystem::path&,const Context&,const RecordFile&,const FrameStamp& expected);
// Nullopt means no applicable stored native plastic field; it is not zero strain.
std::vector<std::optional<double>> ParentPlasticMaxima(const Context&,const FrameRecord&);
} // namespace crash::output::full_shell
