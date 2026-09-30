#pragma once
#include "PhysicalAcceptedFrames.h"
#include "Fields.h"
namespace crash::output::physical_frames {
struct PhysicalAcceptedFrames::Impl {
    Impl(const Mapping& m,records::Context c,Forecast f)
        :mapping(m),context(std::move(c)),forecast(f),frames(context),
         positions(3*f.physical_nodes),velocities(3*f.physical_nodes),
         layered(f.layered_rows),qbat(f.qbat_rows),flags(std::max(f.layered_rows,f.qbat_rows)) {}
    Mapping mapping;
    records::Context context;
    Forecast forecast;
    detail::FrameBuffers frames;
    std::vector<double> positions,velocities;
    std::vector<tl::fea::ShellBatchLayeredSection> layered;
    std::vector<tl::fea::qbat::BatchResult> qbat;
    std::vector<std::uint8_t> flags;
};
namespace detail {
void CheckBacking(const Mapping&,const Run&);
records::Context MakeContext(const Mapping&,Run&,records::Identity,Limits);
} // namespace detail
} // namespace crash::output::physical_frames
