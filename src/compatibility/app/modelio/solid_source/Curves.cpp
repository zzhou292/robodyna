#include "Internal.h"
#include "modelio/source_assembly/SourceCurve.h"
namespace crash::modelio::solid_source::detail {
void ReadCurveData(const tied_shell::SourceEvidence& source,std::uint64_t id,
    std::size_t count,double scale,std::vector<double>& x,std::vector<double>& y) {
    assembly::reader::ReadSourceCurve(source.block,source.cards,id,count,scale,x,y);
}
} // namespace crash::modelio::solid_source::detail
