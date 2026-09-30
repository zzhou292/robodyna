#include "SourceFields.h"
#include "AuxiliarySourceCards.h"
namespace crash::modelio::assembly::reader {
std::optional<double> SourceScalar(const std::string& line, unsigned field, unsigned width) {
    Require(width > 0 && field <= SIZE_MAX / width, "Invalid fixed-column field index");
    const auto offset = std::size_t(field) * width;
    const auto text = offset < line.size() ? line.substr(offset, width) : std::string{};
    const auto begin = text.find_first_not_of(" \t"), end = text.find_last_not_of(" \t");
    if (begin == std::string::npos) return {};
    auto first = text.data() + begin;
    const auto last = text.data() + end + 1;
    if (first != last && *first == '+') ++first;
    double value = 0;
    const auto parsed = std::from_chars(first, last, value);
    Require(parsed.ec == std::errc{} && parsed.ptr == last && std::isfinite(value),
            "Invalid original numerical card field");
    return value;
}
double RequiredScalar(const std::string& row,unsigned column,unsigned width) {
    const auto value=SourceScalar(row,column,width);
    Require(value.has_value(),"Missing required numerical source field");
    return *value;
}
void RequireBlankFields(const std::string& row,unsigned first,unsigned last,unsigned width) {
    Require(width && first<=last && last<=SIZE_MAX/width,"Invalid source blank-field range");
    Require(auxiliary::BlankTail(row,std::size_t(width)*last),"Extra original source fields");
    for(unsigned field=first;field<last;++field)
        Require(!SourceScalar(row,field,width),"Unsupported original source option");
}
} // namespace crash::modelio::assembly::reader
