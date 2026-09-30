#include "IntervalValues.h"
#include "output/ArtifactIO.h"
#include <cmath>
#include <iomanip>
#include <sstream>

namespace crash::output::interval {
void CheckFinite(const Values& values) {
    for (double value : values.reals)
        Require(std::isfinite(value), "Nonfinite accepted interval diagnostic");
}

std::string CsvRow(const Values& values) {
    CheckFinite(values);
    const auto& i = values.integers;
    const auto& r = values.reals;
    std::ostringstream row;
    row << std::setprecision(17) << i[Owner] << ',' << i[BaseEpoch] << ',' << i[Attempt] << ','
        << r[BaseTime] << ',' << i[Epoch] << ',' << r[Time];
    for (std::size_t j = 2; j < r.size(); ++j) row << ',' << r[j];
    row << '\n';
    return row.str();
}

namespace {
std::vector<std::string> Fields(bool integers) {
    std::string header = CsvHeader;
    header.pop_back(); // The fixed canonical header ends in one newline.
    std::istringstream stream(header);
    std::vector<std::string> fields;
    std::string name;
    for (std::size_t column = 0; std::getline(stream, name, ','); ++column) {
        const bool integer = column == 0 || column == 1 || column == 2 || column == 4;
        if (integer == integers) fields.push_back(name);
    }
    Require(fields.size() == (integers ? IntegerCount : RealCount), "Interval column authority changed");
    return fields;
}
}
std::vector<std::string> IntegerFields() { return Fields(true); }
std::vector<std::string> RealFields() { return Fields(false); }
} // namespace crash::output::interval
