#include "tests/serialization_compat/ArchiveFormats.h"

#include <fstream>
#include <iterator>
#include <sstream>
#include <stdexcept>

#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

namespace robodyna::serialization_compat {
using chrono::make_ChNameValue;

const char* Extension(Format format) {
    switch (format) {
        case Format::Json: return "json";
        case Format::Xml: return "xml";
        case Format::Binary: return "bin";
    }
    throw std::invalid_argument("unknown archive format");
}

std::string WriteFixture(Format format, FixtureModel& fixture) {
    std::ostringstream stream(std::ios::out | std::ios::binary);
    // Archive destructors finish their format before the resulting bytes are read.
    switch (format) {
        case Format::Json: {
            chrono::ChArchiveOutJSON archive(stream);
            archive << CHNVP(fixture);
            break;
        }
        case Format::Xml: {
            chrono::ChArchiveOutXML archive(stream);
            archive << CHNVP(fixture);
            break;
        }
        case Format::Binary: {
            chrono::ChArchiveOutBinary archive(stream);
            archive << CHNVP(fixture);
            break;
        }
    }
    if (!stream)
        throw std::runtime_error("failed to write archive");
    return stream.str();
}

void ReadFixture(Format format, const std::string& bytes, FixtureModel& fixture) {
    std::istringstream stream(bytes, std::ios::in | std::ios::binary);
    switch (format) {
        case Format::Json: {
            chrono::ChArchiveInJSON archive(stream);
            archive >> CHNVP(fixture);
            break;
        }
        case Format::Xml: {
            chrono::ChArchiveInXML archive(stream);
            archive >> CHNVP(fixture);
            break;
        }
        case Format::Binary: {
            chrono::ChArchiveInBinary archive(stream);
            archive >> CHNVP(fixture);
            break;
        }
    }
}

std::string ReadBytes(const std::string& path) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream)
        throw std::runtime_error("cannot read archive: " + path);
    return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}
}  // namespace robodyna::serialization_compat
