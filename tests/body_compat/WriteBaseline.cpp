#include "tests/body_compat/BodyFixture.h"
#include "tests/archive_support/StreamArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"
#include <filesystem>
#include <fstream>
#include <iostream>

using chrono::make_ChNameValue;
namespace {
template <class Writer, class Reader>
void WriteAndCheck(const std::filesystem::path& path) {
    robodyna::body_compat::Fixture fixture;
    robodyna::body_compat::Populate(fixture);
    robodyna::body_compat::Check(fixture);
    const auto bytes = robodyna::archive_test::Write<Writer>(fixture);
    robodyna::body_compat::Fixture loaded;
    robodyna::archive_test::Read<Reader>(bytes, loaded);
    robodyna::body_compat::Check(loaded);
    robodyna::body_compat::Fixture second;
    robodyna::body_compat::Populate(second);
    if (robodyna::archive_test::Write<Writer>(second) != bytes)
        throw std::runtime_error("body fixture writer is not deterministic");
    if (bytes.size() > 1024 * 1024)
        throw std::runtime_error("body fixture exceeds the 1 MiB per-format bound");
    std::ofstream output(path, std::ios::binary);
    output.write(bytes.data(), bytes.size());
    output.close();
    if (!output)
        throw std::runtime_error("body fixture write failed");
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2)
            throw std::runtime_error("usage: write_baseline NEW_OUTPUT_DIRECTORY");
        const std::filesystem::path output(argv[1]);
        if (!std::filesystem::create_directory(output))
            throw std::runtime_error("body baseline output exists; refusing overwrite");
        WriteAndCheck<chrono::ChArchiveOutJSON, chrono::ChArchiveInJSON>(output / "baseline.json");
        WriteAndCheck<chrono::ChArchiveOutXML, chrono::ChArchiveInXML>(output / "baseline.xml");
        WriteAndCheck<chrono::ChArchiveOutBinary, chrono::ChArchiveInBinary>(output / "baseline.bin");
        std::ofstream stream(output / "producer.json");
        {
            chrono::ChArchiveOutJSON archive(stream);
            std::string compiler = __VERSION__;
            auto language = static_cast<unsigned long long>(__cplusplus);
            auto pointer_bytes = static_cast<unsigned long long>(sizeof(void*));
            auto size_t_bytes = static_cast<unsigned long long>(sizeof(std::size_t));
            std::string body_rtti = typeid(chrono::ChBody).name();
            std::string scope = "Pre-base-body rename: BodyEasyBox constructor and marker/force ownership";
            archive << CHNVP(compiler) << CHNVP(language) << CHNVP(pointer_bytes) << CHNVP(size_t_bytes)
                    << CHNVP(body_rtti) << CHNVP(scope);
        }
        stream.close();
        if (!stream)
            throw std::runtime_error("body baseline producer metadata write failed");
        std::cout << "Body constructor and child-ownership archives passed three formats and deterministic writes.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
