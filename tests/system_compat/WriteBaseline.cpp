#include "tests/system_compat/SystemFixture.h"
#include "tests/archive_support/StreamArchive.h"
#include "chrono/serialization/ChArchiveBinary.h"
#include "chrono/serialization/ChArchiveJSON.h"
#include "chrono/serialization/ChArchiveXML.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <typeinfo>

namespace {
using chrono::make_ChNameValue;
using robodyna::system_compat::Fixture;

template <class Writer, class Reader>
std::string WriteAndCheck(Fixture& fixture, const std::filesystem::path& path) {
    const auto bytes = robodyna::archive_test::Write<Writer>(fixture);
    Fixture loaded;
    robodyna::archive_test::Read<Reader>(bytes, loaded);
    robodyna::system_compat::Check(loaded, true);
    if (robodyna::archive_test::Write<Writer>(fixture) != bytes)
        throw std::runtime_error("system fixture repeated writer is not deterministic");
    if (bytes.size() > 1024 * 1024)
        throw std::runtime_error("system fixture exceeds the 1 MiB per-format bound");
    std::ofstream output(path, std::ios::binary);
    output.write(bytes.data(), bytes.size());
    output.close();
    if (!output)
        throw std::runtime_error("system fixture write failed");
    return bytes;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2)
            throw std::runtime_error("usage: write_baseline NEW_OUTPUT_DIRECTORY");
        const std::filesystem::path output(argv[1]);
        if (!std::filesystem::create_directory(output))
            throw std::runtime_error("system baseline directory exists; refusing overwrite");
        // Construct this graph first and only once. Its inherited assembly names
        // are auto-generated; do not normalize or mutate protected names.
        Fixture fixture;
        robodyna::system_compat::Populate(fixture);
        robodyna::system_compat::Check(fixture, false);
        const auto json = WriteAndCheck<chrono::ChArchiveOutJSON, chrono::ChArchiveInJSON>(fixture, output / "baseline.json");
        WriteAndCheck<chrono::ChArchiveOutXML, chrono::ChArchiveInXML>(fixture, output / "baseline.xml");
        WriteAndCheck<chrono::ChArchiveOutBinary, chrono::ChArchiveInBinary>(fixture, output / "baseline.bin");
        const std::string system_rtti = typeid(chrono::ChSystem).name();
        const std::string system_version_key = "_version_" + system_rtti;
        if (json.find('"' + system_version_key + '"') == std::string::npos)
            throw std::runtime_error("unregistered system version key was not observed in the actual archive");
        std::ofstream stream(output / "producer.json");
        {
            chrono::ChArchiveOutJSON archive(stream);
            std::string compiler = __VERSION__;
            auto language = static_cast<unsigned long long>(__cplusplus);
            auto pointer_bytes = static_cast<unsigned long long>(sizeof(void*));
            auto size_t_bytes = static_cast<unsigned long long>(sizeof(std::size_t));
            std::string nsc_rtti = typeid(chrono::ChSystemNSC).name();
            std::string smc_rtti = typeid(chrono::ChSystemSMC).name();
            std::string scope = "Pre-system rename: represented metadata/settings and body ownership; no contact/history/timestepper restart";
            archive << CHNVP(compiler) << CHNVP(language) << CHNVP(pointer_bytes) << CHNVP(size_t_bytes)
                    << CHNVP(system_rtti) << CHNVP(system_version_key) << CHNVP(nsc_rtti) << CHNVP(smc_rtti) << CHNVP(scope);
        }
        stream.close();
        if (!stream)
            throw std::runtime_error("system producer metadata write failed");
        std::cout << "System metadata archives passed three formats; actual base version key: " << system_version_key << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
