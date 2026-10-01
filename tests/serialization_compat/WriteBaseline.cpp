#include "tests/serialization_compat/ArchiveFormats.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <typeinfo>

#include "chrono/serialization/ChArchiveJSON.h"

using chrono::make_ChNameValue;

namespace {
void WriteBytes(const std::filesystem::path& path, const std::string& bytes) {
    std::ofstream stream(path, std::ios::binary);
    stream.write(bytes.data(), bytes.size());
    stream.close();
    if (!stream)
        throw std::runtime_error("cannot write baseline: " + path.string());
}
}  // namespace

int main(int argc, char** argv) {
    using namespace robodyna::serialization_compat;
    try {
        if (argc != 2)
            throw std::runtime_error("usage: write_baseline NEW_OUTPUT_DIRECTORY");
        const std::filesystem::path output(argv[1]);
        if (!std::filesystem::create_directory(output))
            throw std::runtime_error("baseline output directory already exists; refusing overwrite");
        for (const auto format : {Format::Json, Format::Xml, Format::Binary}) {
            FixtureModel fixture;
            PopulateFixture(fixture);
            CheckFixture(fixture);
            const auto bytes = WriteFixture(format, fixture);
            FixtureModel loaded;
            ReadFixture(format, bytes, loaded);
            CheckFixture(loaded);
            // Same-process reconstruction catches volatile/uninitialized fields.
            FixtureModel second;
            PopulateFixture(second);
            if (WriteFixture(format, second) != bytes)
                throw std::runtime_error("baseline archive is not deterministic");
            WriteBytes(output / (std::string("baseline.") + Extension(format)), bytes);
        }
        {
            std::ofstream stream(output / "producer.json");
            chrono::ChArchiveOutJSON archive(stream);
            std::string compiler = __VERSION__;
            auto language = static_cast<unsigned long long>(__cplusplus);
            auto pointer_bytes = static_cast<unsigned long long>(sizeof(void*));
            auto size_t_bytes = static_cast<unsigned long long>(sizeof(std::size_t));
            std::string vector_rtti = typeid(chrono::ChVector3d).name();
            std::string frame_rtti = typeid(chrono::ChFramed).name();
            std::string moving_frame_rtti = typeid(chrono::ChFrameMoving<>).name();
            std::string body_frame_rtti = typeid(chrono::ChBodyFrame).name();
            archive << CHNVP(compiler) << CHNVP(language) << CHNVP(pointer_bytes) << CHNVP(size_t_bytes)
                    << CHNVP(vector_rtti) << CHNVP(frame_rtti) << CHNVP(moving_frame_rtti) << CHNVP(body_frame_rtti);
        }
        std::cout << "Wrote three current-implementation archives; round trips and deterministic writes passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
