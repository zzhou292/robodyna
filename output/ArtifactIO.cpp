#include "ArtifactIO.h"
#include "chrono_thirdparty/rapidjson/ostreamwrapper.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include <openssl/evp.h>

#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>

namespace crash::output {

void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::string ReadBounded(const std::filesystem::path& path, std::size_t cap) {
    std::ifstream input(path, std::ios::binary);
    Require(bool(input), "Required artifact/input could not be opened");
    std::string result;
    char chunk[4096];
    while (input) {
        input.read(chunk, sizeof(chunk));
        const auto count = input.gcount();
        Require(count >= 0 && static_cast<std::size_t>(count) <= cap - result.size(),
                "Artifact/input exceeds byte cap");
        result.append(chunk, static_cast<std::size_t>(count));
    }
    Require(input.eof() && !input.bad(), "Artifact/input read failed");
    return result;
}

std::string Sha256(const std::string& bytes) {
    std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
    unsigned size = 0;
    std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    Require(bool(context) && EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) == 1 &&
                EVP_DigestUpdate(context.get(), bytes.data(), bytes.size()) == 1 &&
                EVP_DigestFinal_ex(context.get(), digest.data(), &size) == 1 && size == 32,
            "OpenSSL SHA256 failed");
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (unsigned i = 0; i < size; ++i) text << std::setw(2) << unsigned(digest[i]);
    return text.str();
}

std::uint64_t Bits(double value) {
    static_assert(sizeof(value) == sizeof(std::uint64_t), "Artifacts require binary64 doubles");
    std::uint64_t result;
    std::memcpy(&result, &value, sizeof(result));
    return result;
}

void String(Document& doc, const char* name, const std::string& value) {
    Value key(name, doc.GetAllocator());
    Value text(value.c_str(), static_cast<rapidjson::SizeType>(value.size()), doc.GetAllocator());
    doc.AddMember(key, text, doc.GetAllocator());
}

void Number(Document& doc, const char* name, double value) {
    Require(std::isfinite(value), "Nonfinite artifact metric");
    Value key(name, doc.GetAllocator());
    doc.AddMember(key, value, doc.GetAllocator());
}

void Integer(Document& doc, const char* name, std::uint64_t value) {
    Value key(name, doc.GetAllocator()), number;
    number.SetUint64(value);
    doc.AddMember(key, number, doc.GetAllocator());
}

void Boolean(Document& doc, const char* name, bool value) {
    Value key(name, doc.GetAllocator());
    doc.AddMember(key, value, doc.GetAllocator());
}

void WriteJson(const std::filesystem::path& path, const Document& doc) {
    Require(!std::filesystem::exists(path), "Refusing to overwrite an artifact");
    std::ofstream output(path, std::ios::binary);
    Require(bool(output), "Could not create JSON artifact");
    rapidjson::OStreamWrapper stream(output);
    rapidjson::PrettyWriter<rapidjson::OStreamWrapper> writer(stream);
    Require(doc.Accept(writer), "JSON artifact serialization failed");
    output << '\n';
    output.flush();
    Require(bool(output), "JSON artifact write failed");
    output.close();
    Require(!output.fail(), "JSON artifact close failed");
}

void WriteBytes(const std::filesystem::path& path, const std::string& bytes) {
    Require(!std::filesystem::exists(path), "Refusing to overwrite an artifact");
    std::ofstream output(path, std::ios::binary);
    Require(bool(output), "Could not create input copy");
    output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
    output.flush();
    Require(bool(output), "Input copy write failed");
    output.close();
    Require(!output.fail(), "Input copy close failed");
}

}  // namespace crash::output
