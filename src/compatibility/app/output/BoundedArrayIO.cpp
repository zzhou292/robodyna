#include "BoundedArrayIO.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace crash::output::arrays {
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559, "IEEE binary64 is required");

const char* Dtype(Scalar s) {
    switch (s) {
        case Scalar::UInt16: return "<u2";
        case Scalar::UInt32: return "<u4";
        case Scalar::UInt64: return "<u8";
        case Scalar::Int32: return "<i4";
        case Scalar::Float64: return "<f8";
    }
    throw std::runtime_error("Unsupported array dtype");
}
std::size_t ScalarBytes(Scalar s) {
    const auto* dtype = Dtype(s);
    return static_cast<std::size_t>(dtype[2] - '0');
}

std::size_t ByteCount(const Layout& l, Limits cap) {
    const auto width = ScalarBytes(l.scalar);
    Require(cap.file_bytes && cap.file_bytes <= kArtifactFileCap && cap.rows && cap.rows <= UINT32_MAX &&
        cap.columns && cap.columns <= 64 && l.rows <= cap.rows && l.columns && l.columns <= cap.columns,
        "Array layout exceeds declared capacity");
    Require(l.rows <= cap.file_bytes / width / l.columns, "Array byte count exceeds capacity");
    Require(l.fields.empty() || l.fields.size() == l.columns, "Array field count mismatch");
    std::set<std::string> names;
    for (const auto& f : l.fields) {
        Require(!f.empty() && f.size() <= 128, "Invalid or duplicate array field name");
        const bool printable = std::all_of(f.begin(), f.end(), [](unsigned char c) { return c >= 32 && c <= 126; });
        Require(printable && names.insert(f).second, "Invalid or duplicate array field name");
    }
    return static_cast<std::size_t>(l.rows) * l.columns * width;
}

void CheckHash(const std::string& hash) {
    Require(hash.size() == 64 && hash.find_first_not_of("0123456789abcdef") == std::string::npos,
        "Invalid SHA256 identity");
}

void CheckRelativeName(const std::string& name) {
    Require(!name.empty() && name.size() <= 255 && name.front() != '/' && name.back() != '/' &&
        name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_./") == std::string::npos,
        "Unsafe array path");
    for (const auto& part : std::filesystem::path(name))
        Require(part != "." && part != ".." && !part.empty(), "Unsafe array path component");
    Require(name.find("//") == std::string::npos, "Empty array path component");
}

namespace {
void FiniteBytes(const std::string& bytes, Scalar scalar, bool little_endian) {
    if (scalar != Scalar::Float64) return;
    for (std::size_t i = 0; i < bytes.size(); i += 8) {
        std::uint64_t bits = 0;
        if (little_endian) {
            for (unsigned j = 0; j < 8; ++j)
                bits |= std::uint64_t(static_cast<unsigned char>(bytes[i + j])) << (8 * j);
        } else {
            std::memcpy(&bits, bytes.data() + i, 8);
        }
        double value;
        std::memcpy(&value, &bits, 8);
        Require(std::isfinite(value), "Nonfinite binary64 array value");
    }
}
bool LittleEndian() {
    const std::uint16_t x = 1;
    unsigned char bytes[2];
    std::memcpy(bytes, &x, 2);
    return bytes[0] == 1;
}

void SwapElements(std::string& data, std::size_t width) {
    for (std::size_t i = 0; i < data.size(); i += width)
        std::reverse(data.begin() + i, data.begin() + i + width);
}
} // namespace

void CheckDescriptor(const Descriptor& d, Limits cap) {
    CheckRelativeName(d.file);
    CheckHash(d.sha256);
    Require(d.bytes == ByteCount(d.layout, cap), "Array shape/byte identity mismatch");
}

std::filesystem::path CheckedPath(const std::filesystem::path& root, const std::string& name, bool existing) {
    CheckRelativeName(name);
    Require(std::filesystem::symlink_status(root).type() == std::filesystem::file_type::directory,
        "Array root is not a real directory");
    auto path = std::filesystem::canonical(root);
    const auto relative = std::filesystem::path(name);
    for (auto it = relative.begin(); it != relative.end(); ++it) {
        path /= *it;
        auto next = it;
        ++next;
        const auto type = std::filesystem::symlink_status(path).type();
        if (next != relative.end()) {
            Require(type == std::filesystem::file_type::directory, "Array parent is not a real directory");
        } else if (existing) {
            Require(type == std::filesystem::file_type::regular, "Array is not a regular file");
        } else {
            Require(type == std::filesystem::file_type::not_found, "Array destination already exists");
        }
    }
    return path;
}

std::string ReadBytes(const std::filesystem::path& root, const Descriptor& d, Limits cap) {
    CheckDescriptor(d, cap);
    const auto path = CheckedPath(root, d.file, true);
    Require(std::filesystem::file_size(path) == d.bytes, "Array file size mismatch");
    auto bytes = output::ReadBounded(path, d.bytes);
    Require(bytes.size() == d.bytes && Sha256(bytes) == d.sha256, "Array byte/hash identity mismatch");
    FiniteBytes(bytes, d.layout.scalar, true);
    return bytes;
}

Descriptor WriteBytes(const std::filesystem::path& root, const std::string& name, const Layout& l,
                      const std::string& bytes, Limits cap) {
    Require(bytes.size() == ByteCount(l, cap), "Array encoded extent mismatch");
    FiniteBytes(bytes, l.scalar, true);
    Descriptor d{name, l, bytes.size(), Sha256(bytes)};
    CheckDescriptor(d, cap);
    const auto path = CheckedPath(root, name, false);
    output::WriteBytes(path, bytes);
    return d;
}

namespace detail {
std::string Encode(const Layout& l, const void* data, std::size_t count, Scalar type, Limits cap) {
    const auto size = ByteCount(l, cap), width = ScalarBytes(type);
    Require(type == l.scalar && count == size / width && (data || !count), "Array typed input extent mismatch");
    std::string bytes(size, '\0');
    if (size) std::memcpy(bytes.data(), data, size);
    FiniteBytes(bytes, type, false);
    if (!LittleEndian()) SwapElements(bytes, width);
    return bytes;
}

std::string Decode(const Descriptor& d, const std::string& bytes, Scalar type, Limits cap) {
    CheckDescriptor(d, cap);
    Require(type == d.layout.scalar && bytes.size() == d.bytes && Sha256(bytes) == d.sha256,
        "Array typed byte/hash identity mismatch");
    FiniteBytes(bytes, type, true);
    auto native = bytes;
    if (!LittleEndian()) SwapElements(native, ScalarBytes(type));
    return native;
}
} // namespace detail
} // namespace crash::output::arrays
