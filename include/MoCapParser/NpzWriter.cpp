#include "MoCapParser/NpzWriter.hpp"

#include <cstring>
#include <fstream>
#include <sstream>

namespace mocap_subscriber {
namespace {

uint32_t crc32(const uint8_t* data, size_t size)
{
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < size; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1u) ^ (0xedb88320u & static_cast<uint32_t>(-(crc & 1u)));
        }
    }
    return ~crc;
}

void appendU16(std::vector<uint8_t>& out, uint16_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xffu));
    out.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
}

void writeU16(std::ofstream& out, uint16_t value)
{
    char bytes[2]{static_cast<char>(value & 0xffu), static_cast<char>((value >> 8u) & 0xffu)};
    out.write(bytes, sizeof(bytes));
}

void writeU32(std::ofstream& out, uint32_t value)
{
    writeU16(out, static_cast<uint16_t>(value & 0xffffu));
    writeU16(out, static_cast<uint16_t>((value >> 16u) & 0xffffu));
}

size_t elementCount(const std::vector<size_t>& shape)
{
    size_t count = 1;
    for (const size_t dim : shape) {
        count *= dim;
    }
    return count;
}

size_t elementSize(const std::string& dtype)
{
    if (dtype == "<f8" || dtype == "<i8") {
        return 8;
    }
    if (dtype == "|u1") {
        return 1;
    }
    return 0;
}

std::string shapeString(const std::vector<size_t>& shape)
{
    std::ostringstream out;
    out << '(';
    for (size_t i = 0; i < shape.size(); ++i) {
        if (i > 0) {
            out << ", ";
        }
        out << shape[i];
    }
    if (shape.size() == 1) {
        out << ',';
    }
    out << ')';
    return out.str();
}

std::vector<uint8_t> makeNpyPayload(const NpzArray& array)
{
    std::string header = "{'descr': '" + array.dtype + "', 'fortran_order': False, 'shape': " + shapeString(array.shape) + ", }";
    const size_t preamble_size = 10;
    const size_t padded_size = preamble_size + header.size() + 1;
    const size_t padding = (16 - (padded_size % 16)) % 16;
    header.append(padding, ' ');
    header.push_back('\n');

    std::vector<uint8_t> out;
    out.reserve(preamble_size + header.size() + array.data.size());
    out.push_back(0x93);
    out.insert(out.end(), {'N', 'U', 'M', 'P', 'Y'});
    out.push_back(1);
    out.push_back(0);
    appendU16(out, static_cast<uint16_t>(header.size()));
    out.insert(out.end(), header.begin(), header.end());
    out.insert(out.end(), array.data.begin(), array.data.end());
    return out;
}

template <typename T>
std::vector<uint8_t> copyBytes(const std::vector<T>& values)
{
    std::vector<uint8_t> bytes(sizeof(T) * values.size());
    if (!values.empty()) {
        std::memcpy(bytes.data(), values.data(), bytes.size());
    }
    return bytes;
}

struct ZipEntry {
    std::string file_name;
    std::vector<uint8_t> payload;
    uint32_t crc{0};
    uint32_t local_header_offset{0};
};

bool writeLocalFile(std::ofstream& out, ZipEntry& entry)
{
    const auto offset = out.tellp();
    if (offset < 0 || entry.file_name.size() > 0xffffu || entry.payload.size() > 0xffffffffu) {
        return false;
    }

    entry.local_header_offset = static_cast<uint32_t>(offset);
    entry.crc = crc32(entry.payload.data(), entry.payload.size());

    writeU32(out, 0x04034b50u);
    writeU16(out, 20);
    writeU16(out, 0);
    writeU16(out, 0);
    writeU16(out, 0);
    writeU16(out, 0);
    writeU32(out, entry.crc);
    writeU32(out, static_cast<uint32_t>(entry.payload.size()));
    writeU32(out, static_cast<uint32_t>(entry.payload.size()));
    writeU16(out, static_cast<uint16_t>(entry.file_name.size()));
    writeU16(out, 0);
    out.write(entry.file_name.data(), static_cast<std::streamsize>(entry.file_name.size()));
    out.write(reinterpret_cast<const char*>(entry.payload.data()), static_cast<std::streamsize>(entry.payload.size()));
    return static_cast<bool>(out);
}

bool writeCentralDirectory(std::ofstream& out, const std::vector<ZipEntry>& entries, uint32_t central_dir_offset)
{
    for (const auto& entry : entries) {
        writeU32(out, 0x02014b50u);
        writeU16(out, 20);
        writeU16(out, 20);
        writeU16(out, 0);
        writeU16(out, 0);
        writeU16(out, 0);
        writeU16(out, 0);
        writeU32(out, entry.crc);
        writeU32(out, static_cast<uint32_t>(entry.payload.size()));
        writeU32(out, static_cast<uint32_t>(entry.payload.size()));
        writeU16(out, static_cast<uint16_t>(entry.file_name.size()));
        writeU16(out, 0);
        writeU16(out, 0);
        writeU16(out, 0);
        writeU16(out, 0);
        writeU32(out, 0);
        writeU32(out, entry.local_header_offset);
        out.write(entry.file_name.data(), static_cast<std::streamsize>(entry.file_name.size()));
    }

    const auto end_offset = out.tellp();
    if (end_offset < 0) {
        return false;
    }
    const uint32_t central_dir_size = static_cast<uint32_t>(end_offset) - central_dir_offset;

    writeU32(out, 0x06054b50u);
    writeU16(out, 0);
    writeU16(out, 0);
    writeU16(out, static_cast<uint16_t>(entries.size()));
    writeU16(out, static_cast<uint16_t>(entries.size()));
    writeU32(out, central_dir_size);
    writeU32(out, central_dir_offset);
    writeU16(out, 0);
    return static_cast<bool>(out);
}

}  // namespace

NpzArray NpzWriter::makeFloat64Array(const std::string& name, const std::vector<size_t>& shape,
                                     const std::vector<double>& values)
{
    NpzArray array{name, "<f8", shape, copyBytes(values)};
    return array;
}

NpzArray NpzWriter::makeInt64Array(const std::string& name, const std::vector<size_t>& shape,
                                   const std::vector<int64_t>& values)
{
    NpzArray array{name, "<i8", shape, copyBytes(values)};
    return array;
}

NpzArray NpzWriter::makeUInt8Array(const std::string& name, const std::vector<size_t>& shape,
                                   const std::vector<uint8_t>& values)
{
    NpzArray array{name, "|u1", shape, copyBytes(values)};
    return array;
}

bool NpzWriter::write(const std::string& path, const std::vector<NpzArray>& arrays)
{
    std::vector<ZipEntry> entries;
    entries.reserve(arrays.size());

    for (const auto& array : arrays) {
        const size_t item_size = elementSize(array.dtype);
        if (array.name.empty() || item_size == 0 || array.data.size() != elementCount(array.shape) * item_size) {
            return false;
        }
        entries.push_back(ZipEntry{array.name + ".npy", makeNpyPayload(array)});
    }

    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return false;
    }

    for (auto& entry : entries) {
        if (!writeLocalFile(out, entry)) {
            return false;
        }
    }

    const auto central_dir_offset = out.tellp();
    if (central_dir_offset < 0 || central_dir_offset > 0xffffffffu) {
        return false;
    }
    return writeCentralDirectory(out, entries, static_cast<uint32_t>(central_dir_offset));
}

}  // namespace mocap_subscriber
