#include "sensors/camera/image_frame_writer.hpp"

#include <cstdint>
#include <fstream>
#include <vector>

namespace hako::robots::sensor::camera
{
namespace
{
uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t size)
{
    static uint32_t table[256] {};
    static bool initialized = false;
    if (!initialized) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) {
                c = (c & 1U) ? (0xedb88320U ^ (c >> 1U)) : (c >> 1U);
            }
            table[i] = c;
        }
        initialized = true;
    }

    crc = crc ^ 0xffffffffU;
    for (size_t i = 0; i < size; ++i) {
        crc = table[(crc ^ data[i]) & 0xffU] ^ (crc >> 8U);
    }
    return crc ^ 0xffffffffU;
}

uint32_t adler32(const std::vector<uint8_t>& data)
{
    constexpr uint32_t kMod = 65521U;
    uint32_t a = 1U;
    uint32_t b = 0U;
    for (uint8_t byte : data) {
        a = (a + byte) % kMod;
        b = (b + a) % kMod;
    }
    return (b << 16U) | a;
}

void append_be32(std::vector<uint8_t>& out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>((value >> 24U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 16U) & 0xffU));
    out.push_back(static_cast<uint8_t>((value >> 8U) & 0xffU));
    out.push_back(static_cast<uint8_t>(value & 0xffU));
}

void append_chunk(
    std::vector<uint8_t>& png,
    const char type[4],
    const std::vector<uint8_t>& payload)
{
    append_be32(png, static_cast<uint32_t>(payload.size()));
    const size_t type_offset = png.size();
    png.insert(png.end(), type, type + 4);
    png.insert(png.end(), payload.begin(), payload.end());

    const uint32_t crc = crc32_update(
        0U,
        png.data() + type_offset,
        4 + payload.size());
    append_be32(png, crc);
}

std::vector<uint8_t> zlib_store(const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> out;
    out.push_back(0x78);
    out.push_back(0x01);

    size_t offset = 0;
    while (offset < data.size()) {
        const size_t remaining = data.size() - offset;
        const uint16_t block_size = static_cast<uint16_t>(
            remaining > 65535U ? 65535U : remaining);
        const bool final_block = (offset + block_size) == data.size();

        out.push_back(final_block ? 0x01 : 0x00);
        out.push_back(static_cast<uint8_t>(block_size & 0xffU));
        out.push_back(static_cast<uint8_t>((block_size >> 8U) & 0xffU));
        const uint16_t nlen = static_cast<uint16_t>(~block_size);
        out.push_back(static_cast<uint8_t>(nlen & 0xffU));
        out.push_back(static_cast<uint8_t>((nlen >> 8U) & 0xffU));
        out.insert(out.end(), data.begin() + static_cast<long>(offset),
                   data.begin() + static_cast<long>(offset + block_size));
        offset += block_size;
    }

    append_be32(out, adler32(data));
    return out;
}
}

bool WriteImageFrameToPng(
    const ImageFrame& frame,
    const std::filesystem::path& path)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.channels != 3 ||
        frame.format != "R8G8B8" ||
        frame.data.size() != static_cast<size_t>(frame.width * frame.height * 3)) {
        return false;
    }

    if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
    }

    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>((frame.width * 3 + 1) * frame.height));
    for (int y = 0; y < frame.height; ++y) {
        raw.push_back(0);
        const size_t row_offset = static_cast<size_t>(y * frame.width * 3);
        raw.insert(
            raw.end(),
            frame.data.begin() + static_cast<long>(row_offset),
            frame.data.begin() + static_cast<long>(row_offset + frame.width * 3));
    }

    std::vector<uint8_t> png {
        0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'
    };

    std::vector<uint8_t> ihdr;
    append_be32(ihdr, static_cast<uint32_t>(frame.width));
    append_be32(ihdr, static_cast<uint32_t>(frame.height));
    ihdr.push_back(8);
    ihdr.push_back(2);
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(0);
    append_chunk(png, "IHDR", ihdr);

    append_chunk(png, "IDAT", zlib_store(raw));
    append_chunk(png, "IEND", {});

    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) {
        return false;
    }
    ofs.write(reinterpret_cast<const char*>(png.data()), static_cast<std::streamsize>(png.size()));
    return ofs.good();
}
}
