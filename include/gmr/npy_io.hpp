#pragma once

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace gmr::npy {

struct Array2D {
    size_t rows = 0;
    size_t cols = 0;
    std::vector<double> values;
};

inline std::string trim(std::string value) {
    value.erase(value.begin(), std::find_if(value.begin(), value.end(),
        [](unsigned char c) { return !std::isspace(c); }));
    value.erase(std::find_if(value.rbegin(), value.rend(),
        [](unsigned char c) { return !std::isspace(c); }).base(), value.end());
    return value;
}

inline std::string field(const std::string& header, const std::string& name) {
    const size_t key = header.find("'" + name + "'");
    if (key == std::string::npos)
        throw std::runtime_error("NPY header is missing " + name);
    const size_t colon = header.find(':', key);
    if (colon == std::string::npos)
        throw std::runtime_error("invalid NPY header field: " + name);
    return header.substr(colon + 1);
}

inline Array2D load2D(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open NPY: " + path);

    char magic[6] = {};
    input.read(magic, sizeof(magic));
    if (!input || std::memcmp(magic, "\x93NUMPY", 6) != 0)
        throw std::runtime_error("invalid NPY magic: " + path);

    uint8_t major = 0, minor = 0;
    input.read(reinterpret_cast<char*>(&major), 1);
    input.read(reinterpret_cast<char*>(&minor), 1);
    (void)minor;
    uint32_t header_size = 0;
    if (major == 1) {
        uint8_t bytes[2] = {};
        input.read(reinterpret_cast<char*>(bytes), 2);
        header_size = static_cast<uint32_t>(bytes[0]) |
                      (static_cast<uint32_t>(bytes[1]) << 8U);
    } else if (major == 2 || major == 3) {
        uint8_t bytes[4] = {};
        input.read(reinterpret_cast<char*>(bytes), 4);
        header_size = static_cast<uint32_t>(bytes[0]) |
                      (static_cast<uint32_t>(bytes[1]) << 8U) |
                      (static_cast<uint32_t>(bytes[2]) << 16U) |
                      (static_cast<uint32_t>(bytes[3]) << 24U);
    } else {
        throw std::runtime_error("unsupported NPY version in " + path);
    }
    if (!input || header_size == 0 || header_size > 1024U * 1024U)
        throw std::runtime_error("invalid NPY header size in " + path);

    std::string header(header_size, '\0');
    input.read(header.data(), static_cast<std::streamsize>(header.size()));
    if (!input) throw std::runtime_error("truncated NPY header: " + path);

    const std::string descr_tail = field(header, "descr");
    const size_t quote1 = descr_tail.find_first_of("'\"");
    const size_t quote2 = quote1 == std::string::npos ? std::string::npos :
                          descr_tail.find(descr_tail[quote1], quote1 + 1);
    if (quote1 == std::string::npos || quote2 == std::string::npos)
        throw std::runtime_error("invalid NPY descr in " + path);
    const std::string descr = descr_tail.substr(quote1 + 1, quote2 - quote1 - 1);
    const bool f32 = descr == "<f4" || descr == "|f4" || descr == "=f4";
    const bool f64 = descr == "<f8" || descr == "|f8" || descr == "=f8";
    if (!f32 && !f64)
        throw std::runtime_error("NPY dtype must be little-endian float32/float64: " + descr);

    const std::string fortran_tail = field(header, "fortran_order");
    if (fortran_tail.find("True") < fortran_tail.find(','))
        throw std::runtime_error("Fortran-order NPY is not supported: " + path);

    const std::string shape_tail = field(header, "shape");
    const size_t open = shape_tail.find('(');
    const size_t close = shape_tail.find(')', open);
    if (open == std::string::npos || close == std::string::npos)
        throw std::runtime_error("invalid NPY shape in " + path);
    std::string shape = shape_tail.substr(open + 1, close - open - 1);
    std::vector<size_t> dims;
    std::stringstream shape_stream(shape);
    std::string token;
    while (std::getline(shape_stream, token, ',')) {
        token = trim(token);
        if (token.empty()) continue;
        const unsigned long long value = std::stoull(token);
        if (value > std::numeric_limits<size_t>::max())
            throw std::runtime_error("NPY dimension is too large: " + path);
        dims.push_back(static_cast<size_t>(value));
    }
    if (dims.size() == 1) dims.insert(dims.begin(), 1);
    if (dims.size() != 2 || dims[0] == 0 || dims[1] == 0)
        throw std::runtime_error("NPY must have shape (T,D): " + path);
    if (dims[0] > std::numeric_limits<size_t>::max() / dims[1])
        throw std::runtime_error("NPY element count overflow: " + path);

    Array2D result;
    result.rows = dims[0];
    result.cols = dims[1];
    const size_t count = result.rows * result.cols;
    result.values.resize(count);
    if (f32) {
        std::vector<float> raw(count);
        input.read(reinterpret_cast<char*>(raw.data()),
                   static_cast<std::streamsize>(raw.size() * sizeof(float)));
        if (!input) throw std::runtime_error("truncated NPY payload: " + path);
        std::transform(raw.begin(), raw.end(), result.values.begin(),
                       [](float value) { return static_cast<double>(value); });
    } else {
        input.read(reinterpret_cast<char*>(result.values.data()),
                   static_cast<std::streamsize>(result.values.size() * sizeof(double)));
        if (!input) throw std::runtime_error("truncated NPY payload: " + path);
    }
    return result;
}

inline void save2D(const std::string& path, const std::vector<double>& values,
                   size_t rows, size_t cols) {
    if (rows == 0 || cols == 0 || values.size() != rows * cols)
        throw std::runtime_error("invalid array dimensions for NPY output: " + path);
    std::ostringstream dict;
    dict << "{'descr': '<f8', 'fortran_order': False, 'shape': ("
         << rows << ", " << cols << "), }";
    std::string header = dict.str();
    const size_t preamble = 10;
    size_t padding = (16 - ((preamble + header.size() + 1) % 16)) % 16;
    header.append(padding, ' ');
    header.push_back('\n');
    if (header.size() > 65535)
        throw std::runtime_error("NPY v1 header is too large: " + path);

    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("cannot create NPY: " + path);
    output.write("\x93NUMPY", 6);
    const uint8_t version[2] = {1, 0};
    output.write(reinterpret_cast<const char*>(version), 2);
    const uint16_t length = static_cast<uint16_t>(header.size());
    const uint8_t length_bytes[2] = {
        static_cast<uint8_t>(length & 0xffU),
        static_cast<uint8_t>((length >> 8U) & 0xffU),
    };
    output.write(reinterpret_cast<const char*>(length_bytes), 2);
    output.write(header.data(), static_cast<std::streamsize>(header.size()));
    output.write(reinterpret_cast<const char*>(values.data()),
                 static_cast<std::streamsize>(values.size() * sizeof(double)));
    if (!output) throw std::runtime_error("failed writing NPY: " + path);
}

}  // namespace gmr::npy
