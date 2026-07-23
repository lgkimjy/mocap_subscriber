#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace mocap_subscriber {

struct NpzArray {
    std::string name;
    std::string dtype;
    std::vector<size_t> shape;
    std::vector<uint8_t> data;
};

class NpzWriter {
public:
    [[nodiscard]] static NpzArray makeFloat64Array(const std::string& name, const std::vector<size_t>& shape,
                                                   const std::vector<double>& values);
    [[nodiscard]] static NpzArray makeInt64Array(const std::string& name, const std::vector<size_t>& shape,
                                                 const std::vector<int64_t>& values);
    [[nodiscard]] static NpzArray makeUInt8Array(const std::string& name, const std::vector<size_t>& shape,
                                                 const std::vector<uint8_t>& values);
    [[nodiscard]] static bool write(const std::string& path, const std::vector<NpzArray>& arrays);
};

}  // namespace mocap_subscriber
