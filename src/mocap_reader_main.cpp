#include "MoCapParser/NatNetClient.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic<bool> keep_running{true};

void handleSignal(int)
{
    keep_running = false;
}

std::vector<int32_t> parseIds(std::string value)
{
    std::replace(value.begin(), value.end(), ',', ' ');
    std::stringstream ss(value);
    std::vector<int32_t> ids;
    int32_t id = 0;
    while (ss >> id) {
        ids.push_back(id);
    }
    return ids;
}

std::string rigidBodyLabel(const mocap_subscriber::NatNetConfig& config, size_t index)
{
    if (index < config.rigid_body_names.size()) {
        return config.rigid_body_names[index];
    }
    return "rb" + std::to_string(config.rigid_body_ids[index]);
}

constexpr const char* kGreen = "\033[32m";
constexpr const char* kYellow = "\033[33m";
constexpr const char* kReset = "\033[0m";
constexpr size_t kMinBodyColumnWidth = 54;

std::string formatScalar(double value)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(4) << value;
    return out.str();
}

std::string formatVector3(const Eigen::Vector3d& v)
{
    std::ostringstream out;
    out << "[" << formatScalar(v.x()) << " " << formatScalar(v.y()) << " " << formatScalar(v.z()) << "]";
    return out.str();
}

std::string formatQuaternionWxyz(const Eigen::Quaterniond& q)
{
    std::ostringstream out;
    out << "[" << formatScalar(q.w()) << " " << formatScalar(q.x()) << " " << formatScalar(q.y()) << " " << formatScalar(q.z()) << "]";
    return out.str();
}

size_t visibleLength(const std::string& text)
{
    size_t count = 0;
    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '\033') {
            while (i < text.size() && text[i] != 'm') {
                ++i;
            }
            continue;
        }
        ++count;
    }
    return count;
}

std::string padRightVisible(std::string text, size_t width)
{
    const size_t length = visibleLength(text);
    if (length < width) {
        text.append(width - length, ' ');
    }
    return text;
}

void appendAlignedRow(std::ostringstream& out, const std::vector<std::string>& cells, size_t width)
{
    for (size_t i = 0; i < cells.size(); ++i) {
        if (i > 0) {
            out << " | ";
        }
        out << padRightVisible(cells[i], width);
    }
}

void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " [config/natnet.conf] [--id ID1,ID2] [--multicast 0|1] [--verbose]\n";
}

}  // namespace

int main(int argc, char* argv[])
{
    using mocap_subscriber::NatNetClient;
    using mocap_subscriber::loadNatNetConfig;

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    std::string config_path{"config/natnet.conf"};
    bool config_set = false;
    mocap_subscriber::NatNetConfig config;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        }
        if ((arg == "--id" || arg == "--ids") && i + 1 < argc) {
            config.rigid_body_ids = parseIds(argv[++i]);
            continue;
        }
        if (arg == "--verbose" || arg == "-v") {
            verbose = true;
            continue;
        }
        if (arg == "--multicast" && i + 1 < argc) {
            config.use_multicast = std::string(argv[++i]) != "0";
            continue;
        }
        if (!config_set) {
            config_path = arg;
            config = loadNatNetConfig(config_path);
            config_set = true;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << '\n';
        printUsage(argv[0]);
        return 2;
    }

    if (!config_set) {
        config = loadNatNetConfig(config_path);
    }

    NatNetClient client(config);
    if (!client.start()) {
        return 1;
    }

    std::cout << std::fixed << std::setprecision(4);
    if (verbose) {
        std::cout << "Reading mocap pose. Press Ctrl+C to exit.\n";
    } else {
        std::cout << "NatNet client running. Press Ctrl+C to exit. Use --verbose to print poses.\n";
    }

    size_t printed_lines = 0;
    while (keep_running.load()) {
        if (!verbose) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        const auto stats = client.stats();
        std::ostringstream seen_ids;
        seen_ids << "[";
        for (size_t i = 0; i < stats.last_seen_rigid_body_id_count; ++i) {
            if (i > 0) seen_ids << ",";
            seen_ids << stats.last_seen_rigid_body_ids[i];
        }
        seen_ids << "]";

        std::vector<std::string> label_cells;
        std::vector<std::string> status_cells;
        std::vector<std::string> position_cells;
        std::vector<std::string> quaternion_cells;
        label_cells.reserve(config.rigid_body_ids.size());
        status_cells.reserve(config.rigid_body_ids.size());
        position_cells.reserve(config.rigid_body_ids.size());
        quaternion_cells.reserve(config.rigid_body_ids.size());
        size_t column_width = kMinBodyColumnWidth;

        for (size_t i = 0; i < config.rigid_body_ids.size(); ++i) {
            const int32_t id = config.rigid_body_ids[i];
            const auto pose = client.latestPoseById(id);

            std::ostringstream label;
            label << rigidBodyLabel(config, i) << "(id=" << id << ")";
            label_cells.push_back(label.str());

            if (pose) {
                const bool fresh = client.isPoseFresh(id, std::chrono::milliseconds(250));
                status_cells.push_back(std::string(kGreen) + "matched" + kReset + " fresh=" + (fresh ? "yes" : "no"));
                position_cells.push_back(formatVector3(pose->position));
                quaternion_cells.push_back(formatQuaternionWxyz(pose->orientation));
            } else {
                status_cells.push_back(std::string(kYellow) + "waiting" + kReset);
                position_cells.push_back("[-- -- --]");
                quaternion_cells.push_back("[-- -- -- --]");
            }
        }

        for (const auto* cells : {&label_cells, &status_cells, &position_cells, &quaternion_cells}) {
            for (const auto& cell : *cells) {
                column_width = std::max(column_width, visibleLength(cell));
            }
        }

        std::ostringstream frame_line;
        frame_line << "frame=" << std::setw(8) << stats.last_frame_number << " packets=" << stats.packets_received
                   << " frames=" << stats.frames_received << " seen_ids=" << seen_ids.str();

        std::ostringstream names_line;
        appendAlignedRow(names_line, label_cells, column_width);
        std::ostringstream status_line;
        appendAlignedRow(status_line, status_cells, column_width);
        std::ostringstream position_line;
        appendAlignedRow(position_line, position_cells, column_width);
        std::ostringstream quaternion_line;
        appendAlignedRow(quaternion_line, quaternion_cells, column_width);

        const std::array<std::string, 5> block{
            frame_line.str(), names_line.str(), status_line.str(), position_line.str(), quaternion_line.str()};

        if (printed_lines > 0) {
            std::cout << "\033[" << printed_lines << "A";
        }
        for (const auto& row : block) {
            std::cout << "\033[2K" << row << '\n';
        }
        printed_lines = block.size();
        std::cout << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    client.stop();
    std::cout << "\nDone.\n";
    return 0;
}
