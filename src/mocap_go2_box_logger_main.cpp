#include "MoCapParser/NatNetClient.hpp"
#include "MoCapParser/NpzWriter.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
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

struct Options {
    std::string config_path{"config/natnet.conf"};
    std::string output_dir{"demo/go2-box"};
    std::string go2_name{"go2"};
    std::string box_name{"box1"};
    std::optional<int32_t> go2_id;
    std::optional<int32_t> box_id;
    bool verbose{false};
    bool help_requested{false};
};

struct LogBuffer {
    std::vector<double> time_seconds;
    std::vector<int64_t> frame_number;
    std::vector<uint8_t> go2_valid;
    std::vector<uint8_t> box1_valid;
    std::vector<double> go2_position;
    std::vector<double> go2_quaternion_wxyz;
    std::vector<double> box1_position;
    std::vector<double> box1_quaternion_wxyz;
};

struct FrameAccumulator {
    uint32_t frame_number{0};
    double time_seconds{0.0};
    std::optional<mocap_subscriber::Pose> go2_pose;
    std::optional<mocap_subscriber::Pose> box_pose;
};

void printUsage(const char* argv0)
{
    std::cout << "Usage: " << argv0 << " [config/natnet.conf] [--output-dir demo/go2-box]\n"
              << "       [--go2-id ID] [--box-id ID] [--go2-name NAME] [--box-name NAME] [--verbose]\n";
}

std::optional<int32_t> findRigidBodyIdByName(const mocap_subscriber::NatNetConfig& config, const std::string& name)
{
    const size_t count = std::min(config.rigid_body_ids.size(), config.rigid_body_names.size());
    for (size_t i = 0; i < count; ++i) {
        if (config.rigid_body_names[i] == name) {
            return config.rigid_body_ids[i];
        }
    }
    return std::nullopt;
}

std::string timestampForFileName()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_time{};
    localtime_r(&now, &local_time);

    std::ostringstream out;
    out << std::put_time(&local_time, "%Y%m%d_%H%M%S");
    return out.str();
}

void appendPose(std::vector<double>& position, std::vector<double>& quaternion, const std::optional<mocap_subscriber::Pose>& pose,
                bool valid)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (!valid || !pose) {
        position.insert(position.end(), {nan, nan, nan});
        quaternion.insert(quaternion.end(), {nan, nan, nan, nan});
        return;
    }

    position.insert(position.end(), {pose->position.x(), pose->position.y(), pose->position.z()});
    quaternion.insert(quaternion.end(), {pose->orientation.w(), pose->orientation.x(), pose->orientation.y(), pose->orientation.z()});
}

void appendFrame(LogBuffer& log, const FrameAccumulator& frame)
{
    if (frame.frame_number == 0 || (!frame.go2_pose && !frame.box_pose)) {
        return;
    }

    const bool go2_valid = frame.go2_pose.has_value();
    const bool box_valid = frame.box_pose.has_value();
    log.time_seconds.push_back(frame.time_seconds);
    log.frame_number.push_back(frame.frame_number);
    log.go2_valid.push_back(static_cast<uint8_t>(go2_valid));
    log.box1_valid.push_back(static_cast<uint8_t>(box_valid));
    appendPose(log.go2_position, log.go2_quaternion_wxyz, frame.go2_pose, go2_valid);
    appendPose(log.box1_position, log.box1_quaternion_wxyz, frame.box_pose, box_valid);
}

bool parseArgs(int argc, char* argv[], Options& options)
{
    bool config_set = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            options.help_requested = true;
            return false;
        }
        if (arg == "--output-dir" && i + 1 < argc) {
            options.output_dir = argv[++i];
            continue;
        }
        if (arg == "--go2-id" && i + 1 < argc) {
            options.go2_id = static_cast<int32_t>(std::stoi(argv[++i]));
            continue;
        }
        if (arg == "--box-id" && i + 1 < argc) {
            options.box_id = static_cast<int32_t>(std::stoi(argv[++i]));
            continue;
        }
        if (arg == "--go2-name" && i + 1 < argc) {
            options.go2_name = argv[++i];
            continue;
        }
        if (arg == "--box-name" && i + 1 < argc) {
            options.box_name = argv[++i];
            continue;
        }
        if (arg == "--verbose" || arg == "-v") {
            options.verbose = true;
            continue;
        }
        if (!config_set) {
            options.config_path = arg;
            config_set = true;
            continue;
        }
        std::cerr << "Unknown argument: " << arg << '\n';
        printUsage(argv[0]);
        return false;
    }
    return true;
}

bool writeLog(const std::string& output_path, const LogBuffer& log, int32_t go2_id, int32_t box_id)
{
    using mocap_subscriber::NpzWriter;

    const size_t n = log.time_seconds.size();
    const std::vector<size_t> samples_shape{n};
    const std::vector<size_t> position_shape{n, 3};
    const std::vector<size_t> quaternion_shape{n, 4};

    return NpzWriter::write(output_path, {
        NpzWriter::makeFloat64Array("time_seconds", samples_shape, log.time_seconds),
        NpzWriter::makeInt64Array("frame_number", samples_shape, log.frame_number),
        NpzWriter::makeInt64Array("go2_rigid_body_id", {1}, std::vector<int64_t>{go2_id}),
        NpzWriter::makeInt64Array("box1_rigid_body_id", {1}, std::vector<int64_t>{box_id}),
        NpzWriter::makeUInt8Array("go2_valid", samples_shape, log.go2_valid),
        NpzWriter::makeUInt8Array("box1_valid", samples_shape, log.box1_valid),
        NpzWriter::makeFloat64Array("go2_position", position_shape, log.go2_position),
        NpzWriter::makeFloat64Array("go2_quaternion_wxyz", quaternion_shape, log.go2_quaternion_wxyz),
        NpzWriter::makeFloat64Array("box1_position", position_shape, log.box1_position),
        NpzWriter::makeFloat64Array("box1_quaternion_wxyz", quaternion_shape, log.box1_quaternion_wxyz),
    });
}

}  // namespace

int main(int argc, char* argv[])
{
    using mocap_subscriber::NatNetClient;
    using mocap_subscriber::loadNatNetConfig;

    Options options;
    if (!parseArgs(argc, argv, options)) {
        return options.help_requested ? 0 : 2;
    }

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    auto config = loadNatNetConfig(options.config_path);
    if (!options.go2_id) {
        options.go2_id = findRigidBodyIdByName(config, options.go2_name);
    }
    if (!options.box_id) {
        options.box_id = findRigidBodyIdByName(config, options.box_name);
    }
    if (!options.box_id && !config.rigid_body_ids.empty()) {
        options.box_id = config.rigid_body_ids[0];
    }
    if (!options.go2_id && config.rigid_body_ids.size() >= 2) {
        options.go2_id = config.rigid_body_ids[1];
    }
    if (!options.go2_id || !options.box_id) {
        std::cerr << "Could not resolve go2/box1 rigid-body IDs. Set rigid_body_name in config or pass --go2-id/--box-id.\n";
        return 2;
    }

    config.rigid_body_ids = {*options.box_id, *options.go2_id};
    config.rigid_body_names = {options.box_name, options.go2_name};

    std::filesystem::create_directories(options.output_dir);
    const std::filesystem::path output_path = std::filesystem::path(options.output_dir) /
                                             ("go2_box_" + timestampForFileName() + ".npz");

    LogBuffer log;
    FrameAccumulator active_frame;
    std::mutex log_mutex;
    const auto start_time = std::chrono::steady_clock::now();

    NatNetClient client(config);
    client.setPoseCallback([&](const mocap_subscriber::Pose& pose) {
        if (pose.rigid_body_id != *options.go2_id && pose.rigid_body_id != *options.box_id) {
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double time_seconds = std::chrono::duration<double>(now - start_time).count();
        std::lock_guard<std::mutex> lock(log_mutex);

        if (active_frame.frame_number != 0 && active_frame.frame_number != pose.frame_number) {
            appendFrame(log, active_frame);
            if (options.verbose && log.time_seconds.size() % 100 == 0) {
                std::cout << "samples=" << log.time_seconds.size() << " frame=" << active_frame.frame_number << '\n';
            }
            active_frame = {};
        }

        if (active_frame.frame_number == 0) {
            active_frame.frame_number = pose.frame_number;
            active_frame.time_seconds = time_seconds;
        }

        if (pose.rigid_body_id == *options.go2_id) {
            active_frame.go2_pose = pose;
        } else if (pose.rigid_body_id == *options.box_id) {
            active_frame.box_pose = pose;
        }
    });

    if (!client.start()) {
        return 1;
    }

    std::cout << "Logging every received frame for go2(id=" << *options.go2_id << ") and box1(id=" << *options.box_id
              << ") to " << output_path.string() << '\n';
    std::cout << "Press Ctrl+C to stop and write NPZ.\n";

    while (keep_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    client.stop();
    {
        std::lock_guard<std::mutex> lock(log_mutex);
        appendFrame(log, active_frame);
    }

    if (!writeLog(output_path.string(), log, *options.go2_id, *options.box_id)) {
        std::cerr << "Failed to write NPZ: " << output_path.string() << '\n';
        return 1;
    }

    std::cout << "Wrote " << log.time_seconds.size() << " samples: " << output_path.string() << '\n';
    return 0;
}
