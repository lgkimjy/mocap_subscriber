# mocap_subscriber

C++ NatNet receiver for OptiTrack Motive rigid-body streaming.

- Library target: `mocap_natnet`
- Public header: `include/MoCapParser/NatNetClient.hpp`
- Implementation: `include/MoCapParser/NatNetClient.cpp`
- Example executable: `src/mocap_reader_main.cpp`

## Dependencies

- CMake >= 3.16
- C++17 compiler
- Eigen3
- Linux UDP socket API / pthreads

Ubuntu:

```bash
sudo apt update
sudo apt install cmake g++ libeigen3-dev
```

No OptiTrack NatNet SDK is required. The client parses NatNet UDP packets directly.

## Motive Setup

In Motive, enable NatNet streaming:

- `Streaming > NatNet > Enable`: ON
- `Rigid Bodies`: ON
- `Transmission Type`: match `use_multicast` in `config/natnet.conf`
- `Local Interface`: select the Motive PC network interface connected to the Linux PC
- `Up Axis`: use `Z Up` if downstream robotics code expects Z-up data

Default NatNet ports:

- Command: UDP `1510`
- Data: UDP `1511`

## Config

The executable reads `config/natnet.conf` by default.

```conf
use_multicast=false
server_address=192.168.0.189
local_address=192.168.0.28
command_port=1510
data_port=1511
rigid_body_id=6,7
rigid_body_name="box1","go2"
```

Fields:

- `server_address`: Motive Windows PC IP
- `local_address`: Linux PC IP on the same network. Use `0.0.0.0` to listen on all interfaces
- `use_multicast`: `false` for Unicast, `true` for Multicast
- `rigid_body_id`: comma-separated Motive rigid-body IDs to read
- `rigid_body_name`: local labels matched by order with `rigid_body_id`

## Linux Firewall

NatNet uses UDP. If Motive handshakes but frame data does not arrive, allow UDP `1510` and `1511` on the Linux PC.

Check whether `ufw` is active:

```bash
sudo ufw status verbose
```

Allow NatNet UDP ports with `ufw`:

```bash
sudo ufw allow 1510/udp
sudo ufw allow 1511/udp
sudo ufw reload
```

If the system uses `firewalld`:

```bash
sudo firewall-cmd --permanent --add-port=1510/udp
sudo firewall-cmd --permanent --add-port=1511/udp
sudo firewall-cmd --reload
```

If neither firewall manager is used, inspect raw rules:

```bash
sudo iptables -S
sudo iptables -L -n -v
```

Temporary test rule with `iptables`:

```bash
sudo iptables -I INPUT -p udp --dport 1510 -j ACCEPT
sudo iptables -I INPUT -p udp --dport 1511 -j ACCEPT
```

To confirm packets are reaching Linux:

```bash
sudo tcpdump -ni any udp port 1510 or udp port 1511
```

## Build / Run

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/mocap_reader config/natnet.conf --verbose
```

CLI overrides:

```bash
./build/mocap_reader config/natnet.conf --id 4,5 --multicast 0 --verbose
```

## C++ Usage

```cpp
#include "MoCapParser/NatNetClient.hpp"

mocap_subscriber::NatNetConfig cfg;
cfg.server_address = "192.168.0.189";
cfg.local_address = "0.0.0.0";
cfg.rigid_body_ids = {4, 5};
cfg.rigid_body_names = {"robot", "box1"};

mocap_subscriber::NatNetClient client(cfg);
client.start();

if (auto pose = client.latestPoseById(4)) {
    Eigen::Vector3d p = pose->position;
    Eigen::Quaterniond q = pose->orientation;  // w, x, y, z
}

bool fresh = client.isPoseFresh(4, std::chrono::milliseconds(250));
```

Callback usage:

```cpp
client.setPoseCallback([](const mocap_subscriber::Pose& pose) {
    // Copy pose into your controller, logger, estimator, etc.
});
```

## Runtime Output

Default mode prints connection messages only. Use `--verbose` for live diagnostics:

```text
frame= 2537416 packets=6858 frames=6857 seen_ids=[7]
box1(id=6)                                             | go2(id=7)
waiting                                                | matched fresh=yes
[-- -- --]                                             | [1.0920 0.2949 0.0688]
[-- -- -- --]                                          | [0.9962 0.0000 0.0000 -0.0872]
```

Rows are: name/id, status, position `[x y z]`, quaternion `[w x y z]`.

Useful diagnostics:

- `packets=0`: no NatNet UDP packets are reaching this process
- `packets>0, frames=0`: packets arrive, but frame parsing is failing
- `seen_ids=[...]`: rigid-body IDs observed in recent frames
- If the desired ID is missing from `seen_ids`, update `rigid_body_id`
- The client keeps the last valid pose; it does not reset stale data to zero
- Use `isPoseFresh(id, max_age)` before using a pose in control code

## Notes

- The live frame stream is ID-based. `rigid_body_name` is only a local display label.
- Quaternions use Eigen order: `(w, x, y, z)`.
- If Motive streams `Z Up`, do not apply another coordinate transform in code.
