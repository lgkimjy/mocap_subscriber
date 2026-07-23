# mocap_subscriber

C++ NatNet receiver for OptiTrack Motive rigid-body streaming.

- Library target: `mocap_natnet`
- Reader executable: `mocap_reader`
- Logger executable: `mocap_go2_box_logger`
- Public headers: `include/MoCapParser/`

## Dependencies

- CMake >= 3.16
- C++17 compiler
- Eigen3
- Linux UDP socket API / pthreads
- Python 3 + NumPy + Matplotlib, only for reading and plotting `.npz` logs

Ubuntu:

```bash
sudo apt update
sudo apt install cmake g++ libeigen3-dev python3-numpy python3-matplotlib
```

No OptiTrack NatNet SDK is required.

## Motive Setup

In Motive:

- `Streaming > NatNet > Enable`: ON
- `Rigid Bodies`: ON
- `Transmission Type`: match `use_multicast` in `config/natnet.conf`
- `Local Interface`: select the Motive PC interface connected to the Linux PC
- `Up Axis`: use `Z Up` if downstream code expects Z-up data

Default NatNet ports:

- Command: UDP `1510`
- Data: UDP `1511`

## Config

Default file: `config/natnet.conf`

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
- `rigid_body_id`: comma-separated Motive rigid-body IDs
- `rigid_body_name`: local labels matched by order with `rigid_body_id`

## Linux Firewall

NatNet uses UDP. If the handshake works but frame data does not arrive, allow UDP `1510` and `1511` on Linux.

For `ufw`:

```bash
sudo ufw status verbose
sudo ufw allow 1510/udp
sudo ufw allow 1511/udp
sudo ufw reload
```

For `firewalld`:

```bash
sudo firewall-cmd --permanent --add-port=1510/udp
sudo firewall-cmd --permanent --add-port=1511/udp
sudo firewall-cmd --reload
```

For raw `iptables` inspection:

```bash
sudo iptables -S
sudo iptables -L -n -v
```

Temporary `iptables` test rule:

```bash
sudo iptables -I INPUT -p udp --dport 1510 -j ACCEPT
sudo iptables -I INPUT -p udp --dport 1511 -j ACCEPT
```

Check whether packets reach Linux:

```bash
sudo tcpdump -ni any udp port 1510 or udp port 1511
```

## Build

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

## Live Reader

```bash
./build/mocap_reader config/natnet.conf --verbose
```

CLI override:

```bash
./build/mocap_reader config/natnet.conf --id 6,7 --multicast 0 --verbose
```

Verbose output rows are: name/id, status, position `[x y z]`, quaternion `[w x y z]`.

## Go2 / Box1 NPZ Logger

The logger writes every received NatNet frame for go2/box1 to `demo/go2-box/*.npz`.

```bash
./build/mocap_go2_box_logger config/natnet.conf
```

Stop with Ctrl+C. The NPZ file is written when the process exits.

Explicit ID override:

```bash
./build/mocap_go2_box_logger config/natnet.conf --box-id 6 --go2-id 7
```

An `.npz` file is a zip bundle of NumPy arrays. Inspect and plot a log with:

```bash
python3 scripts/read_go2_box_npz.py demo/go2-box/go2_box_YYYYMMDD_HHMMSS.npz --list-keys
```

If the file path is omitted, the script reads the latest `.npz` in `demo/go2-box`:

```bash
python3 scripts/read_go2_box_npz.py
```

The script saves a top-view figure next to the `.npz` file:

```text
demo/go2-box/go2_box_YYYYMMDD_HHMMSS_top_view.png
```

Plot options:

```bash
python3 scripts/read_go2_box_npz.py --axis-interval 20 --axis-scale 0.15
python3 scripts/read_go2_box_npz.py --no-plot
```

The plot uses the streamed Z-up convention: top view is the world `x-y` plane.
World `x` is red, world `y` is blue. Body axes are drawn every `--axis-interval` samples.

Saved arrays:

- `time_seconds`: logger time from start, shape `(N,)`
- `frame_number`: NatNet frame number, shape `(N,)`
- `go2_position`, `box1_position`: `[x y z]`, shape `(N, 3)`
- `go2_quaternion_wxyz`, `box1_quaternion_wxyz`: `[w x y z]`, shape `(N, 4)`
- `go2_valid`, `box1_valid`: `1` if that rigid body was present in the logged frame
- `go2_rigid_body_id`, `box1_rigid_body_id`: IDs used for logging

Missing rigid bodies are saved as `NaN` with valid flag `0`.

## C++ Usage

```cpp
#include "MoCapParser/NatNetClient.hpp"

mocap_subscriber::NatNetConfig cfg;
cfg.server_address = "192.168.0.189";
cfg.local_address = "0.0.0.0";
cfg.rigid_body_ids = {6, 7};
cfg.rigid_body_names = {"box1", "go2"};

mocap_subscriber::NatNetClient client(cfg);
client.start();

if (auto pose = client.latestPoseById(7)) {
    Eigen::Vector3d p = pose->position;
    Eigen::Quaterniond q = pose->orientation;
}

bool fresh = client.isPoseFresh(7, std::chrono::milliseconds(250));
```

Notes:

- The live frame stream is ID-based. `rigid_body_name` is only a local label.
- Quaternions use Eigen order: `(w, x, y, z)`.
- If Motive streams `Z Up`, do not apply another coordinate transform in code.
