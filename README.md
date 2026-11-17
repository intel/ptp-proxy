# PTP Proxy Kernel Module (`ptp_proxy.ko`)

`ptp_proxy` is a Linux kernel module that provides a **userspace-backed virtual PTP Hardware Clock (`/dev/ptpN`)** and virtual proxy network interface (`ptpproxyN`). It acts as a bridge between unmodified Linux PTP applications (`ptp4l`, `phc2sys`, `chrony`) and hardware clocks managed by userspace frameworks like **DPDK**, **VFIO**, or custom hardware/software clock daemons.

---

## Key Features

- **Zero-Modification PTP Applications**: Standard `ptp4l`, `phc2sys`, and `chrony` daemons run without patch or configuration changes.
- **Hardware Timestamp Preservation**: Hardware RX/TX timestamps captured by DPDK/userspace drivers are fed directly into the kernel network stack and `MSG_ERRQUEUE`.
- **Multi-Domain Virtual Clock (vclock) Support**: Full support for kernel `n_vclocks` created on top of `/dev/ptp0`, with dual-domain timestamping (`SOF_TIMESTAMPING_BIND_PHC`).
- **Discipliner Ownership Locking**: Automatic single-process discipliner lock (`tgid` protection) prevents conflicting frequency adjustments when multiple PTP daemons run concurrently.
- **Out-of-Order TX Correlation**: High-throughput multi-slot TX timestamp matching via collision-safe 32-bit cookie correlation.
- **Response Correlation and Validation**: Transaction IDs reject unsolicited, duplicate, stale, and mismatched command responses; returned timestamps are range-checked.
- **Per-Instance Diagnostics**: Debugfs counters expose command activity, validation failures, timestamp-cookie events, queue drops, and ownership events.
- **Portable Architecture**: Non-sleeping atomic-context caching workaround for `getcycles64`, free of architecture-specific assembly or x86 assumptions.

---

## Repository Structure

```text
kernel/ptp_proxy/
├── ptp_proxy.c          # Driver implementation (PHC ops, netdev, UAPI)
├── ptp_proxy_uapi.h     # Userspace API header (commands, structs, ioctls)
├── Makefile             # Out-of-tree Kbuild Makefile
├── 99-ptp-proxy.rules   # udev permissions for the control device
├── README.md            # Quickstart and usage guide (this file)
├── SECURITY.md          # Intel vulnerability reporting policy
└── DESIGN.md            # Detailed upstream architecture & subsystem spec
```

See [SECURITY.md](SECURITY.md) for reporting security vulnerabilities in this
module to Intel PSIRT.

---

## Building and Installation

### Out-of-Tree Build (Standalone)

To compile `ptp_proxy.ko` against your running kernel headers:

```bash
cd kernel/ptp_proxy
make
```

To install and load the module:

```bash
sudo insmod ptp_proxy.ko
```

Verify loading in kernel logs:

```bash
dmesg | grep ptp_proxy
# Expected output:
# ptp_proxy: loaded v1.0 (/dev/ptp_proxy)
```

Verify character device availability:

```bash
ls -l /dev/ptp_proxy
# Access is deployment-specific; the recommended policy is root:ptp-proxy 0660.
# Instance creation additionally requires CAP_SYS_TIME.
```

---

## Module Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cmd_timeout_ms` | `int` | `100` | Timeout in milliseconds waiting for userspace to respond to synchronous PTP commands (`GETTIME`, `SETTIME`, `ADJTIME`, `ADJFINE`). |

Example loading with custom timeout:

```bash
sudo insmod ptp_proxy.ko cmd_timeout_ms=200
```

---

## User Interface & Device Nodes

The module exposes a control device and creates anonymous file descriptors for instances:

### 1. Control Device: `/dev/ptp_proxy`

Opening `/dev/ptp_proxy` and issuing the `PTP_PROXY_IOC_CREATE` ioctl registers a new virtual PHC `/dev/ptpN` and returns an anonymous **Instance File Descriptor (`inst_fd`)**.

### 2. Instance File Descriptor (`inst_fd`)

The `inst_fd` manages the PTP clock commands:
- **`read(inst_fd)`**: Reads incoming kernel PTP requests (`struct ptp_proxy_request`).
- **`write(inst_fd)`**: Writes responses (`struct ptp_proxy_response`) back to the kernel.
- **`poll(inst_fd)`**: Returns `POLLIN` when a kernel command or notification is ready.
- **`close(inst_fd)`**: Destroys the virtual PTP clock `/dev/ptpN` and cleans up resources.

### 3. Netdev & Packet File Descriptor (`pkt_fd`)

Issuing `PTP_PROXY_IOC_CREATE_NETDEV` on `inst_fd` creates a virtual Ethernet device (`ptpproxy0`, `ptpproxy1`, ...) and returns an anonymous **Packet File Descriptor (`pkt_fd`)**:
- **`read(pkt_fd)`**: Dequeues outbound PTP frames sent by `ptp4l` via `ndo_start_xmit` along with inline metadata headers (`struct ptp_proxy_ts_hdr`).
- **`write(pkt_fd)`**: Injects inbound PTP frames into the kernel network stack with hardware timestamps (`PTP_PROXY_TS_VALID`), or returns hardware TX departure timestamps to `ptp4l` via socket `MSG_ERRQUEUE` (`PTP_PROXY_TS_TX_ERRQUEUE`).

---

## Quickstart Testing with `ptp4l`

### Step 1: Start Userspace Backend (e.g., DPDK Relay / Test App)

The userspace backend connects to `/dev/ptp_proxy`, creates the PHC `/dev/ptp0` and netdev `ptpproxy0`, and handles hardware timestamps from the NIC.

### Step 2: Verify Created Devices

```bash
# Check created interface
ip link show ptpproxy0

# Verify hardware timestamping capabilities reported by ethtool
ethtool -T ptpproxy0
```

### Step 3: Run `ptp4l`

Run standard `ptp4l` against `ptpproxy0`:

```bash
sudo ptp4l -i ptpproxy0 -m -2 -s -H --summary_interval 1
```

### Diagnostics

When debugfs is mounted, each live instance exposes counters at:

```bash
sudo cat /sys/kernel/debug/ptp_proxy/ptpN/stats
```

The output includes per-command activity, command timeouts and rejected
responses, invalid timestamp responses, TX-cookie allocation/expiry events,
packet queue drops, notification replacement, discipliner events, and the
current pending TX depth (`pending_tx_current`).

---

## Multi-Domain Virtual Clocks (`vclocks`)

To create virtual clocks on top of the base `/dev/ptp0` for multi-domain PTP:

```bash
# Create 2 virtual clocks on base PHC /dev/ptp0
echo 2 | sudo tee /sys/class/ptp/ptp0/n_vclocks

# Verify created vclocks (/dev/ptp1, /dev/ptp2)
ls -l /dev/ptp*
```

Run independent `ptp4l` instances referencing the vclocks:

```bash
# Domain 0 on vclock /dev/ptp1
sudo ptp4l -i ptpproxy0 -p /dev/ptp1 -m -2 -s -H --domainNumber=0

# Domain 1 on vclock /dev/ptp2
sudo ptp4l -i ptpproxy0 -p /dev/ptp2 -m -2 -s -H --domainNumber=1
```

---

## License

This kernel module is licensed under **GPL-2.0**.
UAPI headers (`ptp_proxy_uapi.h`) are licensed under **GPL-2.0 WITH Linux-syscall-note**.
