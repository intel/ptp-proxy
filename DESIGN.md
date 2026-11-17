# PTP Proxy Driver Design Document

## 1. Executive Summary & Problem Statement

### 1.1 Motivation
Hardware-based Precision Time Protocol (PTP, IEEE 1588) relies on a PTP Hardware Clock (PHC) exposed as a Linux `/dev/ptpN` device and bound to a network interface. In high-performance networking environments utilizing kernel-bypass frameworks such as **DPDK (Data Plane Development Kit)** or **VFIO**, physical NIC ports and their associated PHC hardware registers are owned and managed directly by userspace drivers rather than kernel network drivers.

Consequently, standard Linux PTP infrastructure tools (`ptp4l`, `phc2sys`, `chrony`) cannot directly access or discipline the hardware clock, forcing users to either:
1. Write custom, proprietary PTP daemons integrated directly into the DPDK application.
2. Run unmaintained, out-of-tree TUN/TAP hacks that degrade packet timestamp accuracy.

### 1.2 Solution: `ptp_proxy`
The `ptp_proxy` driver introduces a generic, userspace-backed PTP clock framework into the Linux kernel. It creates a virtual PHC device (`/dev/ptpN`) and an associated virtual Ethernet interface (`ptpproxyN`). All standard Linux PTP ioctls, timecounter operations, and timestamp requests issued by `ptp4l` are proxied to the userspace DPDK/VFIO application via file-descriptor IPC.

```text
┌─────────────────────────────────────────────────────────────────────────┐
│                          Linux Kernel Subsystem                         │
│                                                                         │
│   ptp4l / phc2sys / chrony (Unmodified PTP Stack)                       │
│        │                               │                                │
│        │ clock_adjtime / ioctl         │ sendmsg() / recvmsg()          │
│        ▼                               ▼                                │
│    /dev/ptpN                     ptpproxy0 (netdev)                     │
│  (Virtual PHC)                         │                                │
│        │                               │ ndo_start_xmit / netif_rx      │
│        ▼                               ▼                                │
│ ─────────────────────────────────────────────────────────────────────── │
│                      ptp_proxy.ko Driver Kernel                         │
│ ─────────────────────────────────────────────────────────────────────── │
│        ▲                               ▲                                │
│        │ read/write (inst_fd)          │ read/write (pkt_fd)            │
│        │ Synchronous Cmds              │ Inline Timestamp Frames        │
│        ▼                               ▼                                │
│ ┌─────────────────────────────────────────────────────────────────────┐ │
│ │                  DPDK / Userspace Service Application               │ │
│ │  - Services clock_adjtime / gettime on real NIC PHC                 │ │
│ │  - Relays PTP packets & extracts HW RX/TX timestamps                │ │
│ └─────────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 2. Architecture & Subsystem Layers

The `ptp_proxy` system consists of four primary operational layers:

1. **Control Layer (`/dev/ptp_proxy`)**: Misc character device handling instance allocation via `PTP_PROXY_IOC_CREATE`.
2. **PTP Clock Layer (`/dev/ptpN`)**: Virtual PTP clock registered with the kernel PTP subsystem (`ptp_clock_register()`), translating PTP clock ops into IPC requests.
3. **Netdev & Relay Layer (`ptpproxyN` & `pkt_fd`)**: Virtual network device handling packet forwarding, NAPI integration, and hardware timestamp injection/feedback.
4. **IPC Channel (`inst_fd` & `pkt_fd`)**: Anonymous file descriptors created via `anon_inode_getfd()`, providing non-blocking polling and ring buffers between kernel and userspace.

---

## 3. Control & Instance Lifecycle

### 3.1 Device Allocation & File Descriptors
Instance creation follows the anonymous inode model (similar to KVM / epoll):

1. Userspace opens `/dev/ptp_proxy`.
2. Userspace issues `ioctl(fd, PTP_PROXY_IOC_CREATE, &create_args)`.
3. The driver allocates a `struct ptp_proxy_inst` and registers a new virtual PHC (`/dev/ptpN`).
4. An anonymous **Instance File Descriptor (`inst_fd`)** is created and returned to userspace.

```c
struct ptp_proxy_create {
    __s32 ptp_index;    /* OUT: created /dev/ptpN index */
    __s32 max_adj;      /* IN: maximum freq adjustment (ppb) */
};
```

### 3.2 Reference Counting & Tear-Down
To prevent race conditions between userspace closing the file descriptors and active kernel threads executing PTP callbacks, `struct ptp_proxy_inst` uses Linux kernel `refcount_t`:

- Opening `inst_fd` or `pkt_fd` increments `refcnt`.
- Active ioctls or NAPI polls temporarily acquire references (`ptp_proxy_inst_get()`).
- Closing `inst_fd` sets `inst->alive = false`, unregisters the PTP clock (`ptp_clock_unregister()`), and wakes all sleeping threads.
- When `refcnt` reaches zero (`ptp_proxy_inst_put()`), memory is freed safely.

---

## 4. Command Path Architecture & Serialization

### 4.1 Synchronous Command Loop
Kernel PTP clock operations (`gettime64`, `settime64`, `adjtime`, `adjfine`, `getcrosststamp`) execute synchronously:

```text
Kernel PTP Callback                ptp_proxy Driver                  Userspace (DPDK)
───────────────────                ────────────────                  ────────────────
ptp_proxy_adjfine()
        │
        ├─────────────────────────► ptp_proxy_exec_cmd()
        │                           │ Populate pending_req
        │                           │ cmd_pending = true
        │                           │ wake_up(&cmd_wq)
        │                           │
        │                           ├────── read(inst_fd) ───────────► Read request
        │                           │                                  Process in HW
        │                           │ ◄──── write(inst_fd) ─────────── Write response
        │                           │
        │                           │ Receive pending_resp
        │                           │ wake_up(&resp_wq)
        ▼                           ▼
Return result ◄───────────────────── Return 0 / errno
```

### 4.2 Serialization & Timeout Protection
- **Command Serialization**: All synchronous commands are serialized using `mutex_lock(&inst->cmd_lock)`. Only one command is in flight per instance at any time, eliminating complex request ring buffers.
- **Timeout Protection**: The kernel sleeps on `wait_event_interruptible_timeout()` for up to `cmd_timeout_ms` (configurable module parameter, default `100ms`). If userspace crashes or hangs, the command times out safely and returns `-ETIMEDOUT` to prevent kernel hangs. Responses must echo the nonzero transaction ID assigned to the request; unsolicited, duplicate, stale, and mismatched responses are rejected.

---

## 5. Non-Sleeping Atomic Context Caching (`getcycles64`)

### 5.1 The Atomic Context Problem
Kernel virtual clocks (`vclocks`) call `getcycles64()` on the parent PHC inside `ptp_vclock_read()`. This function executes within `timecounter_read()` under a spinlock (`spinlock_t`), creating an **atomic context**. Issuing a sleeping IPC command (`ptp_proxy_exec_cmd()`) from `getcycles64()` would trigger `BUG: scheduling while atomic`.

### 5.2 Delayed Workqueue & Interpolation Solution
To resolve this without breaking vclock compatibility:

1. `ptp_proxy` spawns a periodic delayed workqueue (`cycles_work`) running every **1 ms** in process context.
2. `cycles_work` executes `CMD_GETCYCLES` over `inst_fd` and samples `ktime_get_raw_ns()` before and after the IPC call.
3. The midpoint `ktime` and returned hardware cycles are stored under `cycles_lock`.
4. `ptp_proxy_getcycles64()` executes in atomic context without sleeping, returning the cached cycle timestamp.

```text
Process Context (Workqueue every 1ms):
  t_before = ktime_get_raw_ns()
  exec_cmd(CMD_GETCYCLES) -> cycles_hw
  t_after = ktime_get_raw_ns()
  cached_cycles = cycles_hw
  cached_ktime = (t_before + t_after) / 2

Atomic Context (getcycles64 under spinlock):
  read cached_cycles (spin_lock_irqsave)
  return cached_cycles
```

---

## 6. Packet Data Path (`ptpproxyN` Netdev & `pkt_fd`)

### 6.1 Netdev Creation (`PTP_PROXY_IOC_CREATE_NETDEV`)
Calling `PTP_PROXY_IOC_CREATE_NETDEV` on `inst_fd` registers an Ethernet device (`ptpproxy0`, `ptpproxy1`, ...) and returns a dedicated **Packet File Descriptor (`pkt_fd`)**.

Features of `ptpproxyN`:
- Virtual MAC generated randomly via `eth_hw_addr_random()`.
- NAPI instance registered (`netif_napi_add`) to provide `napi_id` bindings required for Linux netdev timestamping.
- `ethtool -T` reports `SOF_TIMESTAMPING_TX_HARDWARE`, `SOF_TIMESTAMPING_RX_HARDWARE`, `SOF_TIMESTAMPING_RAW_HARDWARE`, and `SOF_TIMESTAMPING_BIND_PHC`.

### 6.2 Outbound Path (TX: `ptp4l` -> `pkt_fd`)
1. `ptp4l` transmits a PTP packet (e.g. `Delay_Req`) on `ptpproxy0`.
2. `ndo_start_xmit()` flags `SKBTX_IN_PROGRESS` if hardware timestamping is requested.
3. The `skb` is enqueued in `inst->pkt_queue` and wakes readers on `pkt_wq`.
4. Userspace reads `pkt_fd`. The driver prepends a `struct ptp_proxy_ts_hdr` header carrying a unique 32-bit correlation cookie (`hdr.cookie`):

```c
struct ptp_proxy_ts_hdr {
    __s64  sec;         /* HW timestamp seconds */
    __s64  nsec;        /* HW timestamp nanoseconds */
    __s64  raw_sec;     /* Optional free-running domain seconds */
    __s64  raw_nsec;    /* Optional free-running domain nanoseconds */
    __u32  flags;       /* PTP_PROXY_TS_* flags */
    __u32  cookie;      /* Correlation ID for out-of-order TX */
};
```

5. The original `skb` is saved in `inst->pending_tx_list` indexed by `cookie`.

### 6.3 Inbound Path (RX: `pkt_fd` -> `ptp4l`)
1. DPDK captures an incoming PTP packet and its hardware RX timestamp.
2. Userspace writes `[struct ptp_proxy_ts_hdr][Ethernet frame]` to `pkt_fd` with flag `PTP_PROXY_TS_VALID`.
3. The kernel allocates an `skb`, populates metadata into `skb->cb` (`struct ptp_proxy_inline_tstamps`), sets `SKBTX_HW_TSTAMP_NETDEV`, and passes it to `netif_rx()`.

### 6.4 Hardware TX Timestamp Feedback (`MSG_ERRQUEUE`)
1. DPDK transmits the frame on the physical NIC and retrieves the hardware departure timestamp.
2. Userspace writes `[struct ptp_proxy_ts_hdr][Ethernet frame]` to `pkt_fd` with flag `PTP_PROXY_TS_TX_ERRQUEUE` and the matching `hdr.cookie`.
3. The kernel driver looks up the original pending `skb` in `inst->pending_tx_list` by `cookie`.
4. The driver populates `skb_shared_hwtstamps` and calls `skb_tstamp_tx(tx_skb, &hwts)`.
5. The kernel socket layer delivers the departure timestamp to `ptp4l` via `recvmsg(MSG_ERRQUEUE)`.

---

## 7. Dual-Domain Timestamps & Multi-vclock Support

To support multi-domain PTP via Linux `n_vclocks` created on top of base `/dev/ptp0`:

- The kernel socket layer uses `ndo_get_tstamp(dev, hwtstamps, cycles)` callback when Delivering timestamps.
- When `cycles == false` (standard base PHC), `ndo_get_tstamp` returns `meta->adjusted_ns` (PHC wall-clock time).
- When `cycles == true` (`SOF_TIMESTAMPING_BIND_PHC` on a `vclock`), `ndo_get_tstamp` returns `meta->raw_ns` (unadjusted hardware cycle counter).
- Each `vclock` interpolates time independently from `meta->raw_ns` using its own frequency adjustment.

```c
static ktime_t ptp_proxy_ndev_get_tstamp(struct net_device *dev,
                                         const struct skb_shared_hwtstamps *hwtstamps,
                                         bool cycles)
{
    const struct ptp_proxy_inline_tstamps *meta = hwtstamps->netdev_data;
    s64 ns = cycles ? meta->raw_ns : meta->adjusted_ns;
    return ns_to_ktime(ns);
}
```

---

## 8. Discipline Ownership Protection (referenced `struct pid`)

When running multi-domain PTP, multiple `ptp4l` or `phc2sys` processes share the same underlying virtual PHC. To prevent multiple daemons from concurrently issuing conflicting `adjfine()` or `adjtime()` commands:

1. The first process group (`tgid`) to issue `adjtime()` or `adjfine()` claims discipliner ownership using a referenced `struct pid *`.
2. Subsequent `adjtime()` / `adjfine()` calls from other `tgid`s are rejected with `-EACCES`.
3. If the owner process exits or crashes, the referenced PID is checked under `discipliner_lock` and ownership is automatically transferred to the new caller. Release is accepted only from the matching owner.

---

## 9. Kernel UAPI Contract Specification (`include/uapi/linux/ptp_proxy.h`)

```c
/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_PTP_PROXY_H
#define _UAPI_LINUX_PTP_PROXY_H

#include <linux/types.h>
#include <linux/ioctl.h>

enum ptp_proxy_cmd {
    PTP_PROXY_CMD_GETTIME        = 1,
    PTP_PROXY_CMD_SETTIME        = 2,
    PTP_PROXY_CMD_ADJTIME        = 3,
    PTP_PROXY_CMD_ADJFINE        = 4,
    PTP_PROXY_CMD_GETCYCLES      = 5,
    PTP_PROXY_CMD_GETCROSSTSTAMP = 6,
    PTP_PROXY_CMD_GETCROSSCYCLES = 7,
    PTP_PROXY_CMD_NOTIFY_IPV4    = 8,
};

struct ptp_proxy_request {
    __u32 cmd;
    __u32 transaction_id; /* nonzero synchronous request ID; zero for notifications */
    union {
        struct { __s64 sec; __s64 nsec; } settime;
        __s64 adj_delta_ns;
        __s64 adj_scaled_ppm;
        __u32 ipv4_addr;
    };
};

struct ptp_proxy_response {
    __s32 result;
    __u32 transaction_id; /* must match the request */
    union {
        struct { __s64 sec; __s64 nsec; } time;
        struct {
            __s64 device_ns;
            __s64 sys_realtime_ns;
            __s64 sys_monoraw_ns;
        } crosststamp;
    };
};

struct ptp_proxy_create {
    __s32 ptp_index;
    __s32 max_adj;
};

#define PTP_PROXY_IOC_MAGIC   'P'
#define PTP_PROXY_IOC_CREATE  _IOWR(PTP_PROXY_IOC_MAGIC, 1, struct ptp_proxy_create)

struct ptp_proxy_ts_hdr {
    __s64  sec;
    __s64  nsec;
    __s64  raw_sec;
    __s64  raw_nsec;
    __u32  flags;
    __u32  cookie;        /* 32-bit TX correlation ID; zero is invalid */
};

#define PTP_PROXY_TS_VALID         0x01
#define PTP_PROXY_TS_TX_ERRQUEUE   0x02
#define PTP_PROXY_TS_TX_REQUEST    0x04
#define PTP_PROXY_TS_RAW_VALID     0x08

struct ptp_proxy_netdev_create {
    char   ifname[16];
    __u32  _reserved;
};

#define PTP_PROXY_IOC_CREATE_NETDEV  _IOWR(PTP_PROXY_IOC_MAGIC, 2, struct ptp_proxy_netdev_create)

#endif /* _UAPI_LINUX_PTP_PROXY_H */
```
