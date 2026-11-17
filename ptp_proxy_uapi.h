/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * PTP Proxy - Userspace-backed virtual PTP Hardware Clock
 *
 * UAPI header defining the interface between the ptp_proxy kernel
 * driver and userspace backends that service PTP clock operations.
 *
 * This header is shared between kernel and userspace. It would
 * reside at include/uapi/linux/ptp_proxy.h in an upstream tree.
 *
 * Architecture:
 *   PTP consumer → /dev/ptpN (virtual) → ptp_proxy (kernel)
 *     → read/write fd → userspace backend → real NIC PHC
 *
 * Design goals for upstream:
 *   - No architecture-specific dependencies (no TSC, no x86-isms)
 *   - Uses CLOCK_MONOTONIC_RAW for portable time interpolation
 *   - Standard fd-based interface (read/write/poll)
 *   - Instance lifecycle tied to fd (close fd → cleanup)
 *   - Seqcount protocol for lock-free fast-path gettime
 *
 * Copyright(c) 2026 Intel Corporation
 */

#ifndef _UAPI_LINUX_PTP_PROXY_H
#define _UAPI_LINUX_PTP_PROXY_H

#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/ioctl.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
/*
 * Use linux/types.h if already available (e.g. pulled in by
 * linux/if_tun.h).  Otherwise fall back to stdint typedefs.
 */
#ifdef __linux__
#include <linux/types.h>
#else
typedef uint8_t  __u8;
typedef uint32_t __u32;
typedef uint64_t __u64;
typedef int32_t  __s32;
typedef int64_t  __s64;
#endif
#endif

/*
 * Command types posted by the kernel for userspace to process.
 * Userspace reads these via read() on the instance fd.
 */
enum ptp_proxy_cmd {
	PTP_PROXY_CMD_GETTIME        = 1,  /* Read current (adjusted) clock time */
	PTP_PROXY_CMD_SETTIME        = 2,  /* Set clock time */
	PTP_PROXY_CMD_ADJTIME        = 3,  /* Adjust clock by delta (ns) */
	PTP_PROXY_CMD_ADJFINE        = 4,  /* Adjust frequency (scaled_ppm) */
	PTP_PROXY_CMD_GETCYCLES      = 5,  /* Read free-running (unadjusted) clock */
	PTP_PROXY_CMD_GETCROSSTSTAMP = 6,  /* Cross-timestamp: ns + sys_realtime + sys_monoraw */
	PTP_PROXY_CMD_GETCROSSCYCLES = 7,  /* Cross-timestamp: cycles + sys_realtime + sys_monoraw */
	PTP_PROXY_CMD_NOTIFY_IPV4    = 8,  /* Notification: IPv4 address change */
};

/*
 * Command request - read by userspace via read() on instance fd.
 *
 * The kernel posts one command at a time and blocks (with timeout)
 * until userspace writes back a response. This serialization avoids
 * the need for a command ring buffer.
 *
 * Userspace workflow:
 *   1. poll() or blocking read() on instance fd
 *   2. Read this struct
 *   3. Execute the requested operation on the real hardware
 *   4. Echo transaction_id in the ptp_proxy_response
 */
struct ptp_proxy_request {
	__u32 cmd;          /* enum ptp_proxy_cmd */
	__u32 transaction_id; /* synchronous request ID; zero for notifications */
	union {
		struct {
			__s64 sec;
			__s64 nsec;
		} settime;                /* PTP_PROXY_CMD_SETTIME params */
		__s64 adj_delta_ns;       /* PTP_PROXY_CMD_ADJTIME param */
		__s64 adj_scaled_ppm;     /* PTP_PROXY_CMD_ADJFINE param */
		__u32 ipv4_addr;          /* PTP_PROXY_CMD_NOTIFY_IPV4 (network order) */
	};
};

/*
 * Command response - written by userspace via write() on instance fd.
 *
 * After processing a command, userspace writes this struct to deliver
 * the result. The kernel accepts it only while the matching transaction is
 * pending, then unblocks and returns the result to the PTP clock consumer.
 */
struct ptp_proxy_response {
	__s32 result;       /* 0 = success, negative errno on failure */
	__u32 transaction_id; /* must match the synchronous request */
	union {
		struct {
			__s64 sec;
			__s64 nsec;
		} time;             /* GETTIME / GETCYCLES result */

		/*
		 * Cross-timestamp result (GETCROSSTSTAMP / GETCROSSCYCLES).
		 * device_ns holds nanoseconds for GETCROSSTSTAMP,
		 * raw cycles for GETCROSSCYCLES.
		 */
		struct {
			__s64 device_ns;       /* device time (ns or cycles) */
			__s64 sys_realtime_ns; /* CLOCK_REALTIME at same point */
			__s64 sys_monoraw_ns;  /* CLOCK_MONOTONIC_RAW at same point */
		} crosststamp;
	};
};

/*
 * Instance creation parameters.
 *
 * Passed to PTP_PROXY_IOC_CREATE ioctl on /dev/ptp_proxy.
 * The ioctl returns the instance fd as its return value (like
 * KVM_CREATE_VM). The ptp_index field is filled in by the kernel
 * so userspace knows which /dev/ptpN was created.
 */
struct ptp_proxy_create {
	__s32 ptp_index;    /* OUT: /dev/ptpN index for the virtual clock */
	__s32 max_adj;      /* IN: max freq adjustment (ppb), 0 = default */
};

/*
 * ioctl on /dev/ptp_proxy control device.
 *
 * PTP_PROXY_IOC_CREATE:
 *   Create a new virtual PTP clock backed by userspace.
 *   arg: pointer to struct ptp_proxy_create
 *   Returns: instance fd (>= 0) on success, -1 on error
 *
 *   The returned fd supports:
 *     read()  - Receive commands from kernel (struct ptp_proxy_request)
 *     write() - Send responses to kernel (struct ptp_proxy_response)
 *     poll()  - Wait for commands (POLLIN when command pending)
 *     close() - Destroy the virtual PTP clock and free resources
 */
#define PTP_PROXY_IOC_MAGIC   'P'
#define PTP_PROXY_IOC_CREATE  _IOWR(PTP_PROXY_IOC_MAGIC, 1, \
				    struct ptp_proxy_create)

/* ============================================================
 *  Packet fd interface (Phase 1: integrated net_device)
 *
 *  Each ptp_proxy instance can create a PTP-aware net_device
 *  (ptpproxy0, ptpproxy1, ...) with a packet fd for inline
 *  timestamped packet exchange with userspace.
 *
 *  The packet fd is obtained via PTP_PROXY_IOC_CREATE_NETDEV.
 * ============================================================ */

/*
 * Packet timestamp header — prepended to Ethernet frames on the
 * packet fd. Same layout and semantics for read() and write().
 */
struct ptp_proxy_ts_hdr {
	__s64  sec;         /* HW timestamp seconds */
	__s64  nsec;        /* HW timestamp nanoseconds (0–999999999) */
	__s64  raw_sec;     /* Optional raw/free-running domain seconds */
	__s64  raw_nsec;    /* Optional raw/free-running domain nanoseconds */
	__u32  flags;       /* PTP_PROXY_TS_* flags below */
	__u32  cookie;      /* TX correlation ID; zero is invalid */
};

/* Flags for ptp_proxy_ts_hdr.flags */
#define PTP_PROXY_TS_VALID         0x01  /* sec/nsec carry a valid HW TS */
#define PTP_PROXY_TS_TX_ERRQUEUE   0x02  /* deliver as TX TS via MSG_ERRQUEUE */
#define PTP_PROXY_TS_TX_REQUEST    0x04  /* ptp4l requested TX timestamping */
#define PTP_PROXY_TS_RAW_VALID     0x08  /* raw_sec/raw_nsec are valid */

/*
 * Create net_device + packet fd.
 *
 * Must be called on an instance fd (returned by PTP_PROXY_IOC_CREATE).
 * Creates a net_device linked to the instance's PHC.
 * Returns: packet fd (>= 0) on success, -1 on error.
 *
 * arg: pointer to struct ptp_proxy_netdev_create
 *
 * If ifname is non-empty on input, the driver uses it as the
 * interface name (e.g. "dtap0"). Otherwise a default name
 * "ptpproxyN" is generated.  On output, ifname contains the
 * actual created interface name.
 */
struct ptp_proxy_netdev_create {
	char   ifname[16];   /* IN/OUT: desired name (or empty) / created name */
	__u32  _reserved;
};

#define PTP_PROXY_IOC_CREATE_NETDEV  _IOWR(PTP_PROXY_IOC_MAGIC, 2, \
					   struct ptp_proxy_netdev_create)

/*
 * Exclusive discipliner lock.
 *
 * Only one process may hold the discipliner role at a time.
 * Calling CLAIM while another holder exists returns -EBUSY.
 * RELEASE by non-holder is a no-op (idempotent).
 * The lock is automatically released when the instance fd is closed.
 *
 * Must be called on an instance fd (returned by PTP_PROXY_IOC_CREATE).
 */
#define PTP_PROXY_IOC_CLAIM_DISCIPLINER   _IO(PTP_PROXY_IOC_MAGIC, 3)
#define PTP_PROXY_IOC_RELEASE_DISCIPLINER _IO(PTP_PROXY_IOC_MAGIC, 4)

#endif /* _UAPI_LINUX_PTP_PROXY_H */
