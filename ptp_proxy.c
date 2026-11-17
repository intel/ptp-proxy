// SPDX-License-Identifier: GPL-2.0
/*
 * PTP Proxy - Userspace-backed virtual PTP Hardware Clock
 *
 * Creates virtual PTP clocks (/dev/ptpN) whose operations are serviced
 * by a userspace process via a standard fd interface (read/write/poll).
 *
 * This enables PTP clock consumers (e.g., ptp4l, phc2sys) to discipline
 * PTP clocks managed entirely in userspace, such as NIC PHCs bound to
 * userspace drivers (DPDK/VFIO), software-defined clocks, or remote
 * clocks accessed over a network.
 *
 * Interface:
 *   /dev/ptp_proxy  - Control device (misc device)
 *                     ioctl(PTP_PROXY_IOC_CREATE) → instance fd
 *
 *   Instance fd     - Per-instance interface:
 *                     read()  → receive command from kernel
 *                     write() → send response to kernel
 *                     poll()  → POLLIN when command pending
 *                     close() → destroy virtual PTP clock
 *
 * Command path:
 *   All PTP clock operations (gettime, settime, adjtime, adjfine) are
 *   served via the command interface. The kernel posts a request and
 *   sleeps until userspace reads the request, processes it, and writes
 *   back a response.
 *
 * Architecture portability:
 *   - No TSC, rdtsc, or x86-specific dependencies
 *   - Interpolation uses ktime_get_raw_ns() (works on all Linux archs)
 *   - Memory ordering uses smp_load_acquire/smp_store_release
 *   - Userspace uses C11 atomics and clock_gettime(CLOCK_MONOTONIC_RAW)
 *
 * Copyright(c) 2026 Intel Corporation
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/anon_inodes.h>
#include <linux/file.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/time64.h>
#include <linux/workqueue.h>
#include <linux/ptp_clock_kernel.h>
#include <linux/uaccess.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/version.h>
#include <linux/skbuff.h>
#include <linux/net_tstamp.h>
#include <linux/inetdevice.h>
#include <linux/refcount.h>
#include <linux/sched.h>
#include <linux/pid.h>
#include <linux/rcupdate.h>
#include <linux/overflow.h>
#include <linux/capability.h>
#include <linux/user_namespace.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/atomic.h>
#include <net/busy_poll.h>

#include "ptp_proxy_uapi.h"

#define DRIVER_NAME    "ptp_proxy"
#define DRIVER_VERSION "1.0"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("PTP Proxy - userspace-backed virtual PTP clock");
MODULE_VERSION(DRIVER_VERSION);

/* Timeout for waiting on userspace response (milliseconds) */
static int cmd_timeout_ms = 100;
module_param(cmd_timeout_ms, int, 0644);
MODULE_PARM_DESC(cmd_timeout_ms,
		 "Timeout waiting for userspace command response (ms)");

/* Default max frequency adjustment in ppb */
#define PTP_PROXY_DEFAULT_MAX_ADJ  500000000

/* Interval for periodic getcycles cache refresh (ms) */
#define PTP_PROXY_CYCLES_REFRESH_MS  1

/* Maximum packets queued in net_device → userspace direction */
#define PTP_PROXY_PKT_QUEUE_LEN    64

/* Max in-flight TX timestamp requests tracked in kernel */
#define PTP_PROXY_TX_PENDING_LEN   64

/* DPDK retires missing hardware timestamps after about 100 ms. */
#define PTP_PROXY_TX_PENDING_TIMEOUT_MS 1000

/* Global netdev instance counter */
static atomic_t ptp_proxy_netdev_count = ATOMIC_INIT(0);

/* Instance tracking for module cleanup */
static LIST_HEAD(ptp_proxy_inst_list);
static DEFINE_MUTEX(ptp_proxy_inst_lock);

/* Root debugfs directory for per-instance stat exports (may be NULL). */
static struct dentry *ptp_proxy_debugfs_root;

/* Net device private data — defined early for use in cleanup path */
struct ptp_proxy_netdev_priv {
	struct ptp_proxy_inst *inst;
	struct napi_struct napi;
};

/* ============================================================
 *  Per-instance state
 * ============================================================ */

/*
 * Per-instance diagnostic counters exposed via debugfs.
 * All fields use atomic64 so readers get a consistent snapshot
 * without additional locking. Counters saturate at U64_MAX.
 */
struct ptp_proxy_stats {
	/* Command path */
	atomic64_t cmd_posted;
	atomic64_t cmd_timeout;
	atomic64_t cmd_response_stale;      /* txid mismatch or zero (-ESTALE) */
	atomic64_t cmd_response_dup;        /* response already delivered (-EALREADY) */
	atomic64_t cmd_response_unsol;      /* no cmd in flight (-ENOENT) */

	/* Per-command-type call counts (subset of cmd_posted) */
	atomic64_t cmd_gettime;
	atomic64_t cmd_settime;
	atomic64_t cmd_adjtime;
	atomic64_t cmd_adjfine;
	atomic64_t cmd_getcycles;
	atomic64_t cmd_getcrosststamp;
	atomic64_t cmd_getcrosscycles;

	/* Cycles / gettime / crosststamp validation */
	atomic64_t cycles_backward;
	atomic64_t cycles_invalid;          /* -ERANGE / -EOVERFLOW */
	atomic64_t cycles_stale_reads;      /* cache read after backend stall */
	atomic64_t gettime_invalid;
	atomic64_t crosststamp_invalid;

	/* TX timestamp path */
	atomic64_t tx_pending_alloc_fail;
	atomic64_t tx_pending_queue_full;
	atomic64_t tx_cookie_expired;       /* pending entry aged out */
	atomic64_t tx_cookie_unknown;       /* backend acked cookie we don't track */

	/* Packet path */
	atomic64_t pkt_queue_drops;         /* pkt_queue full */
	atomic64_t pkt_rx_drops;            /* netif_rx rejected packet */
	atomic64_t pkt_write_invalid;       /* malformed header / short frame */

	/* Notifications */
	atomic64_t notify_ipv4_queued;
	atomic64_t notify_ipv4_replaced;    /* bounded queue collapsed entries */
	atomic64_t notify_alloc_fail;

	/* Discipliner ownership */
	atomic64_t discipliner_denied;
	atomic64_t discipliner_takeover;    /* previous owner exited */
};

struct ptp_proxy_inst {
	/* Lifetime across instance fd, packet fd, and transient users */
	refcount_t                    refcnt;

	/* Instance list linkage */
	struct list_head        list;

	/* PTP clock registered with the PTP subsystem */
	struct ptp_clock       *ptp_clock;
	struct ptp_clock_info   ptp_info;

	/*
	 * Command interface.
	 * Only one command is in flight at a time, serialized by cmd_lock.
	 * The kernel posts a request and sleeps on resp_wq.
	 * Userspace reads the request via read(), processes it, and
	 * writes the response via write(), waking the kernel.
	 */
	struct mutex                   cmd_lock;
	spinlock_t                     cmd_state_lock;
	wait_queue_head_t              cmd_wq;   /* userspace waits here */
	wait_queue_head_t              resp_wq;  /* kernel waits here */

	struct ptp_proxy_request       pending_req;
	struct ptp_proxy_response      pending_resp;
	bool                           cmd_pending;  /* request available */
	bool                           resp_ready;   /* response received */
	u32                            next_transaction_id;

	/* Lifecycle: set to false when userspace closes the fd */
	bool                           alive;

	/* First process group to call adjfine/adjtime owns discipline. */
	struct mutex                   discipliner_lock;
	struct pid                    *discipliner_pid;

	/* --- Cached getcycles64 (non-sleeping for atomic context) --- */
	spinlock_t                     cycles_lock;
	u64                            cached_cycles_ns;  /* last CMD_GETCYCLES result */
	u64                            cached_cycles_ktime; /* ktime_get_raw_ns() at read */
	bool                           cycles_valid;      /* cache has been populated */
	bool                           cycles_stale;      /* last sample is too old */
	struct delayed_work            cycles_work;       /* periodic refresh */

	/* --- Net_device + packet fd (Phase 1) --- */
	struct mutex                  netdev_lock; /* serialize create/destroy */
	struct net_device             *netdev;
	struct sk_buff_head            pkt_queue;     /* netdev TX → userspace */
	wait_queue_head_t              pkt_wq;        /* wake packet fd readers */
	bool                           netdev_created;

	/* Pending TX skbs awaiting timestamp feedback (cookie keyed). */
	spinlock_t                     pending_tx_lock;
	struct list_head               pending_tx_list;
	u32                            next_tx_cookie;
	u16                            pending_tx_count;

	/* --- Notification queue (persists until userspace reads) --- */
	struct list_head               notify_queue;  /* list of ptp_proxy_notify */
	spinlock_t                     notify_lock;
	u64                            notify_seq;

	/* --- Diagnostic counters --- */
	struct ptp_proxy_stats         stats;
	struct dentry                 *debugfs_dir;
};

/* Queued notification entry (used by notify_queue) */
struct ptp_proxy_notify {
	struct list_head        list;
	struct ptp_proxy_request req;
};

/* Pending TX timestamp entry. */
struct ptp_proxy_pending_tx {
	struct list_head list;
	struct sk_buff *skb;
	u32 cookie;
	unsigned long expires;
};

static void ptp_proxy_pending_tx_expire(struct ptp_proxy_inst *inst);

static inline bool ptp_proxy_inst_get(struct ptp_proxy_inst *inst)
{
	return refcount_inc_not_zero(&inst->refcnt);
}

static inline void ptp_proxy_inst_put(struct ptp_proxy_inst *inst)
{
	if (refcount_dec_and_test(&inst->refcnt))
		kfree(inst);
}

/* ============================================================
 *  Diagnostic counters (debugfs)
 * ============================================================ */

#define PTP_PROXY_STAT_INC(inst, field) \
	atomic64_inc(&(inst)->stats.field)

static int ptp_proxy_stats_show(struct seq_file *m, void *v)
{
	struct ptp_proxy_inst *inst = m->private;
	const struct ptp_proxy_stats *s = &inst->stats;
	unsigned long flags;
	unsigned int pending_tx_now;

#define P(field) \
	seq_printf(m, "%-24s %llu\n", #field, \
		   (unsigned long long)atomic64_read(&s->field))

	P(cmd_posted);
	P(cmd_timeout);
	P(cmd_response_stale);
	P(cmd_response_dup);
	P(cmd_response_unsol);
	P(cmd_gettime);
	P(cmd_settime);
	P(cmd_adjtime);
	P(cmd_adjfine);
	P(cmd_getcycles);
	P(cmd_getcrosststamp);
	P(cmd_getcrosscycles);
	P(cycles_backward);
	P(cycles_invalid);
	P(cycles_stale_reads);
	P(gettime_invalid);
	P(crosststamp_invalid);
	P(tx_pending_alloc_fail);
	P(tx_pending_queue_full);
	P(tx_cookie_expired);
	P(tx_cookie_unknown);
	P(pkt_queue_drops);
	P(pkt_rx_drops);
	P(pkt_write_invalid);
	P(notify_ipv4_queued);
	P(notify_ipv4_replaced);
	P(notify_alloc_fail);
	P(discipliner_denied);
	P(discipliner_takeover);
#undef P

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	pending_tx_now = inst->pending_tx_count;
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);
	seq_printf(m, "%-24s %u\n", "pending_tx_current", pending_tx_now);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(ptp_proxy_stats);

static void ptp_proxy_debugfs_create(struct ptp_proxy_inst *inst, int index)
{
	char name[16];

	if (!ptp_proxy_debugfs_root)
		return;

	snprintf(name, sizeof(name), "ptp%d", index);
	inst->debugfs_dir = debugfs_create_dir(name, ptp_proxy_debugfs_root);
	if (IS_ERR_OR_NULL(inst->debugfs_dir)) {
		inst->debugfs_dir = NULL;
		return;
	}
	debugfs_create_file("stats", 0400, inst->debugfs_dir, inst,
			    &ptp_proxy_stats_fops);
}

static void ptp_proxy_debugfs_destroy(struct ptp_proxy_inst *inst)
{
	debugfs_remove_recursive(inst->debugfs_dir);
	inst->debugfs_dir = NULL;
}

/* ============================================================
 *  Command path: kernel posts command, sleeps for response
 * ============================================================ */

static int ptp_proxy_exec_cmd(struct ptp_proxy_inst *inst,
			      const struct ptp_proxy_request *req,
			      struct ptp_proxy_response *resp)
{
	unsigned long flags;
	int timeout_ms;
	int ret;

	if (!inst->alive)
		return -ENODEV;

	mutex_lock(&inst->cmd_lock);

	/* Post the command for userspace */
	spin_lock_irqsave(&inst->cmd_state_lock, flags);
	inst->pending_req  = *req;
	inst->pending_req.transaction_id = ++inst->next_transaction_id;
	if (inst->pending_req.transaction_id == 0)
		inst->pending_req.transaction_id = ++inst->next_transaction_id;
	inst->resp_ready   = false;
	inst->cmd_pending  = true;
	spin_unlock_irqrestore(&inst->cmd_state_lock, flags);

	PTP_PROXY_STAT_INC(inst, cmd_posted);

	switch (req->cmd) {
	case PTP_PROXY_CMD_GETTIME:
		PTP_PROXY_STAT_INC(inst, cmd_gettime);
		break;
	case PTP_PROXY_CMD_SETTIME:
		PTP_PROXY_STAT_INC(inst, cmd_settime);
		break;
	case PTP_PROXY_CMD_ADJTIME:
		PTP_PROXY_STAT_INC(inst, cmd_adjtime);
		break;
	case PTP_PROXY_CMD_ADJFINE:
		PTP_PROXY_STAT_INC(inst, cmd_adjfine);
		break;
	case PTP_PROXY_CMD_GETCYCLES:
		PTP_PROXY_STAT_INC(inst, cmd_getcycles);
		break;
	case PTP_PROXY_CMD_GETCROSSTSTAMP:
		PTP_PROXY_STAT_INC(inst, cmd_getcrosststamp);
		break;
	case PTP_PROXY_CMD_GETCROSSCYCLES:
		PTP_PROXY_STAT_INC(inst, cmd_getcrosscycles);
		break;
	}

	/* Wake userspace (blocked in read() or poll()) */
	wake_up_interruptible(&inst->cmd_wq);

	/* Sleep until userspace responds, fd closes, or timeout */
	timeout_ms = max(READ_ONCE(cmd_timeout_ms), 1);
	ret = wait_event_interruptible_timeout(
		inst->resp_wq,
		READ_ONCE(inst->resp_ready) || !READ_ONCE(inst->alive),
		msecs_to_jiffies(timeout_ms));

	spin_lock_irqsave(&inst->cmd_state_lock, flags);
	if (!READ_ONCE(inst->alive)) {
		ret = -ENODEV;
	} else if (ret == 0) {
		/* Timeout - userspace didn't respond in time */
		ret = -ETIMEDOUT;
	} else if (ret > 0 && inst->resp_ready) {
		/* Got a response */
		*resp = inst->pending_resp;
		ret = resp->result > 0 ? -EPROTO : resp->result;
	}
	/* ret < 0: interrupted by signal */

	inst->cmd_pending = false;
	inst->resp_ready = false;
	spin_unlock_irqrestore(&inst->cmd_state_lock, flags);

	if (ret == -ETIMEDOUT)
		pr_warn_ratelimited("ptp_proxy: cmd %u timed out (%d ms)\n",
				    req->cmd, timeout_ms);
	if (ret == -ETIMEDOUT)
		PTP_PROXY_STAT_INC(inst, cmd_timeout);
	mutex_unlock(&inst->cmd_lock);

	return ret;
}

/* ============================================================
 *  PTP clock operations (called by PTP subsystem)
 * ============================================================ */

static int ptp_proxy_response_to_timespec64(
		const struct ptp_proxy_response *resp, struct timespec64 *ts)
{
	struct timespec64 value = {
		.tv_sec = resp->time.sec,
		.tv_nsec = resp->time.nsec,
	};

	if (!timespec64_valid_strict(&value))
		return -ERANGE;

	*ts = value;
	return 0;
}

static int ptp_proxy_gettime64(struct ptp_clock_info *info,
			       struct timespec64 *ts)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = { .cmd = PTP_PROXY_CMD_GETTIME };
	struct ptp_proxy_response resp;
	int ret;

	ret = ptp_proxy_exec_cmd(inst, &req, &resp);
	if (ret == 0)
		ret = ptp_proxy_response_to_timespec64(&resp, ts);
	if (ret == -ERANGE)
		PTP_PROXY_STAT_INC(inst, gettime_invalid);
	return ret;
}

static int ptp_proxy_gettimex64(struct ptp_clock_info *info,
				struct timespec64 *ts,
				struct ptp_system_timestamp *sts)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = { .cmd = PTP_PROXY_CMD_GETTIME };
	struct ptp_proxy_response resp;
	int ret;

	ptp_read_system_prets(sts);
	ret = ptp_proxy_exec_cmd(inst, &req, &resp);
	ptp_read_system_postts(sts);
	if (ret == 0)
		ret = ptp_proxy_response_to_timespec64(&resp, ts);
	if (ret == -ERANGE)
		PTP_PROXY_STAT_INC(inst, gettime_invalid);
	return ret;
}

/*
 * getcycles64 - Return raw/free-running (unadjusted) PHC time.
 *
 * IMPORTANT: This is called from atomic/spinlock context by the kernel's
 * ptp_convert_timestamp() for vclock support. It MUST NOT sleep.
 *
 * WORKAROUND: The kernel's ptp_vclock_read() calls the parent PHC's
 * getcycles64 under a spinlock (timecounter_read → ptp_read_system_*),
 * making it illegal to sleep.  Our getcycles64 needs to issue a command
 * to userspace (DPDK) via inst_fd, which sleeps.  We work around this
 * by using a delayed workqueue that refreshes a cache every 1ms in
 * process context, and getcycles64 returns the last valid cached value.
 *
 * This is a workaround for the kernel bug:
 *   "PTP vclock: BUG: scheduling while atomic"
 *   https://lore.kernel.org/netdev/69d0ff33-bd32-6aa5-d36c-fbdc3c01337c@redhat.com/
 *
 * Once the kernel fixes getcycles64 to be called from sleepable context,
 * this caching layer can be removed and getcycles64 can directly issue
 * CMD_GETCYCLES via ptp_proxy_exec_cmd().
 *
 * The cache can freeze vclock progress while refresh is unavailable. Since
 * the vclock caller ignores errors, stale reads retain the last valid value
 * rather than returning an invalid timestamp.
 */
static int ptp_proxy_getcycles64(struct ptp_clock_info *info,
				 struct timespec64 *ts)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	u64 cycles_ns, now;
	bool became_stale = false;
	unsigned long flags;

	spin_lock_irqsave(&inst->cycles_lock, flags);
	if (!inst->cycles_valid) {
		spin_unlock_irqrestore(&inst->cycles_lock, flags);
		/*
		 * MUST NOT return -EAGAIN: the kernel's ptp_vclock_read()
		 * ignores our return value and would use ts={0,0}, corrupting
		 * the vclock timecounter via the 32-bit cycle mask.
		 * Fall back to CLOCK_MONOTONIC_RAW domain until first cache
		 * refresh completes.
		 */
		now = ktime_get_raw_ns();
		*ts = ns_to_timespec64(now);
		return 0;
	}
	cycles_ns = inst->cached_cycles_ns;
	now = ktime_get_raw_ns();
	if (now - inst->cached_cycles_ktime >
	    max_t(u64, 2ULL * max(READ_ONCE(cmd_timeout_ms), 1),
		  10ULL * PTP_PROXY_CYCLES_REFRESH_MS) * NSEC_PER_MSEC &&
	    !inst->cycles_stale) {
		inst->cycles_stale = true;
		became_stale = true;
	}
	spin_unlock_irqrestore(&inst->cycles_lock, flags);

	if (became_stale)
		pr_warn_ratelimited("ptp_proxy: GETCYCLES cache is stale\n");

	if (inst->cycles_stale)
		PTP_PROXY_STAT_INC(inst, cycles_stale_reads);

	/* Return the last valid sample even when stale; the caller ignores errors. */
	*ts = ns_to_timespec64(cycles_ns);
	return 0;
}

/*
 * getcyclesx64 - Return raw/free-running cycles with system timestamps.
 *
 * Extended version of getcycles64. Called from process context by the
 * vclock's gettimex64 path (PTP_SYS_OFFSET_EXTENDED on a vclock).
 *
	* Uses the same cached cycles as getcycles64. The pre/post
	* system timestamps bracket only the cache read, giving
 * a sub-microsecond window — tighter than a sleeping IPC roundtrip.
 */
static int ptp_proxy_getcyclesx64(struct ptp_clock_info *info,
				  struct timespec64 *ts,
				  struct ptp_system_timestamp *sts)
{
	int ret;

	ptp_read_system_prets(sts);
	ret = ptp_proxy_getcycles64(info, ts);
	ptp_read_system_postts(sts);

	return ret;
}

/*
 * Workqueue handler: periodically refresh the cached cycles value
 * by issuing CMD_GETCYCLES through the normal (sleeping) command path.
 *
 * WORKAROUND: Part of the getcycles64 atomic-context workaround.
 * See getcycles64 comment and kernel bug link above.
 * This entire workqueue + cache mechanism can be removed once the
 * kernel allows getcycles64 to sleep.
 */
static int ptp_proxy_response_time_to_ns(const struct ptp_proxy_response *resp,
					 u64 *ns)
{
	u64 sec;
	u64 value;

	if (resp->time.sec < 0 || resp->time.nsec < 0 ||
	    resp->time.nsec >= NSEC_PER_SEC)
		return -ERANGE;

	sec = resp->time.sec;
	if (check_mul_overflow(sec, (u64)NSEC_PER_SEC, &value) ||
	    check_add_overflow(value, (u64)resp->time.nsec, &value))
		return -EOVERFLOW;

	*ns = value;
	return 0;
}

static void ptp_proxy_cycles_refresh_work(struct work_struct *work)
{
	struct ptp_proxy_inst *inst =
		container_of(work, struct ptp_proxy_inst, cycles_work.work);
	struct ptp_proxy_request req = { .cmd = PTP_PROXY_CMD_GETCYCLES };
	struct ptp_proxy_response resp;
	u64 cycles_ns, old_cycles_ns = 0;
	u64 ktime_before, ktime_after, ktime_mid;
	bool backward = false;
	unsigned long flags;
	int ret;

	if (!inst->alive)
		return;

	ptp_proxy_pending_tx_expire(inst);

	/* Bracket the command with ktime reads for best interpolation */
	ktime_before = ktime_get_raw_ns();
	ret = ptp_proxy_exec_cmd(inst, &req, &resp);
	ktime_after = ktime_get_raw_ns();

	if (ret == 0)
		ret = ptp_proxy_response_time_to_ns(&resp, &cycles_ns);

	if (ret == 0) {
		ktime_mid = ktime_before + (ktime_after - ktime_before) / 2;

		spin_lock_irqsave(&inst->cycles_lock, flags);
		if (inst->cycles_valid && cycles_ns < inst->cached_cycles_ns) {
			old_cycles_ns = inst->cached_cycles_ns;
			backward = true;
		} else {
			inst->cached_cycles_ns = cycles_ns;
			inst->cached_cycles_ktime = ktime_mid;
			inst->cycles_valid = true;
			inst->cycles_stale = false;
		}
		spin_unlock_irqrestore(&inst->cycles_lock, flags);
	}

	if (ret == -ERANGE || ret == -EOVERFLOW)
		pr_warn_ratelimited("ptp_proxy: invalid GETCYCLES response: %lld.%09lld\n",
				    resp.time.sec, resp.time.nsec);
	else if (backward)
		pr_warn_ratelimited("ptp_proxy: GETCYCLES moved backward: old=%llu new=%llu\n",
				    old_cycles_ns, cycles_ns);

	if (ret == -ERANGE || ret == -EOVERFLOW)
		PTP_PROXY_STAT_INC(inst, cycles_invalid);
	else if (backward)
		PTP_PROXY_STAT_INC(inst, cycles_backward);

	/* Reschedule if still alive */
	if (inst->alive)
		schedule_delayed_work(&inst->cycles_work,
				      msecs_to_jiffies(PTP_PROXY_CYCLES_REFRESH_MS));
}

static int ptp_proxy_settime64(struct ptp_clock_info *info,
			       const struct timespec64 *ts)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = {
		.cmd = PTP_PROXY_CMD_SETTIME,
		.settime = { .sec = ts->tv_sec, .nsec = ts->tv_nsec },
	};
	struct ptp_proxy_response resp;

	return ptp_proxy_exec_cmd(inst, &req, &resp);
}

/* The referenced pid object survives numeric PID reuse. */
static bool ptp_proxy_pid_alive(struct pid *pid)
{
	bool alive;

	rcu_read_lock();
	alive = pid_task(pid, PIDTYPE_TGID) != NULL;
	rcu_read_unlock();

	return alive;
}

/* Auto-claim discipliner role on first adjtime/adjfine; reject other processes.
 * If the previous owner has exited, automatically transfer ownership. */
static int ptp_proxy_check_discipliner(struct ptp_proxy_inst *inst)
{
	struct pid *caller = get_task_pid(current, PIDTYPE_TGID);
	struct pid *old_owner = NULL;
	pid_t caller_nr = task_tgid_nr(current);
	pid_t owner_nr = 0;
	int ret = 0;

	mutex_lock(&inst->discipliner_lock);
	if (!READ_ONCE(inst->alive)) {
		ret = -ENODEV;
	} else if (!inst->discipliner_pid) {
		inst->discipliner_pid = caller;
		caller = NULL;
	} else if (inst->discipliner_pid != caller) {
		owner_nr = pid_nr(inst->discipliner_pid);
		if (!ptp_proxy_pid_alive(inst->discipliner_pid)) {
			old_owner = inst->discipliner_pid;
			inst->discipliner_pid = caller;
			caller = NULL;
		} else {
			ret = -EACCES;
		}
	}
	mutex_unlock(&inst->discipliner_lock);

	if (old_owner) {
		pr_info("ptp_proxy: tgid %d claimed discipliner (previous owner %d exited)\n",
			caller_nr, owner_nr);
		put_pid(old_owner);
		PTP_PROXY_STAT_INC(inst, discipliner_takeover);
	}
	if (caller)
		put_pid(caller);

	if (ret == -EACCES) {
		PTP_PROXY_STAT_INC(inst, discipliner_denied);
		pr_warn_ratelimited(
			"ptp_proxy: tgid %d denied adjtime/adjfine; tgid %d owns discipliner\n",
			caller_nr, owner_nr);
	}

	return ret;
}

static int ptp_proxy_claim_discipliner(struct ptp_proxy_inst *inst)
{
	struct pid *caller = get_task_pid(current, PIDTYPE_TGID);
	struct pid *old_owner = NULL;
	int ret = 0;

	mutex_lock(&inst->discipliner_lock);
	if (!READ_ONCE(inst->alive)) {
		ret = -ENODEV;
	} else if (!inst->discipliner_pid) {
		inst->discipliner_pid = caller;
		caller = NULL;
	} else if (inst->discipliner_pid != caller) {
		if (!ptp_proxy_pid_alive(inst->discipliner_pid)) {
			old_owner = inst->discipliner_pid;
			inst->discipliner_pid = caller;
			caller = NULL;
		} else {
			ret = -EBUSY;
		}
	}
	mutex_unlock(&inst->discipliner_lock);

	if (old_owner)
		put_pid(old_owner);
	if (caller)
		put_pid(caller);

	return ret;
}

static void ptp_proxy_release_discipliner(struct ptp_proxy_inst *inst)
{
	struct pid *caller = get_task_pid(current, PIDTYPE_TGID);
	struct pid *owner = NULL;

	mutex_lock(&inst->discipliner_lock);
	if (inst->discipliner_pid == caller) {
		owner = inst->discipliner_pid;
		inst->discipliner_pid = NULL;
	}
	mutex_unlock(&inst->discipliner_lock);

	if (owner)
		put_pid(owner);
	put_pid(caller);
}

static void ptp_proxy_clear_discipliner(struct ptp_proxy_inst *inst)
{
	struct pid *owner;

	mutex_lock(&inst->discipliner_lock);
	owner = inst->discipliner_pid;
	inst->discipliner_pid = NULL;
	mutex_unlock(&inst->discipliner_lock);

	if (owner)
		put_pid(owner);
}

static int ptp_proxy_adjtime(struct ptp_clock_info *info, s64 delta)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = {
		.cmd = PTP_PROXY_CMD_ADJTIME,
		.adj_delta_ns = delta,
	};
	struct ptp_proxy_response resp;
	int ret;

	ret = ptp_proxy_check_discipliner(inst);
	if (ret)
		return ret;
	return ptp_proxy_exec_cmd(inst, &req, &resp);
}

static int ptp_proxy_adjfine(struct ptp_clock_info *info, long scaled_ppm)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = {
		.cmd = PTP_PROXY_CMD_ADJFINE,
		.adj_scaled_ppm = scaled_ppm,
	};
	struct ptp_proxy_response resp;
	int ret;

	ret = ptp_proxy_check_discipliner(inst);
	if (ret)
		return ret;
	return ptp_proxy_exec_cmd(inst, &req, &resp);
}

static int ptp_proxy_response_to_crosststamp(
		const struct ptp_proxy_response *resp,
		struct system_device_crosststamp *xtstamp)
{
	struct system_device_crosststamp value;

	if (resp->crosststamp.device_ns < 0 ||
	    resp->crosststamp.sys_realtime_ns < 0 ||
	    resp->crosststamp.sys_monoraw_ns < 0)
		return -ERANGE;

	value.device = ns_to_ktime(resp->crosststamp.device_ns);
	value.sys_realtime = ns_to_ktime(resp->crosststamp.sys_realtime_ns);
	value.sys_monoraw = ns_to_ktime(resp->crosststamp.sys_monoraw_ns);
	*xtstamp = value;

	return 0;
}

/*
 * getcrosststamp - Hardware-correlated cross-timestamp (nanoseconds).
 *
 * Returns a triple: (device_ns, sys_realtime, sys_monoraw) captured
 * at a single correlated point in hardware (e.g., PCIe PTM, Intel ART).
 *
 * Used directly by userspace via PTP_SYS_OFFSET_PRECISE ioctl on the
 * base PHC (not vclocks).
 */
static int ptp_proxy_getcrosststamp(struct ptp_clock_info *info,
				    struct system_device_crosststamp *xtstamp)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = { .cmd = PTP_PROXY_CMD_GETCROSSTSTAMP };
	struct ptp_proxy_response resp;
	int ret;

	ret = ptp_proxy_exec_cmd(inst, &req, &resp);
	if (ret)
		return ret;

	ret = ptp_proxy_response_to_crosststamp(&resp, xtstamp);
	if (ret)
		PTP_PROXY_STAT_INC(inst, crosststamp_invalid);
	return ret;
}

/*
 * getcrosscycles - Hardware-correlated cross-timestamp (raw cycles).
 *
 * Same as getcrosststamp but returns raw device cycles instead of
 * nanoseconds. Consumed by vclocks: the vclock applies its own
 * timecounter to convert cycles → vclock nanoseconds, then exposes
 * the result as the vclock's getcrosststamp.
 */
static int ptp_proxy_getcrosscycles(struct ptp_clock_info *info,
				    struct system_device_crosststamp *xtstamp)
{
	struct ptp_proxy_inst *inst =
		container_of(info, struct ptp_proxy_inst, ptp_info);
	struct ptp_proxy_request req = { .cmd = PTP_PROXY_CMD_GETCROSSCYCLES };
	struct ptp_proxy_response resp;
	int ret;

	ret = ptp_proxy_exec_cmd(inst, &req, &resp);
	if (ret)
		return ret;

	ret = ptp_proxy_response_to_crosststamp(&resp, xtstamp);
	if (ret)
		PTP_PROXY_STAT_INC(inst, crosststamp_invalid);
	return ret;
}

/* ============================================================
 *  Instance fd operations (returned by PTP_PROXY_IOC_CREATE)
 * ============================================================ */

/*
 * read() - Receive a pending command from the kernel.
 *
 * Blocks until a command is available (or returns -EAGAIN if
 * O_NONBLOCK is set). Returns exactly sizeof(struct ptp_proxy_request).
 *
 * Priority: synchronous commands drain first so notifications cannot delay
 * the single-in-flight command; persisted notifications follow.
 */
static bool ptp_proxy_deliver_cmd(struct ptp_proxy_inst *inst,
				  char __user *buf, ssize_t *ret)
{
	struct ptp_proxy_request req;
	unsigned long flags;

	spin_lock_irqsave(&inst->cmd_state_lock, flags);
	if (!inst->cmd_pending) {
		spin_unlock_irqrestore(&inst->cmd_state_lock, flags);
		return false;
	}
	req = inst->pending_req;
	spin_unlock_irqrestore(&inst->cmd_state_lock, flags);

	*ret = copy_to_user(buf, &req, sizeof(req)) ? -EFAULT : sizeof(req);
	return true;
}

static bool ptp_proxy_deliver_notification(struct ptp_proxy_inst *inst,
					   char __user *buf, ssize_t *ret)
{
	struct ptp_proxy_notify *n;
	struct ptp_proxy_request req;
	u64 seq;

	spin_lock_bh(&inst->notify_lock);
	if (list_empty(&inst->notify_queue)) {
		spin_unlock_bh(&inst->notify_lock);
		return false;
	}
	n = list_first_entry(&inst->notify_queue,
			     struct ptp_proxy_notify, list);
	req = n->req;
	seq = inst->notify_seq;
	list_del_init(&n->list);
	spin_unlock_bh(&inst->notify_lock);

	if (copy_to_user(buf, &req, sizeof(req))) {
		spin_lock_bh(&inst->notify_lock);
		if (inst->notify_seq == seq)
			list_add(&n->list, &inst->notify_queue);
		else
			kfree(n);
		spin_unlock_bh(&inst->notify_lock);
		*ret = -EFAULT;
	} else {
		kfree(n);
		*ret = sizeof(req);
	}
	return true;
}

static ssize_t ptp_proxy_inst_read(struct file *filp, char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	ssize_t ret;
	int wait_ret;

	if (count < sizeof(struct ptp_proxy_request))
		return -EINVAL;

	if (ptp_proxy_deliver_cmd(inst, buf, &ret))
		return ret;
	if (ptp_proxy_deliver_notification(inst, buf, &ret))
		return ret;

	if (filp->f_flags & O_NONBLOCK)
		return -EAGAIN;

	wait_ret = wait_event_interruptible(
		inst->cmd_wq,
		READ_ONCE(inst->cmd_pending) ||
		!list_empty(&inst->notify_queue) ||
		!READ_ONCE(inst->alive));
	if (wait_ret)
		return wait_ret;
	if (!READ_ONCE(inst->alive))
		return -ENODEV;

	if (ptp_proxy_deliver_cmd(inst, buf, &ret))
		return ret;
	if (ptp_proxy_deliver_notification(inst, buf, &ret))
		return ret;

	return -EAGAIN;
}

/*
 * write() - Send a command response to the kernel.
 *
 * The kernel is sleeping in ptp_proxy_exec_cmd() waiting for this. Accept
 * exactly one response whose transaction ID matches the pending request.
 * Reject unsolicited, duplicate, late, and mismatched responses.
 */
static ssize_t ptp_proxy_inst_write(struct file *filp,
				    const char __user *buf,
				    size_t count, loff_t *ppos)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	struct ptp_proxy_response resp;
	unsigned long flags;
	int ret = 0;

	if (count < sizeof(struct ptp_proxy_response))
		return -EINVAL;

	if (copy_from_user(&resp, buf, sizeof(resp)))
		return -EFAULT;

	spin_lock_irqsave(&inst->cmd_state_lock, flags);
	if (!READ_ONCE(inst->alive))
		ret = -ENODEV;
	else if (!inst->cmd_pending)
		ret = -ENOENT;
	else if (inst->resp_ready)
		ret = -EALREADY;
	else if (resp.transaction_id == 0 ||
		 resp.transaction_id != inst->pending_req.transaction_id)
		ret = -ESTALE;
	else {
		inst->pending_resp = resp;
		inst->resp_ready = true;
	}
	spin_unlock_irqrestore(&inst->cmd_state_lock, flags);

	switch (ret) {
	case -ENOENT:
		PTP_PROXY_STAT_INC(inst, cmd_response_unsol);
		break;
	case -EALREADY:
		PTP_PROXY_STAT_INC(inst, cmd_response_dup);
		break;
	case -ESTALE:
		PTP_PROXY_STAT_INC(inst, cmd_response_stale);
		break;
	}

	if (ret)
		return ret;

	wake_up_interruptible(&inst->resp_wq);

	return sizeof(resp);
}

/*
 * poll() - Report readiness for read (command pending) or write.
 */
static __poll_t ptp_proxy_inst_poll(struct file *filp,
				    struct poll_table_struct *wait)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	__poll_t mask = 0;

	poll_wait(filp, &inst->cmd_wq, wait);

	if (inst->cmd_pending)
		mask |= EPOLLIN | EPOLLRDNORM;

	spin_lock_bh(&inst->notify_lock);
	if (!list_empty(&inst->notify_queue))
		mask |= EPOLLIN | EPOLLRDNORM;
	spin_unlock_bh(&inst->notify_lock);

	if (!inst->alive)
		mask |= EPOLLHUP;

	/* Responses can always be written */
	mask |= EPOLLOUT | EPOLLWRNORM;

	return mask;
}

/*
 * release() - Called when userspace closes the instance fd.
 *
 * Unregisters the virtual PTP clock and frees all resources.
 * Any kernel threads blocked in exec_cmd will see alive=false
 * and return -ENODEV.
 *
 * ptp_clock_unregister() ensures all outstanding PTP callbacks
 * complete before returning, so there are no use-after-free races.
 */
static void ptp_proxy_pending_tx_purge(struct ptp_proxy_inst *inst);

static int ptp_proxy_inst_release(struct inode *inode, struct file *filp)
{
	struct ptp_proxy_inst *inst = filp->private_data;

	/* Signal that userspace is gone */
	inst->alive = false;
	smp_wmb();

	/* Drop the referenced owner before destroying the instance. */
	ptp_proxy_clear_discipliner(inst);

	/* Cancel periodic cycles refresh before anything else */
	cancel_delayed_work_sync(&inst->cycles_work);

	/* Wake any kernel threads waiting for responses */
	wake_up_interruptible(&inst->resp_wq);
	wake_up_interruptible(&inst->cmd_wq);

	/* Wake packet fd readers (they'll see alive=false) */
	wake_up_interruptible(&inst->pkt_wq);

	/* Unregister net_device and drain packet queue */
	mutex_lock(&inst->netdev_lock);
	if (inst->netdev) {
		struct ptp_proxy_netdev_priv *priv = netdev_priv(inst->netdev);
		unregister_netdev(inst->netdev);
		netif_napi_del(&priv->napi);
		inst->netdev = NULL;
		inst->netdev_created = false;
	}
	mutex_unlock(&inst->netdev_lock);
	skb_queue_purge(&inst->pkt_queue);

	/* Free any pending TX skbs */
	ptp_proxy_pending_tx_purge(inst);

	/* Drain notification queue */
	{
		struct ptp_proxy_notify *n, *tmp;

		spin_lock_bh(&inst->notify_lock);
		list_for_each_entry_safe(n, tmp, &inst->notify_queue, list) {
			list_del(&n->list);
			kfree(n);
		}
		spin_unlock_bh(&inst->notify_lock);
	}

	/*
	 * Unregister PTP clock. This blocks until all outstanding
	 * ptp_clock_info callbacks have returned, so we're safe
	 * to free the instance after this.
	 */
	if (inst->ptp_clock) {
		ptp_clock_unregister(inst->ptp_clock);
		inst->ptp_clock = NULL;
	}

	ptp_proxy_debugfs_destroy(inst);

	mutex_lock(&ptp_proxy_inst_lock);
	list_del(&inst->list);
	mutex_unlock(&ptp_proxy_inst_lock);

	ptp_proxy_inst_put(inst);
	return 0;
}

/* Forward declaration — defined after net_device section */
static long ptp_proxy_inst_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg);

static const struct file_operations ptp_proxy_inst_fops = {
	.owner          = THIS_MODULE,
	.read           = ptp_proxy_inst_read,
	.write          = ptp_proxy_inst_write,
	.poll           = ptp_proxy_inst_poll,
	.unlocked_ioctl = ptp_proxy_inst_ioctl,
	.release        = ptp_proxy_inst_release,
};

/* ============================================================
 *  Net_device: PTP-aware Ethernet device (ptpproxyN)
 *
 *  A thin net_device created per instance. ptp4l sends and
 *  receives PTP packets on this interface. ethtool reports
 *  HW timestamp capability linked to the virtual PHC.
 *
 *  TX path (ptp4l → net_device → packet fd → userspace):
 *    ndo_start_xmit queues the skb for userspace to read
 *    from the packet fd (prepended with ptp_proxy_ts_hdr).
 *
 *  RX path (userspace → packet fd write → net_device → ptp4l):
 *    Userspace writes [ts_hdr][frame] to the packet fd.
 *    The kernel constructs an skb, applies HW timestamp,
 *    and delivers it via netif_rx().
 * ============================================================ */

/* Per-skb timestamp metadata consumed by ndo_get_tstamp(). */
struct ptp_proxy_inline_tstamps {
	s64 adjusted_ns;
	s64 raw_ns;
};

/* Dummy NAPI poll — never actually scheduled; only needed for napi_id */
static int ptp_proxy_napi_poll(struct napi_struct *napi, int budget)
{
	napi_complete(napi);
	return 0;
}

static int ptp_proxy_ndev_open(struct net_device *dev)
{
	struct ptp_proxy_netdev_priv *priv = netdev_priv(dev);

	napi_enable(&priv->napi);
	netif_start_queue(dev);
	return 0;
}

static int ptp_proxy_ndev_stop(struct net_device *dev)
{
	struct ptp_proxy_netdev_priv *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	napi_disable(&priv->napi);
	return 0;
}

static ktime_t ptp_proxy_ndev_get_tstamp(struct net_device *dev,
					 const struct skb_shared_hwtstamps *hwtstamps,
					 bool cycles)
{
	const struct ptp_proxy_inline_tstamps *meta;
	s64 ns;

	if (!hwtstamps)
		return 0;

	meta = hwtstamps->netdev_data;
	if (!meta)
		return hwtstamps->hwtstamp;

	ns = cycles ? meta->raw_ns : meta->adjusted_ns;

	return ns_to_ktime(ns);
}

/*
 * ndo_start_xmit — called when ptp4l sends a packet.
 * Queue the skb for the packet fd reader (userspace relay).
 */
static netdev_tx_t ptp_proxy_ndev_xmit(struct sk_buff *skb,
					struct net_device *dev)
{
	struct ptp_proxy_netdev_priv *priv = netdev_priv(dev);
	struct ptp_proxy_inst *inst = priv->inst;
	bool wants_ts = (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) != 0;

	if (!inst->alive) {
		dev_kfree_skb(skb);
		dev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	if (skb_queue_len(&inst->pkt_queue) >= PTP_PROXY_PKT_QUEUE_LEN) {
		pr_warn_ratelimited("ptp_proxy: pkt_queue FULL, dropping (wants_ts=%d, sk=%p)\n",
				    wants_ts, skb->sk);
		dev_kfree_skb(skb);
		dev->stats.tx_dropped++;
		PTP_PROXY_STAT_INC(inst, pkt_queue_drops);
		return NETDEV_TX_OK;
	}

	/* Hot path — no debug logging */

	/* Mark TX timestamp in progress for async delivery */
	if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP)
		skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;

	skb_queue_tail(&inst->pkt_queue, skb);
	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	/* Wake packet fd readers */
	wake_up_interruptible(&inst->pkt_wq);

	return NETDEV_TX_OK;
}

static int ptp_proxy_ndev_eth_ioctl(struct net_device *dev,
				    struct ifreq *ifr, int cmd)
{
	struct hwtstamp_config cfg;

	switch (cmd) {
	case SIOCGHWTSTAMP:
		memset(&cfg, 0, sizeof(cfg));
		cfg.tx_type = HWTSTAMP_TX_ON;
		cfg.rx_filter = HWTSTAMP_FILTER_ALL;
		if (copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)))
			return -EFAULT;
		return 0;

	case SIOCSHWTSTAMP:
		if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
			return -EFAULT;
		/* Accept any configuration — we always timestamp */
		if (copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)))
			return -EFAULT;
		return 0;
	}

	return -EOPNOTSUPP;
}

/* ndo_hwtstamp_set/get accept any configuration; we always timestamp all frames. */
static int ptp_proxy_hwtstamp_set(struct net_device *dev,
				  struct kernel_hwtstamp_config *config,
				  struct netlink_ext_ack *extack)
{
	config->tx_type   = HWTSTAMP_TX_ON;
	config->rx_filter = HWTSTAMP_FILTER_ALL;
	return 0;
}

static int ptp_proxy_hwtstamp_get(struct net_device *dev,
				  struct kernel_hwtstamp_config *config)
{
	config->tx_type   = HWTSTAMP_TX_ON;
	config->rx_filter = HWTSTAMP_FILTER_ALL;
	return 0;
}

static const struct net_device_ops ptp_proxy_ndev_ops = {
	.ndo_open         = ptp_proxy_ndev_open,
	.ndo_stop         = ptp_proxy_ndev_stop,
	.ndo_start_xmit   = ptp_proxy_ndev_xmit,
	.ndo_get_tstamp   = ptp_proxy_ndev_get_tstamp,
	.ndo_eth_ioctl    = ptp_proxy_ndev_eth_ioctl,
	.ndo_hwtstamp_set = ptp_proxy_hwtstamp_set,
	.ndo_hwtstamp_get = ptp_proxy_hwtstamp_get,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 9, 0)
static int ptp_proxy_ethtool_get_ts_info(struct net_device *dev,
					 struct kernel_ethtool_ts_info *info)
#else
static int ptp_proxy_ethtool_get_ts_info(struct net_device *dev,
					 struct ethtool_ts_info *info)
#endif
{
	struct ptp_proxy_netdev_priv *priv = netdev_priv(dev);
	struct ptp_proxy_inst *inst = priv->inst;

	info->so_timestamping =
		SOF_TIMESTAMPING_TX_HARDWARE |
		SOF_TIMESTAMPING_RX_HARDWARE |
		SOF_TIMESTAMPING_RAW_HARDWARE |
		SOF_TIMESTAMPING_BIND_PHC;
	info->tx_types = BIT(HWTSTAMP_TX_ON);
	info->rx_filters = BIT(HWTSTAMP_FILTER_ALL);

	if (inst->ptp_clock)
		info->phc_index = ptp_clock_index(inst->ptp_clock);
	else
		info->phc_index = -1;

	return 0;
}

static const struct ethtool_ops ptp_proxy_ethtool_ops = {
	.get_ts_info = ptp_proxy_ethtool_get_ts_info,
};

static bool
ptp_proxy_pending_tx_add(struct ptp_proxy_inst *inst,
			 struct ptp_proxy_pending_tx *entry)
{
	struct ptp_proxy_pending_tx *pending;
	u32 cookie;
	bool collision;
	unsigned long flags;

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	if (inst->pending_tx_count >= PTP_PROXY_TX_PENDING_LEN) {
		spin_unlock_irqrestore(&inst->pending_tx_lock, flags);
		return false;
	}

	do {
		cookie = ++inst->next_tx_cookie;
		if (cookie == 0)
			cookie = ++inst->next_tx_cookie;

		collision = false;
		list_for_each_entry(pending, &inst->pending_tx_list, list) {
			if (pending->cookie == cookie) {
				collision = true;
				break;
			}
		}
	} while (collision);

	entry->cookie = cookie;
	entry->expires = 0;
	list_add_tail(&entry->list, &inst->pending_tx_list);
	inst->pending_tx_count++;
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);

	return true;
}

static void
ptp_proxy_pending_tx_activate(struct ptp_proxy_inst *inst, u32 cookie)
{
	struct ptp_proxy_pending_tx *entry;
	unsigned long flags;

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	list_for_each_entry(entry, &inst->pending_tx_list, list) {
		if (entry->cookie == cookie) {
			entry->expires = jiffies +
				msecs_to_jiffies(PTP_PROXY_TX_PENDING_TIMEOUT_MS);
			break;
		}
	}
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);
}

static struct ptp_proxy_pending_tx *
ptp_proxy_pending_tx_pop_cookie(struct ptp_proxy_inst *inst, u32 cookie)
{
	struct ptp_proxy_pending_tx *entry;
	struct ptp_proxy_pending_tx *tmp;
	unsigned long flags;

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	list_for_each_entry_safe(entry, tmp, &inst->pending_tx_list, list) {
		if (entry->cookie != cookie)
			continue;
		list_del(&entry->list);
		if (inst->pending_tx_count > 0)
			inst->pending_tx_count--;
		spin_unlock_irqrestore(&inst->pending_tx_lock, flags);
		return entry;
	}
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);

	return NULL;
}

static void
ptp_proxy_pending_tx_expire(struct ptp_proxy_inst *inst)
{
	struct ptp_proxy_pending_tx *entry;
	struct ptp_proxy_pending_tx *tmp;
	LIST_HEAD(expired);
	unsigned long flags;
	unsigned int expired_count = 0;

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	list_for_each_entry_safe(entry, tmp, &inst->pending_tx_list, list) {
		if (entry->expires == 0 || time_before(jiffies, entry->expires))
			continue;
		list_move_tail(&entry->list, &expired);
		inst->pending_tx_count--;
		expired_count++;
	}
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);

	list_for_each_entry_safe(entry, tmp, &expired, list) {
		list_del(&entry->list);
		dev_kfree_skb(entry->skb);
		kfree(entry);
	}

	while (expired_count--)
		PTP_PROXY_STAT_INC(inst, tx_cookie_expired);
}

static void
ptp_proxy_pending_tx_purge(struct ptp_proxy_inst *inst)
{
	LIST_HEAD(tmp_list);
	struct ptp_proxy_pending_tx *entry;
	struct ptp_proxy_pending_tx *tmp;
	unsigned long flags;

	spin_lock_irqsave(&inst->pending_tx_lock, flags);
	list_for_each_entry_safe(entry, tmp, &inst->pending_tx_list, list) {
		list_move_tail(&entry->list, &tmp_list);
	}
	inst->pending_tx_count = 0;
	spin_unlock_irqrestore(&inst->pending_tx_lock, flags);

	list_for_each_entry_safe(entry, tmp, &tmp_list, list) {
		list_del(&entry->list);
		if (entry->skb)
			dev_kfree_skb(entry->skb);
		kfree(entry);
	}
}

/* ============================================================
 *  Packet fd: read/write/poll for PTP packet exchange
 *
 *  read()  → dequeue skb from pkt_queue, prepend ts_hdr
 *  write() → parse ts_hdr + frame, deliver to net_device
 *  poll()  → POLLIN when packets available in pkt_queue
 * ============================================================ */

/* TX_REQUEST is set by the kernel on read(); userspace must not send it. */
#define PTP_PROXY_TS_WRITE_FLAGS_MASK \
	(PTP_PROXY_TS_VALID | PTP_PROXY_TS_TX_ERRQUEUE | PTP_PROXY_TS_RAW_VALID)

static int ptp_proxy_ts_to_ns(s64 sec, s64 nsec, s64 *out_ns)
{
	s64 value;

	if (sec < 0 || nsec < 0 || nsec >= NSEC_PER_SEC)
		return -EINVAL;
	if (check_mul_overflow(sec, (s64)NSEC_PER_SEC, &value) ||
	    check_add_overflow(value, nsec, &value))
		return -EOVERFLOW;

	*out_ns = value;
	return 0;
}

static int ptp_proxy_validate_ts_hdr(const struct ptp_proxy_ts_hdr *hdr)
{
	bool has_ts;

	if (hdr->flags & ~PTP_PROXY_TS_WRITE_FLAGS_MASK)
		return -EINVAL;

	if ((hdr->flags & PTP_PROXY_TS_VALID) &&
	    (hdr->flags & PTP_PROXY_TS_TX_ERRQUEUE))
		return -EINVAL;

	has_ts = hdr->flags &
		 (PTP_PROXY_TS_VALID | PTP_PROXY_TS_TX_ERRQUEUE);
	if (has_ts &&
	    (hdr->sec < 0 || hdr->nsec < 0 || hdr->nsec >= NSEC_PER_SEC))
		return -EINVAL;

	if ((hdr->flags & PTP_PROXY_TS_RAW_VALID) &&
	    (hdr->raw_sec < 0 || hdr->raw_nsec < 0 ||
	     hdr->raw_nsec >= NSEC_PER_SEC))
		return -EINVAL;

	if ((hdr->flags & PTP_PROXY_TS_TX_ERRQUEUE) && hdr->cookie == 0)
		return -EINVAL;

	return 0;
}

static ssize_t ptp_proxy_pkt_read(struct file *filp, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	struct ptp_proxy_ts_hdr hdr;
	struct ptp_proxy_pending_tx *pending = NULL;
	struct sk_buff *skb;
	bool wants_tx_ts;
	size_t frame_len, total;

	if (!inst->alive)
		return -ENODEV;

	/* Wait for a packet */
	if (skb_queue_empty(&inst->pkt_queue)) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		if (wait_event_interruptible(inst->pkt_wq,
			!skb_queue_empty(&inst->pkt_queue) || !inst->alive))
			return -ERESTARTSYS;
		if (!inst->alive)
			return -ENODEV;
	}

	skb = skb_dequeue(&inst->pkt_queue);
	if (!skb)
		return -EAGAIN;

	frame_len = skb->len;
	total = sizeof(hdr) + frame_len;

	if (count < total) {
		/* Not enough buffer — re-queue and fail */
		skb_queue_head(&inst->pkt_queue, skb);
		return -EMSGSIZE;
	}

	/* Build timestamp header */
	memset(&hdr, 0, sizeof(hdr));
	wants_tx_ts = (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) != 0;
	if (wants_tx_ts) {
		pending = kmalloc(sizeof(*pending), GFP_KERNEL);
		if (!pending) {
			PTP_PROXY_STAT_INC(inst, tx_pending_alloc_fail);
			pr_warn_ratelimited("ptp_proxy: pending TX alloc failed, dropping tx timestamp request\n");
			wants_tx_ts = false;
		} else {
			pending->skb = skb;
			INIT_LIST_HEAD(&pending->list);
			if (!ptp_proxy_pending_tx_add(inst, pending)) {
				PTP_PROXY_STAT_INC(inst, tx_pending_queue_full);
				pr_warn_ratelimited("ptp_proxy: pending TX queue full, dropping tx timestamp request\n");
				kfree(pending);
				pending = NULL;
				wants_tx_ts = false;
			} else
				hdr.cookie = pending->cookie;
		}
	}
	if (wants_tx_ts)
		hdr.flags = PTP_PROXY_TS_TX_REQUEST;

	/* Hot path — no debug logging */

	if (copy_to_user(buf, &hdr, sizeof(hdr))) {
		if (pending) {
			struct ptp_proxy_pending_tx *old;

			old = ptp_proxy_pending_tx_pop_cookie(inst, hdr.cookie);
			if (old)
				kfree(old);
		}
		skb_queue_head(&inst->pkt_queue, skb);
		return -EFAULT;
	}
	if (copy_to_user(buf + sizeof(hdr), skb->data, frame_len)) {
		if (pending) {
			struct ptp_proxy_pending_tx *old;

			old = ptp_proxy_pending_tx_pop_cookie(inst, hdr.cookie);
			if (old)
				kfree(old);
		}
		skb_queue_head(&inst->pkt_queue, skb);
		return -EFAULT;
	}
	if (pending)
		ptp_proxy_pending_tx_activate(inst, hdr.cookie);

	if (!(hdr.flags & PTP_PROXY_TS_TX_REQUEST))
		dev_kfree_skb(skb);

	return total;
}

static ssize_t ptp_proxy_pkt_write(struct file *filp,
				   const char __user *buf,
				   size_t count, loff_t *ppos)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	struct ptp_proxy_ts_hdr hdr;
	struct ptp_proxy_inline_tstamps *meta;
	struct sk_buff *skb;
	struct skb_shared_hwtstamps *hwts;
	struct net_device *dev;
	size_t frame_len;

	if (!inst->alive)
		return -ENODEV;

	/* Hold netdev_lock to prevent concurrent unregister_netdev in inst_release. */
	mutex_lock(&inst->netdev_lock);
	if (!inst->netdev) {
		mutex_unlock(&inst->netdev_lock);
		return -ENODEV;
	}
	dev = inst->netdev;
	get_device(&dev->dev);
	mutex_unlock(&inst->netdev_lock);

	if (count < sizeof(hdr)) {
		PTP_PROXY_STAT_INC(inst, pkt_write_invalid);
		put_device(&dev->dev);
		return -EINVAL;
	}

	if (copy_from_user(&hdr, buf, sizeof(hdr))) {
		put_device(&dev->dev);
		return -EFAULT;
	}

	if (ptp_proxy_validate_ts_hdr(&hdr)) {
		PTP_PROXY_STAT_INC(inst, pkt_write_invalid);
		put_device(&dev->dev);
		return -EINVAL;
	}

	frame_len = count - sizeof(hdr);
	if (frame_len < ETH_HLEN || frame_len > ETH_FRAME_LEN + ETH_FCS_LEN) {
		PTP_PROXY_STAT_INC(inst, pkt_write_invalid);
		put_device(&dev->dev);
		return -EINVAL;
	}

	skb = netdev_alloc_skb(dev, frame_len + NET_IP_ALIGN + sizeof(*meta));
	if (!skb) {
		put_device(&dev->dev);
		return -ENOMEM;
	}

	skb_reserve(skb, NET_IP_ALIGN + sizeof(*meta));
	meta = (struct ptp_proxy_inline_tstamps *)(skb->data - sizeof(*meta));

	if (copy_from_user(skb_put(skb, frame_len),
			   buf + sizeof(hdr), frame_len)) {
		kfree_skb(skb);
		put_device(&dev->dev);
		return -EFAULT;
	}

	skb->protocol = eth_type_trans(skb, dev);
	hwts = skb_hwtstamps(skb);

	/* Default to adjusted domain when raw cycles are unavailable. */
	meta->adjusted_ns = 0;
	meta->raw_ns = 0;

	/* Apply HW RX timestamp if flagged */
	if (hdr.flags & PTP_PROXY_TS_VALID) {
		if (ptp_proxy_ts_to_ns(hdr.sec, hdr.nsec, &meta->adjusted_ns)) {
			kfree_skb(skb);
			put_device(&dev->dev);
			return -EINVAL;
		}
		if (hdr.flags & PTP_PROXY_TS_RAW_VALID) {
			if (ptp_proxy_ts_to_ns(hdr.raw_sec, hdr.raw_nsec,
					       &meta->raw_ns)) {
				kfree_skb(skb);
				put_device(&dev->dev);
				return -EINVAL;
			}
		} else {
			meta->raw_ns = meta->adjusted_ns;
		}

		/*
		 * skb_shared_hwtstamps is a union: hwtstamp and netdev_data
		 * share the same 8 bytes. Set SKBTX_HW_TSTAMP_NETDEV so the
		 * kernel calls ndo_get_tstamp() and we can return adjusted_ns
		 * (base PHC) or cycles_ns (vclock/BIND_PHC) as needed.
		 */
		skb_shinfo(skb)->tx_flags |= SKBTX_HW_TSTAMP_NETDEV;
		hwts->netdev_data = meta;
	}

	/* TX timestamp feedback via error queue */
	if (hdr.flags & PTP_PROXY_TS_TX_ERRQUEUE) {
		struct skb_shared_hwtstamps hwts_tx;
		struct sk_buff *tx_skb = NULL;
		struct ptp_proxy_pending_tx *pending = NULL;
		struct ptp_proxy_inline_tstamps *tx_meta;
		s64 ts_sec, ts_nsec;
		s64 tx_adj_ns, tx_cycles_ns;

		/*
		 * TX errqueue timestamps now carry both adjusted and cycles
		 * domains in per-skb metadata. The socket layer will call
		 * ndo_get_tstamp(cycles=false/true) to select which one to use.
		 * Default behavior (no BIND_PHC) consumes adjusted domain.
		 */
		ts_sec = hdr.sec;
		ts_nsec = hdr.nsec;
		if (ptp_proxy_ts_to_ns(ts_sec, ts_nsec, &tx_adj_ns)) {
			kfree_skb(skb);
			put_device(&dev->dev);
			return -EINVAL;
		}

		/* Extract cycles if provided, else fallback to adjusted */
		if (hdr.flags & PTP_PROXY_TS_RAW_VALID) {
			if (ptp_proxy_ts_to_ns(hdr.raw_sec, hdr.raw_nsec,
					       &tx_cycles_ns)) {
				kfree_skb(skb);
				put_device(&dev->dev);
				return -EINVAL;
			}
		} else {
			tx_cycles_ns = tx_adj_ns;
		}

		if (hdr.cookie != 0)
			pending = ptp_proxy_pending_tx_pop_cookie(inst, hdr.cookie);

		if (pending) {
			tx_skb = pending->skb;
			kfree(pending);
		}

		if (tx_skb) {
			/*
			 * Store metadata in skb->cb[] (control buffer, 48 bytes).
			 * It will survive errqueue delivery to socket layer where
			 * ndo_get_tstamp() will be called to select adjusted or cycles.
			 */
			tx_meta = (struct ptp_proxy_inline_tstamps *)tx_skb->cb;
			tx_meta->adjusted_ns = tx_adj_ns;
			tx_meta->raw_ns = tx_cycles_ns;

			/* Enable ndo_get_tstamp callback for timestamp selection */
			skb_shinfo(tx_skb)->tx_flags |= SKBTX_HW_TSTAMP_NETDEV;
			hwts_tx.netdev_data = tx_meta;

			/* Fallback for older code paths that don't use callback */
			hwts_tx.hwtstamp = ktime_set(ts_sec, ts_nsec);

			skb_tstamp_tx(tx_skb, &hwts_tx);
			dev_kfree_skb(tx_skb);
		} else {
			PTP_PROXY_STAT_INC(inst, tx_cookie_unknown);
			pr_warn_ratelimited("ptp_proxy: no pending TX request for cookie %u\n",
					    hdr.cookie);
			kfree_skb(skb);
			dev->stats.rx_dropped++;
			put_device(&dev->dev);
			return -ENOENT;
		}
		kfree_skb(skb);
		dev->stats.rx_packets++;
		dev->stats.rx_bytes += frame_len;

		put_device(&dev->dev);
		return count;
	}

	/* Normal RX delivery */
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += frame_len;
	{
		struct ptp_proxy_netdev_priv *priv = netdev_priv(dev);
		skb_mark_napi_id(skb, &priv->napi);
	}
	if (netif_rx(skb) != NET_RX_SUCCESS) {
		PTP_PROXY_STAT_INC(inst, pkt_rx_drops);
		dev->stats.rx_dropped++;
		pr_warn_ratelimited("ptp_proxy: netif_rx dropped injected packet\n");
	}

	put_device(&dev->dev);
	return count;
}

static __poll_t ptp_proxy_pkt_poll(struct file *filp,
				   struct poll_table_struct *wait)
{
	struct ptp_proxy_inst *inst = filp->private_data;
	__poll_t mask = EPOLLOUT; /* always writable */

	poll_wait(filp, &inst->pkt_wq, wait);

	if (!skb_queue_empty(&inst->pkt_queue))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (!inst->alive)
		mask |= EPOLLHUP;

	return mask;
}

static int ptp_proxy_pkt_release(struct inode *inode, struct file *filp)
{
	struct ptp_proxy_inst *inst = filp->private_data;

	ptp_proxy_inst_put(inst);
	return 0;
}

static const struct file_operations ptp_proxy_pkt_fops = {
	.owner   = THIS_MODULE,
	.read    = ptp_proxy_pkt_read,
	.write   = ptp_proxy_pkt_write,
	.poll    = ptp_proxy_pkt_poll,
	.release = ptp_proxy_pkt_release,
};

/*
 * Create net_device + packet fd for an instance.
 * Called via ioctl on the instance fd.
 */
static int ptp_proxy_create_netdev(struct ptp_proxy_inst *inst,
				   struct ptp_proxy_netdev_create __user *uarg)
{
	struct net_device *dev;
	struct ptp_proxy_netdev_priv *priv;
	struct ptp_proxy_netdev_create create;
	int idx, fd, ret;

	if (!ns_capable(&init_user_ns, CAP_NET_ADMIN))
		return -EPERM;

	mutex_lock(&inst->netdev_lock);
	if (inst->netdev_created) {
		ret = -EEXIST;
		goto out_unlock;
	}

	/* Read user request (may contain desired ifname) */
	memset(&create, 0, sizeof(create));
	if (copy_from_user(&create, uarg, sizeof(create))) {
		ret = -EFAULT;
		goto out_unlock;
	}

	idx = atomic_fetch_add(1, &ptp_proxy_netdev_count);

	/* Allocate Ethernet net_device */
	dev = alloc_etherdev(sizeof(struct ptp_proxy_netdev_priv));
	if (!dev) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	/* Use user-supplied ifname if provided, otherwise default */
	if (create.ifname[0] != '\0') {
		create.ifname[sizeof(create.ifname) - 1] = '\0';
		strscpy(dev->name, create.ifname, IFNAMSIZ);
	} else {
		snprintf(dev->name, IFNAMSIZ, "ptpproxy%d", idx);
	}
	dev->netdev_ops = &ptp_proxy_ndev_ops;
	dev->ethtool_ops = &ptp_proxy_ethtool_ops;
	dev->needs_free_netdev = true;

	/* Generate a random MAC */
	eth_hw_addr_random(dev);

	priv = netdev_priv(dev);
	priv->inst = inst;

	/* Register NAPI — needed for SKBTX_HW_TSTAMP_NETDEV to work
	 * (kernel uses napi_id to resolve device in get_timestamp()) */
	netif_napi_add(dev, &priv->napi, ptp_proxy_napi_poll);

	/* Initialize pending TX tracking */
	INIT_LIST_HEAD(&inst->pending_tx_list);
	inst->pending_tx_count = 0;
	inst->next_tx_cookie = 0;

	ret = register_netdev(dev);
	if (ret) {
		free_netdev(dev);
		goto out_unlock;
	}

	inst->netdev = dev;
	inst->netdev_created = true;

	/* pkt fd holds an extra instance reference. */
	if (!ptp_proxy_inst_get(inst)) {
		/* refcount already zero — inst is being torn down concurrently */
		unregister_netdev(dev);
		ret = -ENODEV;
		goto out_unlock;
	}

	/* Create anonymous packet fd */
	fd = anon_inode_getfd("ptp_proxy_pkt", &ptp_proxy_pkt_fops,
			      inst, O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		ptp_proxy_inst_put(inst);
		unregister_netdev(dev);
		inst->netdev = NULL;
		inst->netdev_created = false;
		ret = fd;
		goto out_unlock;
	}

	/* Return interface name to userspace */
	memset(&create, 0, sizeof(create));
	strscpy(create.ifname, dev->name, sizeof(create.ifname));

	if (copy_to_user(uarg, &create, sizeof(create))) {
		mutex_unlock(&inst->netdev_lock);
		return -EFAULT;  /* fd already installed */
	}

	pr_info("ptp_proxy: created net_device %s (phc=/dev/ptp%d)\n",
		dev->name, ptp_clock_index(inst->ptp_clock));
	mutex_unlock(&inst->netdev_lock);

	return fd;

out_unlock:
	mutex_unlock(&inst->netdev_lock);
	return ret;
}

/* ============================================================
 *  IPv4 address notification (inetaddr notifier)
 *
 *  When ptpproxyN gets an IPv4 address added or removed, we retain the
 *  latest NOTIFY_IPV4 state until DPDK reads it. Since userspace consumes
 *  only the current address, repeated changes are coalesced into one entry.
 * ============================================================ */

/*
 * Retain the latest notification for userspace. The queue is bounded to one
 * entry because NOTIFY_IPV4 supersedes any older address state.
 */
static void ptp_proxy_queue_notification(struct ptp_proxy_inst *inst,
					 const struct ptp_proxy_request *req)
{
	struct ptp_proxy_notify *pending;
	struct ptp_proxy_notify *n;

	PTP_PROXY_STAT_INC(inst, notify_ipv4_queued);

	spin_lock_bh(&inst->notify_lock);
	if (!list_empty(&inst->notify_queue)) {
		pending = list_first_entry(&inst->notify_queue,
					   struct ptp_proxy_notify, list);
		pending->req = *req;
		inst->notify_seq++;
		spin_unlock_bh(&inst->notify_lock);
		PTP_PROXY_STAT_INC(inst, notify_ipv4_replaced);
		wake_up_interruptible(&inst->cmd_wq);
		return;
	}
	spin_unlock_bh(&inst->notify_lock);

	n = kmalloc(sizeof(*n), GFP_KERNEL);
	if (!n) {
		PTP_PROXY_STAT_INC(inst, notify_alloc_fail);
		pr_warn_ratelimited("ptp_proxy: failed to allocate notification\n");
		return;
	}

	n->req = *req;

	spin_lock_bh(&inst->notify_lock);
	if (!list_empty(&inst->notify_queue)) {
		pending = list_first_entry(&inst->notify_queue,
					   struct ptp_proxy_notify, list);
		pending->req = *req;
		inst->notify_seq++;
		spin_unlock_bh(&inst->notify_lock);
		kfree(n);
		PTP_PROXY_STAT_INC(inst, notify_ipv4_replaced);
	} else {
		list_add_tail(&n->list, &inst->notify_queue);
		inst->notify_seq++;
		spin_unlock_bh(&inst->notify_lock);
	}

	/* Wake userspace (may be blocking in read() or poll()) */
	wake_up_interruptible(&inst->cmd_wq);
}

/*
 * inetaddr notifier callback — called when any net_device gets an
 * IPv4 address added (NETDEV_UP) or removed (NETDEV_DOWN).
 *
 * We match against our ptpproxy net_devices and post NOTIFY_IPV4
 * with the new address (or 0 on removal).
 */
static int ptp_proxy_inetaddr_event(struct notifier_block *nb,
				    unsigned long event, void *ptr)
{
	struct in_ifaddr *ifa = ptr;
	struct net_device *dev = ifa->ifa_dev->dev;
	struct ptp_proxy_inst *inst;
	struct ptp_proxy_inst *target = NULL;
	struct ptp_proxy_request req;

	/* Only care about our ptpproxy net_devices */
	mutex_lock(&ptp_proxy_inst_lock);
	list_for_each_entry(inst, &ptp_proxy_inst_list, list) {
		if (inst->netdev == dev) {
			if (ptp_proxy_inst_get(inst))
				target = inst;
			break;
		}
	}
	mutex_unlock(&ptp_proxy_inst_lock);

	if (!target)
		return NOTIFY_DONE;

	memset(&req, 0, sizeof(req));
	req.cmd = PTP_PROXY_CMD_NOTIFY_IPV4;

	switch (event) {
	case NETDEV_UP:
		req.ipv4_addr = ifa->ifa_local;  /* network byte order */
		break;
	case NETDEV_DOWN:
		req.ipv4_addr = 0;  /* address removed */
		break;
	default:
		ptp_proxy_inst_put(target);
		return NOTIFY_DONE;
	}

	if (READ_ONCE(target->alive))
		ptp_proxy_queue_notification(target, &req);
	ptp_proxy_inst_put(target);

	return NOTIFY_DONE;
}

static struct notifier_block ptp_proxy_inetaddr_nb = {
	.notifier_call = ptp_proxy_inetaddr_event,
};

/* ============================================================
 *  Instance fd ioctl handler (for CREATE_NETDEV)
 * ============================================================ */

static long ptp_proxy_inst_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	struct ptp_proxy_inst *inst = filp->private_data;

	switch (cmd) {
	case PTP_PROXY_IOC_CREATE_NETDEV:
		return ptp_proxy_create_netdev(
			inst, (struct ptp_proxy_netdev_create __user *)arg);
	case PTP_PROXY_IOC_CLAIM_DISCIPLINER:
		return ptp_proxy_claim_discipliner(inst);
	case PTP_PROXY_IOC_RELEASE_DISCIPLINER:
		ptp_proxy_release_discipliner(inst);
		return 0;
	default:
		return -ENOTTY;
	}
}

/* ============================================================
 *  Control device: /dev/ptp_proxy
 * ============================================================ */

static long ptp_proxy_ioctl(struct file *filp, unsigned int cmd,
			    unsigned long arg)
{
	struct ptp_proxy_create create;
	struct ptp_proxy_inst *inst;
	int fd, ret;

	if (cmd != PTP_PROXY_IOC_CREATE)
		return -ENOTTY;
	if (!ns_capable(&init_user_ns, CAP_SYS_TIME))
		return -EPERM;

	if (copy_from_user(&create, (void __user *)arg, sizeof(create)))
		return -EFAULT;

	/* Allocate per-instance state */
	inst = kzalloc(sizeof(*inst), GFP_KERNEL);
	if (!inst)
		return -ENOMEM;

	mutex_init(&inst->cmd_lock);
	mutex_init(&inst->discipliner_lock);
	spin_lock_init(&inst->cmd_state_lock);
	init_waitqueue_head(&inst->cmd_wq);
	init_waitqueue_head(&inst->resp_wq);
	init_waitqueue_head(&inst->pkt_wq);
	skb_queue_head_init(&inst->pkt_queue);
	spin_lock_init(&inst->pending_tx_lock);
	INIT_LIST_HEAD(&inst->pending_tx_list);
	inst->next_tx_cookie = 0;
	inst->pending_tx_count = 0;
	spin_lock_init(&inst->cycles_lock);
	inst->cycles_valid = false;
	INIT_LIST_HEAD(&inst->notify_queue);
	spin_lock_init(&inst->notify_lock);
	INIT_DELAYED_WORK(&inst->cycles_work, ptp_proxy_cycles_refresh_work);
	mutex_init(&inst->netdev_lock);
	refcount_set(&inst->refcnt, 1);
	inst->alive = true;

	/* Register virtual PTP clock with the PTP subsystem */
	inst->ptp_info = (struct ptp_clock_info) {
		.owner     = THIS_MODULE,
		.name      = "ptp_proxy",
		.max_adj   = create.max_adj > 0 ? create.max_adj
						 : PTP_PROXY_DEFAULT_MAX_ADJ,
		.n_alarm   = 0,
		.n_ext_ts  = 0,
		.n_per_out = 0,
		.n_pins    = 0,
		.pps       = 0,
		.gettime64     = ptp_proxy_gettime64,
		.gettimex64    = ptp_proxy_gettimex64,
		.getcycles64   = ptp_proxy_getcycles64,
		.getcyclesx64  = ptp_proxy_getcyclesx64,
		.getcrosststamp = ptp_proxy_getcrosststamp,
		.getcrosscycles = ptp_proxy_getcrosscycles,
		.settime64     = ptp_proxy_settime64,
		.adjtime       = ptp_proxy_adjtime,
		.adjfine       = ptp_proxy_adjfine,
	};

	inst->ptp_clock = ptp_clock_register(&inst->ptp_info, NULL);
	if (IS_ERR(inst->ptp_clock)) {
		ret = PTR_ERR(inst->ptp_clock);
		inst->ptp_clock = NULL;
		goto err_free;
	}

	create.ptp_index = ptp_clock_index(inst->ptp_clock);

	/*
	 * Create an anonymous fd for this instance.
	 * The fd is installed in the current process's fd table.
	 * Closing the fd destroys the PTP clock (via release()).
	 * This follows the KVM_CREATE_VM pattern.
	 */
	mutex_lock(&ptp_proxy_inst_lock);
	list_add(&inst->list, &ptp_proxy_inst_list);
	mutex_unlock(&ptp_proxy_inst_lock);

	fd = anon_inode_getfd("ptp_proxy", &ptp_proxy_inst_fops, inst,
			      O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		mutex_lock(&ptp_proxy_inst_lock);
		list_del(&inst->list);
		mutex_unlock(&ptp_proxy_inst_lock);
		ret = fd;
		goto err_ptp;
	}

	/* Return the ptp_index to userspace */
	if (copy_to_user((void __user *)arg, &create, sizeof(create))) {
		/*
		 * The fd is already installed. Returning -EFAULT here
		 * means the ioctl "failed" but the fd exists. The user
		 * should close unknown fds. Same pattern as KVM.
		 */
		return -EFAULT;
	}

	pr_info("ptp_proxy: created /dev/ptp%d (fd=%d)\n",
		create.ptp_index, fd);

	ptp_proxy_debugfs_create(inst, create.ptp_index);

	/* Start periodic getcycles cache refresh */
	schedule_delayed_work(&inst->cycles_work,
			      msecs_to_jiffies(PTP_PROXY_CYCLES_REFRESH_MS));

	return fd;

err_ptp:
	ptp_clock_unregister(inst->ptp_clock);
err_free:
	kfree(inst);
	return ret;
}

static const struct file_operations ptp_proxy_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = ptp_proxy_ioctl,
};

/* ============================================================
 *  Module init / exit
 * ============================================================ */

static struct miscdevice ptp_proxy_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = DRIVER_NAME,
	.fops  = &ptp_proxy_fops,
};

static int __init ptp_proxy_module_init(void)
{
	int ret;

	ret = misc_register(&ptp_proxy_misc);
	if (ret) {
		pr_err("ptp_proxy: failed to register misc device: %d\n", ret);
		return ret;
	}

	ret = register_inetaddr_notifier(&ptp_proxy_inetaddr_nb);
	if (ret) {
		pr_err("ptp_proxy: failed to register IPv4 notifier: %d\n", ret);
		misc_deregister(&ptp_proxy_misc);
		return ret;
	}

	ptp_proxy_debugfs_root = debugfs_create_dir(DRIVER_NAME, NULL);
	if (IS_ERR(ptp_proxy_debugfs_root))
		ptp_proxy_debugfs_root = NULL;

	pr_info("ptp_proxy: loaded v%s (/dev/%s)\n",
		DRIVER_VERSION, DRIVER_NAME);
	return 0;
}

static void __exit ptp_proxy_module_exit(void)
{
	unregister_inetaddr_notifier(&ptp_proxy_inetaddr_nb);
	misc_deregister(&ptp_proxy_misc);
	debugfs_remove_recursive(ptp_proxy_debugfs_root);
	ptp_proxy_debugfs_root = NULL;

	if (!list_empty(&ptp_proxy_inst_list))
		pr_warn("ptp_proxy: %s unloaded with orphaned instances!\n",
			DRIVER_VERSION);

	pr_info("ptp_proxy: unloaded\n");
}

module_init(ptp_proxy_module_init);
module_exit(ptp_proxy_module_exit);
