// SPDX-License-Identifier: GPL-2.0
/*
 * Speaker protection guard for the Surface Pro 11.
 *
 * A speaker stream may be opened and prepared while all of this holds:
 *  - the closed-loop data (control links and calibration records) was loaded,
 *  - no fault was latched, and "sp11.nospk" is not set on the kernel command
 *    line.
 * The amplifier may be UNMUTED (power amplifier enabled) only while, in
 * addition:
 *  - the closed-loop AudioReach graph (speaker protection module and its
 *    control links) runs,
 *  - the V/I sense capture stream is running (and the CPS stream, when the
 *    topology asks for it),
 *  - a calibration (R0/T0) was applied to both speakers.
 * The first group lets user space start the V/I stream and the speaker graph
 * with the amplifier muted; the second group is enforced by the amplifier
 * driver on every unmute path. When the second group stops holding while a
 * stream runs, the amplifiers are muted again.
 * Every input is written from kernel code only. The module parameters are
 * read-only (0444) so they can be set at boot, never at run time.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <sound/sp11-guard.h>

#undef MODULE_PARAM_PREFIX
#define MODULE_PARAM_PREFIX "sp11."

static bool nospk;
module_param(nospk, bool, 0444);
MODULE_PARM_DESC(nospk, "Kill switch: never allow speaker playback (boot-time only)");

static bool allow_nominal;
module_param(allow_nominal, bool, 0444);
MODULE_PARM_DESC(allow_nominal,
		 "Accept a nominal (not measured) R0/T0 start (boot-time only)");

struct sp11_guard {
	spinlock_t lock;	/* protects all fields below */
	unsigned int streams[SP11_STREAM_MAX];
	unsigned int cal_mask;
	bool cal_nominal;
	bool loaded;
	bool graph_up;
	bool cps_required;
	bool fault;
	char fault_src[24];
	u32 fault_code;
	void (*mute)(void *data);
	void *mute_data;
	struct work_struct mute_work;
	struct dentry *debugfs;
};

static struct sp11_guard guard = {
	.lock = __SPIN_LOCK_UNLOCKED(guard.lock),
};

/* Reason a stream may not be opened, NULL when it may. Called with guard.lock held. */
static const char *sp11_guard_open_reason(void)
{
	if (nospk)
		return "sp11.nospk is set";
	if (guard.fault)
		return "a speaker fault is latched";
	if (!guard.loaded)
		return "the closed-loop data is not loaded";

	return NULL;
}

/* Reason the amplifier may not be unmuted, NULL when it may. Called with guard.lock held. */
static const char *sp11_guard_reason(void)
{
	const char *reason = sp11_guard_open_reason();

	if (reason)
		return reason;
	if (!guard.graph_up)
		return "the closed-loop graph is not up";
	if (!guard.streams[SP11_STREAM_VI])
		return "the V/I sense stream is not running";
	if (guard.cps_required && !guard.streams[SP11_STREAM_CPS])
		return "the CPS stream is not running";
	if ((guard.cal_mask & SP11_CAL_ALL_MASK) != SP11_CAL_ALL_MASK)
		return "no R0/T0 calibration was applied";
	if (guard.cal_nominal && !allow_nominal)
		return "only a nominal calibration exists and sp11.allow_nominal is not set";

	return NULL;
}

/*
 * Mute the amplifiers, unless the guard allows playback again by the time the
 * worker runs (a stream that stops and restarts quickly must not be muted by
 * the stop of its previous run).
 */
static void sp11_guard_mute_work(struct work_struct *work)
{
	void (*mute)(void *data);
	void *data;
	bool allowed;

	spin_lock_irq(&guard.lock);
	mute = guard.mute;
	data = guard.mute_data;
	allowed = !sp11_guard_reason();
	spin_unlock_irq(&guard.lock);

	if (mute && !allowed)
		mute(data);
}

/* Ask for a mute when a state change made the guard refuse. Called with guard.lock held. */
static void sp11_guard_mute_if_refused(bool was_allowed)
{
	if (was_allowed && sp11_guard_reason())
		schedule_work(&guard.mute_work);
}

void sp11_guard_loaded(bool loaded)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	guard.loaded = loaded;
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_loaded);

void sp11_guard_graph(bool up, bool cps_required)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	guard.graph_up = up;
	guard.cps_required = up && cps_required;
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_graph);

void sp11_guard_stream(enum sp11_guard_stream stream, bool start)
{
	unsigned long flags;
	bool was_allowed;

	if (stream >= SP11_STREAM_MAX)
		return;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	if (start) {
		guard.streams[stream]++;
	} else if (guard.streams[stream] && !--guard.streams[stream] &&
		   stream == SP11_STREAM_VI) {
		/* The calibration lives in the V/I module of the graph that just stopped */
		guard.cal_mask = 0;
		guard.cal_nominal = false;
	}
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_stream);

void sp11_guard_calibrated(unsigned int ch_mask, bool nominal)
{
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	guard.cal_mask |= ch_mask & SP11_CAL_ALL_MASK;
	guard.cal_nominal = nominal;
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_calibrated);

void sp11_guard_fault(const char *src, u32 code)
{
	unsigned long flags;
	bool first;

	spin_lock_irqsave(&guard.lock, flags);
	first = !guard.fault;
	if (first) {
		guard.fault = true;
		guard.fault_code = code;
		strscpy(guard.fault_src, src, sizeof(guard.fault_src));
	}
	spin_unlock_irqrestore(&guard.lock, flags);

	if (first) {
		pr_err("sp11: speaker fault latched (%s, 0x%08x), speakers muted until reboot\n",
		       src, code);
		/* Mute from a worker: the producers may run in atomic context */
		schedule_work(&guard.mute_work);
	}
}
EXPORT_SYMBOL_GPL(sp11_guard_fault);

void sp11_guard_set_mute(void (*mute)(void *data), void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	guard.mute = mute;
	guard.mute_data = data;
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_set_mute);

void sp11_guard_clear_mute(void *data)
{
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	if (guard.mute_data == data) {
		guard.mute = NULL;
		guard.mute_data = NULL;
	}
	spin_unlock_irqrestore(&guard.lock, flags);
	flush_work(&guard.mute_work);
}
EXPORT_SYMBOL_GPL(sp11_guard_clear_mute);

bool sp11_guard_faulted(void)
{
	return READ_ONCE(guard.fault);
}
EXPORT_SYMBOL_GPL(sp11_guard_faulted);

bool sp11_guard_nospk(void)
{
	return nospk;
}
EXPORT_SYMBOL_GPL(sp11_guard_nospk);

/* May a speaker stream be opened and prepared (the amplifier stays muted)? */
int sp11_guard_check(struct device *dev)
{
	const char *reason;
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	reason = sp11_guard_open_reason();
	spin_unlock_irqrestore(&guard.lock, flags);

	if (!reason)
		return 0;

	dev_warn_ratelimited(dev, "speaker playback refused: %s\n", reason);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL_GPL(sp11_guard_check);

/* May the amplifier be unmuted (power amplifier enabled)? */
int sp11_guard_unmute_check(struct device *dev)
{
	const char *reason;
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	reason = sp11_guard_reason();
	spin_unlock_irqrestore(&guard.lock, flags);

	if (!reason)
		return 0;

	dev_warn_ratelimited(dev, "speaker stays muted: %s\n", reason);

	return -EOPNOTSUPP;
}
EXPORT_SYMBOL_GPL(sp11_guard_unmute_check);

static int sp11_guard_show(struct seq_file *s, void *unused)
{
	const char *reason;

	spin_lock_irq(&guard.lock);
	reason = sp11_guard_reason();
	seq_printf(s, "open_allowed: %s\n", sp11_guard_open_reason() ? "no" : "yes");
	seq_printf(s, "allowed: %s\n", reason ? "no" : "yes");
	seq_printf(s, "reason: %s\n", reason ?: "-");
	seq_printf(s, "loaded: %d\ngraph_up: %d\nvi_streams: %u\ncps_streams: %u\ncps_required: %d\n",
		   guard.loaded, guard.graph_up, guard.streams[SP11_STREAM_VI],
		   guard.streams[SP11_STREAM_CPS], guard.cps_required);
	seq_printf(s, "cal_mask: 0x%x\ncal_nominal: %d\n", guard.cal_mask, guard.cal_nominal);
	seq_printf(s, "fault: %d (%s, 0x%08x)\nnospk: %d\nallow_nominal: %d\n",
		   guard.fault, guard.fault ? guard.fault_src : "-", guard.fault_code,
		   nospk, allow_nominal);
	spin_unlock_irq(&guard.lock);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(sp11_guard);

static int __init sp11_guard_init(void)
{
	INIT_WORK(&guard.mute_work, sp11_guard_mute_work);
	guard.debugfs = debugfs_create_dir("sp11", NULL);
	debugfs_create_file("guard", 0444, guard.debugfs, NULL, &sp11_guard_fops);

	return 0;
}
module_init(sp11_guard_init);

static void __exit sp11_guard_exit(void)
{
	debugfs_remove_recursive(guard.debugfs);
	cancel_work_sync(&guard.mute_work);
}
module_exit(sp11_guard_exit);

MODULE_DESCRIPTION("Surface Pro 11 speaker protection guard");
MODULE_LICENSE("GPL");
