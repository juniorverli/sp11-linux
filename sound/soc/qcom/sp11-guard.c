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

/*
 * This file is a module (CONFIG_SND_SOC_SP11_GUARD=m). The module loader splits
 * "sp11.allow_nominal=1" at the dot, matches "sp11" as the module name and looks
 * for the bare parameter name; a non-empty prefix here made the stored name
 * "sp11.allow_nominal" and the loader ignored the boot parameter ("sp11: unknown
 * parameter 'allow_nominal' ignored", device boot 2026-11-05). Built-in builds
 * keep the prefix so "sp11.nospk" still works on the kernel command line.
 */
#undef MODULE_PARAM_PREFIX
#ifdef MODULE
#define MODULE_PARAM_PREFIX ""
#else
#define MODULE_PARAM_PREFIX "sp11."
#endif

static bool nospk;
module_param(nospk, bool, 0444);
MODULE_PARM_DESC(nospk, "Kill switch: never allow speaker playback (boot-time only)");

static bool allow_nominal;
module_param(allow_nominal, bool, 0444);
MODULE_PARM_DESC(allow_nominal,
		 "Accept a nominal (not measured) R0/T0 start (boot-time only)");

enum sp11_mode {
	SP11_MODE_UNKNOWN,
	SP11_MODE_CLOSED_LOOP,
	SP11_MODE_OPEN_LOOP,
	SP11_MODE_REFUSED,
};

struct sp11_guard {
	enum sp11_mode mode;
	bool caps_ready;
	spinlock_t lock;	/* protects all fields below */
	unsigned int streams[SP11_STREAM_MAX];
	unsigned int cal_mask;
	bool cal_nominal;
	const void *cal_graph;
	const void *vi_graph;
	u64 cal_generation;
	bool vi_ambiguous;
	bool loaded;
	bool graph_up;
	bool cps_required;
	bool sp_enabled;
	bool diagnostics_degraded;
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
int sp11_guard_select_mode(bool marker_valid, bool has_sp)
{
	unsigned long flags;
	enum sp11_mode mode;

	if (!marker_valid)
		mode = SP11_MODE_REFUSED;
	else if (has_sp)
		mode = SP11_MODE_CLOSED_LOOP;
	else
		mode = SP11_MODE_OPEN_LOOP;

	spin_lock_irqsave(&guard.lock, flags);
	if (guard.mode != SP11_MODE_UNKNOWN && guard.mode != mode) {
		guard.mode = SP11_MODE_REFUSED;
		spin_unlock_irqrestore(&guard.lock, flags);
		return -EINVAL;
	}
	guard.mode = mode;
	spin_unlock_irqrestore(&guard.lock, flags);

	pr_info_once("sp11: selected %s speaker mode\n",
		     mode == SP11_MODE_OPEN_LOOP ? "D26 open-loop" :
		     mode == SP11_MODE_CLOSED_LOOP ? "closed-loop" : "refused");
	return mode == SP11_MODE_REFUSED ? -EOPNOTSUPP : 0;
}
EXPORT_SYMBOL_GPL(sp11_guard_select_mode);

void sp11_guard_caps_ready(bool ready)
{
	unsigned long flags;

	spin_lock_irqsave(&guard.lock, flags);
	guard.caps_ready = ready;
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_caps_ready);

static const char *sp11_guard_open_reason(void)
{
	if (guard.mode == SP11_MODE_UNKNOWN || guard.mode == SP11_MODE_REFUSED)
		return "speaker topology mode is unknown or refused";
	if (!guard.caps_ready)
		return "PA, RX or MIX volume caps are not installed";
	if (nospk)
		return "sp11.nospk is set";
	if (guard.fault)
		return "a speaker fault is latched";
	if (guard.mode == SP11_MODE_CLOSED_LOOP && !guard.loaded)
		return "the closed-loop data is not loaded";

	return NULL;
}

/* The graph identities are compared only; the guard never dereferences them. */
static bool sp11_guard_cal_matches_vi(void)
{
	return !guard.vi_ambiguous && guard.vi_graph && guard.vi_graph == guard.cal_graph;
}

/* Reason the amplifier may not be unmuted, NULL when it may. Called with guard.lock held. */
static bool sp11_guard_degraded_fault(void);

static const char *sp11_guard_reason(void)
{
	const char *reason = sp11_guard_open_reason();

	if (reason)
		return reason;
	if (guard.mode == SP11_MODE_OPEN_LOOP)
		return NULL;
	if (sp11_guard_degraded_fault())
		return "degraded diagnostics prerequisites are incomplete";
	if (!guard.graph_up)
		return "the closed-loop graph is not up";
	if (!guard.sp_enabled)
		return "the speaker protection module is not enabled";
	if (guard.diagnostics_degraded && guard.cal_nominal)
		return "degraded diagnostics require closed-loop calibration";
	if (!guard.streams[SP11_STREAM_VI])
		return "the V/I sense stream is not running";
	if (guard.cps_required && !guard.streams[SP11_STREAM_CPS])
		return "the CPS stream is not running";
	if (!sp11_guard_cal_matches_vi())
		return "calibration does not match the running V/I graph";
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

/* Called with guard.lock held. Also invalidates an in-flight SET_CFG. */
static void sp11_guard_calibration_reset_locked(void)
{
	guard.cal_mask = 0;
	guard.cal_nominal = false;
	guard.cal_graph = NULL;
	guard.cal_generation++;
}

void sp11_guard_stream(enum sp11_guard_stream stream, bool start, const void *graph)
{
	unsigned long flags;
	bool was_allowed;

	if (stream >= SP11_STREAM_MAX)
		return;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	if (start) {
		if (stream == SP11_STREAM_VI) {
			if (!guard.streams[stream])
				guard.vi_graph = graph;
			else if (guard.vi_graph != graph)
				guard.vi_ambiguous = true;
			if (!graph)
				guard.vi_ambiguous = true;
		}
		guard.streams[stream]++;
	} else if (guard.streams[stream]) {
		if (stream == SP11_STREAM_VI && guard.vi_graph != graph)
			guard.vi_ambiguous = true;
		if (!--guard.streams[stream] && stream == SP11_STREAM_VI) {
			guard.vi_graph = NULL;
			guard.vi_ambiguous = false;
			sp11_guard_calibration_reset_locked();
		}
	}
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_stream);

/*
 * Measured VI calibration reported by the DSP (EVENT_ID_VI_CALIBRATION and
 * EVENT_ID_VI_PER_SPKR_CALIBRATION). The payload layouts are taken from the
 * Windows ACDB driver's parser (REA 2026-11-05): a count, a state where 4
 * means success, and one R0 (Q24) per channel. A valid event is accepted as a
 * measured (non-nominal) calibration through the existing generation path;
 * graph identity, caps, CPS policy and the fault latch still gate unmute.
 * An invalid event changes nothing.
 */
void sp11_guard_vi_calibration(const void *graph, unsigned int ch_mask, bool valid)
{
	u64 generation;

	if (!valid || !graph || !(ch_mask & SP11_CAL_ALL_MASK))
		return;
	generation = sp11_guard_calibration_begin(graph);
	if (!generation || !sp11_guard_calibrated(generation, ch_mask & SP11_CAL_ALL_MASK, false)) {
		pr_warn("sp11-guard: measured VI calibration rejected (graph or generation)\n");
		return;
	}
	pr_info("sp11-guard: measured VI calibration accepted, mask 0x%x\n",
		ch_mask & SP11_CAL_ALL_MASK);
}
EXPORT_SYMBOL_GPL(sp11_guard_vi_calibration);

void sp11_guard_calibration_reset(void)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	sp11_guard_calibration_reset_locked();
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_calibration_reset);

void sp11_guard_vi_closed(const void *graph)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	/* A close without a matching stop must not leave a usable graph identity. */
	if (guard.streams[SP11_STREAM_VI] && guard.vi_graph == graph)
		guard.vi_ambiguous = true;
	sp11_guard_calibration_reset_locked();
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_vi_closed);

u64 sp11_guard_calibration_begin(const void *graph)
{
	unsigned long flags;
	bool was_allowed;
	u64 generation;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	sp11_guard_calibration_reset_locked();
	guard.cal_graph = graph;
	generation = guard.cal_generation;
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);

	return generation;
}
EXPORT_SYMBOL_GPL(sp11_guard_calibration_begin);

bool sp11_guard_calibrated(u64 generation, unsigned int ch_mask, bool nominal)
{
	unsigned long flags;
	bool accepted;

	spin_lock_irqsave(&guard.lock, flags);
	accepted = guard.cal_graph && guard.cal_generation == generation;
	if (accepted) {
		guard.cal_mask |= ch_mask & SP11_CAL_ALL_MASK;
		guard.cal_nominal = nominal;
	}
	spin_unlock_irqrestore(&guard.lock, flags);

	return accepted;
}
EXPORT_SYMBOL_GPL(sp11_guard_calibrated);

void sp11_guard_sp_enabled(bool enabled)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	guard.sp_enabled = enabled;
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_sp_enabled);

void sp11_guard_diagnostics_degraded(bool degraded)
{
	unsigned long flags;
	bool was_allowed;

	spin_lock_irqsave(&guard.lock, flags);
	was_allowed = !sp11_guard_reason();
	guard.diagnostics_degraded = degraded;
	sp11_guard_mute_if_refused(was_allowed);
	spin_unlock_irqrestore(&guard.lock, flags);
}
EXPORT_SYMBOL_GPL(sp11_guard_diagnostics_degraded);

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

static bool sp11_guard_degraded_fault(void)
{
	return guard.diagnostics_degraded &&
	       (!guard.loaded || !guard.graph_up || !guard.sp_enabled ||
		!guard.streams[SP11_STREAM_VI] || !sp11_guard_cal_matches_vi() ||
		(guard.cps_required && !guard.streams[SP11_STREAM_CPS]) ||
		((guard.cal_mask & SP11_CAL_ALL_MASK) != SP11_CAL_ALL_MASK) ||
		guard.cal_nominal);
}

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
	seq_printf(s, "sp_enabled: %d\ndiagnostics_degraded: %d\nmode: %s\ncaps_ready: %d\n",
		   guard.sp_enabled, guard.diagnostics_degraded,
		   guard.mode == SP11_MODE_OPEN_LOOP ? "open-loop" :
		   guard.mode == SP11_MODE_CLOSED_LOOP ? "closed-loop" : "refused",
		   guard.caps_ready);
	seq_printf(s, "degraded_fault: %d\n", sp11_guard_degraded_fault());
	seq_printf(s, "cal_mask: 0x%x\ncal_nominal: %d\n", guard.cal_mask, guard.cal_nominal);
	seq_printf(s, "cal_source: %s\n", guard.cal_mask ? "measured-or-file" : "none");
	seq_printf(s, "cal_graph_matches_vi: %d\nvi_ambiguous: %d\n",
		   sp11_guard_cal_matches_vi(), guard.vi_ambiguous);
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
