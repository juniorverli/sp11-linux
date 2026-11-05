/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Speaker protection guard for the Surface Pro 11 (microsoft,denali).
 *
 * One kernel-side object decides whether the WSA884x speakers may play.
 * The state is written only by in-kernel producers (the AudioReach core, the
 * WSA884x codec) and read by the machine driver and the amplifier driver.
 * There is no sysfs, ioctl or mixer path that sets it, and a fault latches
 * until reboot.
 *
 * There are two decisions. A stream may be opened and prepared while the
 * amplifier stays muted: that needs the closed-loop data, no fault and no
 * kill switch (sp11_guard_check()). The amplifier may be unmuted only
 * when the graph and the V/I sense run, the calibration is applied and no
 * fault is latched (sp11_guard_unmute_check()). The amplifier driver asks on
 * every unmute path, so trigger, pause release and resume cannot bypass it.
 */
#ifndef __SOUND_SP11_GUARD_H
#define __SOUND_SP11_GUARD_H

#include <linux/errno.h>
#include <linux/types.h>

struct device;

enum sp11_guard_stream {
	SP11_STREAM_VI,
	SP11_STREAM_CPS,
	SP11_STREAM_MAX,
};

/* Speaker channels that need a calibration (left and right) */
#define SP11_CAL_ALL_MASK	0x3

/*
 * IS_ENABLED(), not IS_REACHABLE(): every user of these hooks selects or
 * depends on the guard so that it is never a module for a built-in caller.
 * A broken combination fails to link instead of silently building no-ops.
 */
#if IS_ENABLED(CONFIG_SND_SOC_SP11_GUARD)
/* Producers */
int sp11_guard_select_mode(bool marker_valid, bool has_sp);
void sp11_guard_caps_ready(bool ready);
void sp11_guard_loaded(bool loaded);
void sp11_guard_graph(bool up, bool cps_required);
void sp11_guard_stream(enum sp11_guard_stream stream, bool start);
void sp11_guard_calibrated(unsigned int ch_mask, bool nominal);
void sp11_guard_sp_enabled(bool enabled);
void sp11_guard_diagnostics_degraded(bool degraded);
void sp11_guard_fault(const char *src, u32 code);
void sp11_guard_set_mute(void (*mute)(void *data), void *data);
void sp11_guard_clear_mute(void *data);

/* Consumers */
int sp11_guard_check(struct device *dev);
int sp11_guard_unmute_check(struct device *dev);
bool sp11_guard_faulted(void);
bool sp11_guard_nospk(void);
#else
static inline int sp11_guard_select_mode(bool marker_valid, bool has_sp) { return -EOPNOTSUPP; }
static inline void sp11_guard_caps_ready(bool ready) { }
static inline void sp11_guard_loaded(bool loaded) { }
static inline void sp11_guard_graph(bool up, bool cps_required) { }
static inline void sp11_guard_stream(enum sp11_guard_stream stream, bool start) { }
static inline void sp11_guard_calibrated(unsigned int ch_mask, bool nominal) { }
static inline void sp11_guard_sp_enabled(bool enabled) { }
static inline void sp11_guard_diagnostics_degraded(bool degraded) { }
static inline void sp11_guard_fault(const char *src, u32 code) { }
static inline void sp11_guard_set_mute(void (*mute)(void *data), void *data) { }
static inline void sp11_guard_clear_mute(void *data) { }
static inline bool sp11_guard_faulted(void) { return true; }
static inline bool sp11_guard_nospk(void) { return true; }
/* Without the guard the speakers are never allowed */
static inline int sp11_guard_check(struct device *dev) { return -EOPNOTSUPP; }
static inline int sp11_guard_unmute_check(struct device *dev) { return -EOPNOTSUPP; }
#endif

#endif /* __SOUND_SP11_GUARD_H */
