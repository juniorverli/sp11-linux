// SPDX-License-Identifier: GPL-2.0
/*
 * Closed-loop speaker protection support for AudioReach.
 *
 * Three pieces of data come from files that are generated locally from the
 * Windows calibration data and never ship in the tree:
 *
 *  qcom/<driver>/<card>-sp11cl.bin   control links, calibration records and the
 *                                     nominal speaker resistance (one per model)
 *  qcom/<driver>/<card>-sp11cal.bin  measured or nominal R0/T0 of this unit
 *
 * Without the first file there is no closed loop. Without the second file the
 * speaker protection runs uncalibrated, and the guard keeps the speakers
 * muted (include/sound/sp11-guard.h).
 */

#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/slab.h>
#include <sound/sp11-guard.h>
#include "q6apm.h"
#include "audioreach.h"

#define SP11CL_MAGIC		"SP11CL01"
#define SP11CAL_MAGIC		"SP11CA01"
#define SP11CL_FLAG_CPS		BIT(0)
#define SP11CAL_FLAG_NOMINAL	BIT(0)

/*
 * Relative window around the nominal resistance that a calibration must meet.
 * It is narrow on purpose: a 3 dB error of the voltage sense scaling changes the
 * measured resistance by a factor of 1.41 or 0.71, so a wrong sense gain makes the
 * calibration fail and the speakers stay muted.
 */
#define SP11_R0_MIN_PERCENT	80
#define SP11_R0_MAX_PERCENT	125
#define SP11_T0_MIN_Q6		(-10 * 64)
#define SP11_T0_MAX_Q6		(60 * 64)
#define SP11_CAL_CH		2

struct sp11cl_hdr {
	char magic[8];
	__le32 n_links;
	__le32 n_cal;
	__le32 flags;
	__le32 nominal_re_q24[SP11_CAL_CH];
	__le32 payload_len;
	__le32 payload_crc;
} __packed;

/* The pair is in the order of the vendor graph, peer 2 is the speaker protection module */
struct sp11cl_link {
	__le32 peer1_inst;
	__le32 peer1_port;
	__le32 peer2_inst;
	__le32 peer2_port;
	__le32 intent;
} __packed;

struct sp11cl_cal_rec {
	__le32 instance;
	__le32 param;
	__le32 size;
	u8 data[];
} __packed;

struct sp11cal_ch {
	__le32 r0_q24;
	__le16 t0_q6;
	__le16 reserved;
} __packed;

struct sp11cal_hdr {
	char magic[8];
	__le32 flags;
	__le32 n_ch;
	struct sp11cal_ch ch[SP11_CAL_CH];
	__le32 crc;
} __packed;

struct sp11cl {
	struct kref ref;
	struct device *dev;
	char cal_name[96];
	u32 n_links;
	u32 n_cal;
	u32 flags;
	u32 nominal_re_q24[SP11_CAL_CH];
	u32 payload_len;
	const struct sp11cl_link *links;
	const u8 *cal;
	u8 *blob;
};

/*
 * The loaded data is immutable and reference counted. sp11cl_lock only
 * protects the pointer, so a caller takes a reference and then reads the
 * calibration file and talks to the DSP without holding the lock.
 */
static struct sp11cl *sp11cl;
static DEFINE_MUTEX(sp11cl_lock);
/* Serialize VI record and R0/T0 sends across graph preparation attempts. */
static DEFINE_MUTEX(sp11_vi_cal_lock);

static void sp11cl_free(struct kref *ref)
{
	struct sp11cl *cl = container_of(ref, struct sp11cl, ref);

	kfree(cl->blob);
	kfree(cl);
}

static struct sp11cl *sp11cl_get(void)
{
	struct sp11cl *cl;

	mutex_lock(&sp11cl_lock);
	cl = sp11cl;
	if (cl)
		kref_get(&cl->ref);
	mutex_unlock(&sp11cl_lock);

	return cl;
}

static void sp11cl_put(struct sp11cl *cl)
{
	kref_put(&cl->ref, sp11cl_free);
}

static void sp11cl_release(void *data)
{
	struct sp11cl *cl = data;
	bool current_cl;

	mutex_lock(&sp11cl_lock);
	current_cl = sp11cl == cl;
	if (current_cl)
		sp11cl = NULL;
	mutex_unlock(&sp11cl_lock);
	if (current_cl) {
		sp11_guard_loaded(false);
		sp11_guard_graph(false, false);
	}
	sp11cl_put(cl);
}

static bool sp11cl_validate(struct device *dev, const u8 *data, size_t len,
			    struct sp11cl *cl)
{
	const struct sp11cl_hdr *hdr = (const void *)data;
	const u8 *payload;
	size_t links_len;
	u32 off, i;

	if (len < sizeof(*hdr) || memcmp(hdr->magic, SP11CL_MAGIC, sizeof(hdr->magic)))
		return false;

	cl->payload_len = le32_to_cpu(hdr->payload_len);
	if (cl->payload_len != len - sizeof(*hdr))
		return false;

	payload = data + sizeof(*hdr);
	if (crc32_le(~0, payload, cl->payload_len) != le32_to_cpu(hdr->payload_crc)) {
		dev_err(dev, "sp11cl: checksum mismatch\n");
		return false;
	}

	cl->n_links = le32_to_cpu(hdr->n_links);
	cl->n_cal = le32_to_cpu(hdr->n_cal);
	cl->flags = le32_to_cpu(hdr->flags);
	if (cl->flags & ~SP11CL_FLAG_CPS) {
		dev_err(dev, "sp11cl: unknown flags\n");
		return false;
	}
	for (i = 0; i < SP11_CAL_CH; i++)
		cl->nominal_re_q24[i] = le32_to_cpu(hdr->nominal_re_q24[i]);

	if (check_mul_overflow(cl->n_links, (u32)sizeof(struct sp11cl_link), &links_len) ||
	    links_len > cl->payload_len)
		return false;

	/* Walk the calibration records once so later reads need no bounds checks */
	off = links_len;
	for (i = 0; i < cl->n_cal; i++) {
		const struct sp11cl_cal_rec *rec;
		u32 size;

		if (cl->payload_len - off < sizeof(*rec))
			return false;
		rec = (const void *)(payload + off);
		size = le32_to_cpu(rec->size);
		if (size > SZ_4K || cl->payload_len - off - sizeof(*rec) < ALIGN(size, 4))
			return false;
		off += sizeof(*rec) + ALIGN(size, 4);
	}

	return off == cl->payload_len;
}

int audioreach_sp11_load(struct device *dev, const char *driver_name, const char *card_name)
{
	const struct firmware *fw;
	struct sp11cl *cl;
	char name[128];
	int ret;

	snprintf(name, sizeof(name), "qcom/%s/%s-sp11cl.bin", driver_name, card_name);
	ret = firmware_request_nowarn(&fw, name, dev);
	if (ret)
		return 0;	/* no closed loop for this model */

	cl = kzalloc_obj(*cl);
	if (!cl) {
		release_firmware(fw);
		return -ENOMEM;
	}

	kref_init(&cl->ref);
	cl->dev = dev;
	cl->blob = kmemdup(fw->data, fw->size, GFP_KERNEL);
	if (!cl->blob) {
		ret = -ENOMEM;
		goto out;
	}

	if (!sp11cl_validate(dev, cl->blob, fw->size, cl)) {
		dev_err(dev, "%s is not a valid closed-loop file\n", name);
		ret = -EINVAL;
		goto out;
	}

	cl->links = (const void *)(cl->blob + sizeof(struct sp11cl_hdr));
	cl->cal = cl->blob + sizeof(struct sp11cl_hdr) + cl->n_links * sizeof(*cl->links);
	snprintf(cl->cal_name, sizeof(cl->cal_name), "qcom/%s/%s-sp11cal.bin",
		 driver_name, card_name);

	mutex_lock(&sp11cl_lock);
	sp11cl = cl;
	mutex_unlock(&sp11cl_lock);

	/* On failure the action runs at once and drops the reference */
	ret = devm_add_action_or_reset(dev, sp11cl_release, cl);
	if (ret)
		goto out_nofree;

	sp11_guard_loaded(true);
	release_firmware(fw);
	dev_info(dev, "closed-loop speaker protection data loaded: %u links, %u records\n",
		 cl->n_links, cl->n_cal);

	return 0;
out:
	kfree(cl->blob);
	kfree(cl);
out_nofree:
	release_firmware(fw);

	return ret;
}

static bool sp11_graph_has_instance(const struct audioreach_graph_info *info, u32 iid)
{
	struct audioreach_container *container;
	struct audioreach_sub_graph *sg;
	struct audioreach_module *module;

	list_for_each_entry(sg, &info->sg_list, node)
		list_for_each_entry(container, &sg->container_list, node)
			list_for_each_entry(module, &container->modules_list, node)
				if (module->instance_id == iid)
					return true;

	return false;
}

/* A link is declared once, by the graph that holds its peer 2 module */
unsigned int audioreach_sp11_fill_links(const struct audioreach_graph_info *info,
					struct apm_control_link_obj *obj)
{
	struct sp11cl *cl = sp11cl_get();
	unsigned int i, n = 0;

	if (!cl)
		return 0;

	for (i = 0; i < cl->n_links; i++) {
		const struct sp11cl_link *l = &cl->links[i];

		if (!sp11_graph_has_instance(info, le32_to_cpu(l->peer2_inst)))
			continue;

		if (obj) {
			obj[n].peer1_mod_inst_id = le32_to_cpu(l->peer1_inst);
			obj[n].peer1_mod_port_id = le32_to_cpu(l->peer1_port);
			obj[n].peer2_mod_inst_id = le32_to_cpu(l->peer2_inst);
			obj[n].peer2_mod_port_id = le32_to_cpu(l->peer2_port);
			obj[n].num_props = 1;
			obj[n].intent_data.prop_id = APM_MODULE_PROP_ID_CTRL_LINK_INTENT_LIST;
			obj[n].intent_data.prop_size = 2 * sizeof(u32);
			obj[n].num_intents = 1;
			obj[n].intent_id = le32_to_cpu(l->intent);
		}
		n++;
	}
	sp11cl_put(cl);

	return n;
}

unsigned int audioreach_sp11_count_links(const struct audioreach_graph_info *info)
{
	return audioreach_sp11_fill_links(info, NULL);
}

static int sp11_send_param(struct q6apm_graph *graph, u32 iid, u32 param_id,
			   const void *data, u32 size)
{
	struct apm_module_param_data *param_data;
	int payload_size = APM_MODULE_PARAM_DATA_SIZE + ALIGN(size, 8);
	int ret;
	void *p;

	struct gpr_pkt *pkt __free(kfree) =
		audioreach_alloc_apm_cmd_pkt(payload_size, APM_CMD_SET_CFG, 0);
	if (IS_ERR(pkt))
		return PTR_ERR(pkt);

	p = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;
	param_data = p;
	param_data->module_instance_id = iid;
	param_data->error_code = 0;
	param_data->param_id = param_id;
	param_data->param_size = size;
	memcpy(p + APM_MODULE_PARAM_DATA_SIZE, data, size);

	ret = q6apm_send_cmd_sync(graph->apm, pkt, 0);
	dev_info(graph->dev, "sp11: graph %u SET_CFG iid=0x%x param=0x%08x size=%u rc=%d\n",
		 graph->id, iid, param_id, size, ret);
	return ret;
}

/* Apply the per-unit R0/T0 to the VI module, returns 0 and leaves the guard closed if absent */
static int sp11_apply_r0t0(struct q6apm_graph *graph, const struct audioreach_module *module,
			   const struct sp11cl *cl)
{
	const struct firmware *fw;
	const struct sp11cal_hdr *hdr;
	struct {
		__le32 num_ch;
		struct sp11cal_ch ch[SP11_CAL_CH];
	} __packed cfg = { };
	bool nominal;
	u32 crc, i;
	u64 generation;
	int ret;

	generation = sp11_guard_calibration_begin(graph->ar_graph);
	ret = firmware_request_nowarn(&fw, cl->cal_name, graph->dev);
	if (ret) {
		dev_warn(graph->dev, "no speaker calibration (%s), speakers stay muted\n",
			 cl->cal_name);
		sp11_guard_calibration_reset();
		return 0;
	}

	hdr = (const void *)fw->data;
	if (fw->size != sizeof(*hdr) || memcmp(hdr->magic, SP11CAL_MAGIC, sizeof(hdr->magic)) ||
	    le32_to_cpu(hdr->n_ch) != SP11_CAL_CH) {
		dev_err(graph->dev, "%s is not a valid calibration file\n", cl->cal_name);
		ret = -EINVAL;
		goto out;
	}

	crc = crc32_le(~0, fw->data, offsetof(struct sp11cal_hdr, crc));
	if (crc != le32_to_cpu(hdr->crc)) {
		dev_err(graph->dev, "%s: checksum mismatch\n", cl->cal_name);
		ret = -EINVAL;
		goto out;
	}

	if (le32_to_cpu(hdr->flags) & ~SP11CAL_FLAG_NOMINAL) {
		dev_err(graph->dev, "%s: unknown calibration flags\n", cl->cal_name);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < SP11_CAL_CH; i++) {
		u64 r0 = le32_to_cpu(hdr->ch[i].r0_q24);
		s16 t0 = (s16)le16_to_cpu(hdr->ch[i].t0_q6);
		u64 nom = cl->nominal_re_q24[i];

		if (hdr->ch[i].reserved) {
			dev_err(graph->dev, "speaker %u calibration reserved field is nonzero\n",
				i);
			ret = -EINVAL;
			goto out;
		}
		if (!nom || r0 * 100 < nom * SP11_R0_MIN_PERCENT ||
		    r0 * 100 > nom * SP11_R0_MAX_PERCENT ||
		    t0 < SP11_T0_MIN_Q6 || t0 > SP11_T0_MAX_Q6) {
			dev_err(graph->dev, "speaker %u calibration out of range, speakers stay muted\n",
				i);
			ret = -ERANGE;
			goto out;
		}
		cfg.ch[i] = hdr->ch[i];
		cfg.ch[i].reserved = 0;
	}
	cfg.num_ch = cpu_to_le32(SP11_CAL_CH);

	ret = sp11_send_param(graph, module->instance_id, PARAM_ID_SP_TH_VI_R0T0_CFG,
			      &cfg, sizeof(cfg));
	if (ret) {
		dev_err(graph->dev, "sending R0/T0 failed: %d\n", ret);
		goto out;
	}

	nominal = le32_to_cpu(hdr->flags) & SP11CAL_FLAG_NOMINAL;
	if (!sp11_guard_calibrated(generation, SP11_CAL_ALL_MASK, nominal)) {
		dev_warn(graph->dev, "speaker calibration was invalidated during setup\n");
		ret = -ECANCELED;
		goto out;
	}
	dev_info(graph->dev, "speaker calibration applied (%s)\n",
		 nominal ? "nominal" : "non-nominal");
out:
	if (ret)
		sp11_guard_calibration_reset();
	release_firmware(fw);

	return ret;
}

/* Send the calibration records of one module, then the unit calibration for the VI module */
int audioreach_sp11_graph_params(struct q6apm_graph *graph,
				 const struct audioreach_module *module)
{
	bool vi = module->module_id == MODULE_ID_SPEAKER_PROTECTION_VI;
	struct sp11cl *cl;
	u32 off = 0, i;
	int ret = 0;

	if (vi) {
		mutex_lock(&sp11_vi_cal_lock);
		/* Records can fail before sp11_apply_r0t0() is reached. */
		sp11_guard_calibration_reset();
	}
	cl = sp11cl_get();
	if (!cl)
		goto out;

	for (i = 0; i < cl->n_cal; i++) {
		const struct sp11cl_cal_rec *rec = (const void *)(cl->cal + off);
		u32 size = le32_to_cpu(rec->size);

		off += sizeof(*rec) + ALIGN(size, 4);
		if (le32_to_cpu(rec->instance) != module->instance_id)
			continue;

		ret = sp11_send_param(graph, module->instance_id, le32_to_cpu(rec->param),
				      rec->data, size);
		if (ret) {
			dev_err(graph->dev, "calibration 0x%08x for 0x%x failed: %d\n",
				le32_to_cpu(rec->param), module->instance_id, ret);
			break;
		}
	}

	if (!ret && vi)
		ret = sp11_apply_r0t0(graph, module, cl);
	if (ret && vi)
		sp11_guard_calibration_reset();
	sp11cl_put(cl);
out:
	if (vi)
		mutex_unlock(&sp11_vi_cal_lock);

	return ret;
}

/*
 * This firmware expects the 16-byte module event form the Windows GSL client
 * sends: qcadcm8380.sys registers a custom event with payload size 0x10
 * (instance, event id, register flag, configuration size). The kernel's
 * apm_module_register_events adds error_code and reserved for 24 bytes and the
 * DSP answered that packet with status 3. Send the short form first and keep
 * the long one as a fallback so no other target regresses.
 */
struct sp11_module_event_short {
	__le32 module_instance_id;
	__le32 event_id;
	__le32 is_register;
	__le32 event_config_payload_size;
} __packed;

static int sp11_send_module_event(struct q6apm_graph *graph,
				  const struct audioreach_module *module,
				  u32 event_id, bool short_form)
{
	void *p;

	if (short_form) {
		struct sp11_module_event_short *event;
		struct gpr_pkt *pkt __free(kfree) =
			audioreach_alloc_cmd_pkt(sizeof(*event), APM_CMD_REGISTER_MODULE_EVENTS, 0,
						 graph->port->id, module->instance_id);

		if (IS_ERR(pkt))
			return PTR_ERR(pkt);
		p = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;
		event = p;
		event->module_instance_id = module->instance_id;
		event->event_id = event_id;
		event->is_register = 1;
		event->event_config_payload_size = 0;
		return audioreach_graph_send_cmd_sync(graph, pkt, 0);
	} else {
		struct apm_module_register_events *event;
		struct gpr_pkt *pkt __free(kfree) =
			audioreach_alloc_cmd_pkt(sizeof(*event), APM_CMD_REGISTER_MODULE_EVENTS, 0,
						 graph->port->id, module->instance_id);

		if (IS_ERR(pkt))
			return PTR_ERR(pkt);
		p = (void *)pkt + GPR_HDR_SIZE + APM_CMD_HDR_SIZE;
		event = p;
		event->module_instance_id = module->instance_id;
		event->event_id = event_id;
		event->is_register = 1;
		event->event_config_payload_size = 0;
		return audioreach_graph_send_cmd_sync(graph, pkt, 0);
	}
}

static int sp11_register_one_event(struct q6apm_graph *graph,
				   const struct audioreach_module *module,
				   u32 event_id)
{
	int ret;

	ret = sp11_send_module_event(graph, module, event_id, true);
	if (!ret)
		return 0;
	dev_info(graph->dev,
		 "event 0x%08x registration: short form failed (%d), retrying the long form\n",
		 event_id, ret);

	return sp11_send_module_event(graph, module, event_id, false);
}

int audioreach_sp11_register_events(struct q6apm_graph *graph,
				    const struct audioreach_module *module)
{
	int ret;

	ret = sp11_register_one_event(graph, module, EVENT_ID_SP_DIAGNOSTICS);
	if (ret)
		return ret;
	ret = sp11_register_one_event(graph, module, EVENT_ID_VI_CALIBRATION);
	if (ret)
		return ret;
	/*
	 * The DSP reports the measured R0 through these two events; the layouts
	 * are the ones the Windows ACDB driver parses (REA 2026-11-05).
	 */
	return sp11_register_one_event(graph, module, EVENT_ID_VI_PER_SPKR_CALIBRATION);
}

#define SP11_CALIB_SUCCESS 4

/*
 * Aggregate VI calibration: [num_ch][state][R0_ch0]...[R0_chN], state 4 =
 * success, 5 = failure (Windows ParseAndSendSpeakerProtectionCalibPayload).
 */
static void sp11_vi_calibration_event(struct device *dev, struct q6apm_graph *graph,
				      const void *payload, u32 size)
{
	const __le32 *w = payload;
	unsigned int i, n, mask = 0;
	u32 state;

	if (size < 8)
		return;
	n = le32_to_cpu(w[0]);
	state = le32_to_cpu(w[1]);
	dev_info(dev, "VI calibration: count %u state %u\n", n, state);
	if (n < 1 || n > 2 || state != SP11_CALIB_SUCCESS || size < (2 + n) * 4)
		return;
	for (i = 0; i < n; i++) {
		u32 r0 = le32_to_cpu(w[2 + i]);

		if (!r0)
			continue;
		dev_info(dev, "VI calibration ch %u R0 0x%08x\n", i, r0);
		mask |= BIT(i);
	}
	sp11_guard_vi_calibration(graph, mask, mask != 0);
}

/*
 * Per-speaker calibration: [num_ch][state_0][R0_0][state_1][R0_1]...
 * (Windows ParseAndSendSpeakerProtectionPerSpkrCalibPayload).
 */
static void sp11_vi_per_spkr_calibration_event(struct device *dev, struct q6apm_graph *graph,
					       const void *payload, u32 size)
{
	const __le32 *w = payload;
	unsigned int i, n, mask = 0;

	if (size < 4)
		return;
	n = le32_to_cpu(w[0]);
	if (n < 1 || n > 2 || size < (1 + 2 * n) * 4)
		return;
	for (i = 0; i < n; i++) {
		u32 state = le32_to_cpu(w[1 + 2 * i]);
		u32 r0 = le32_to_cpu(w[2 + 2 * i]);

		dev_info(dev, "VI calibration per speaker ch %u state %u R0 0x%08x\n",
			 i, state, r0);
		if (state == SP11_CALIB_SUCCESS && r0)
			mask |= BIT(i);
	}
	sp11_guard_vi_calibration(graph, mask, mask != 0);
}

/*
 * The layout of the diagnostics payload is not known (Unknown in the dossier).
 * Fail safe: any diagnostics event latches a fault, and the first words are
 * logged so the first device test shows what the module reports.
 */
void audioreach_sp11_event(struct device *dev, struct q6apm_graph *graph, u32 event_id,
			   const void *payload, u32 size)
{
	const __le32 *w = payload;
	u32 first = size >= sizeof(*w) ? le32_to_cpu(w[0]) : 0;
	u32 second = size >= 2 * sizeof(*w) ? le32_to_cpu(w[1]) : 0;

	if (event_id == EVENT_ID_VI_CALIBRATION) {
		sp11_vi_calibration_event(dev, graph, payload, size);
		return;
	}
	if (event_id == EVENT_ID_VI_PER_SPKR_CALIBRATION) {
		sp11_vi_per_spkr_calibration_event(dev, graph, payload, size);
		return;
	}
	if (event_id != EVENT_ID_SP_DIAGNOSTICS)
		return;

	dev_err(dev, "speaker protection diagnostics event: size %u, words 0x%08x 0x%08x\n",
		size, first, second);
	sp11_guard_fault("spv5", first);
}

/* Track the closed-loop graph state for the guard when a graph starts or stops */
void audioreach_sp11_graph_state(struct q6apm_graph *graph, bool started)
{
	struct sp11cl *cl = sp11cl_get();
	bool cps;

	if (!cl)
		return;
	cps = cl->flags & SP11CL_FLAG_CPS;
	sp11cl_put(cl);

	if (q6apm_find_module_by_mid(graph, MODULE_ID_SPEAKER_PROTECTION))
		sp11_guard_graph(started, cps);
	if (!started && q6apm_find_module_by_mid(graph, MODULE_ID_SPEAKER_PROTECTION)) {
		sp11_guard_sp_enabled(false);
		sp11_guard_diagnostics_degraded(false);
	}
	if (q6apm_find_module_by_mid(graph, MODULE_ID_SPEAKER_PROTECTION_VI))
		sp11_guard_stream(SP11_STREAM_VI, started, graph->ar_graph);
	if (q6apm_find_module_by_mid(graph, MODULE_ID_CPS_DATA_ROUTER))
		sp11_guard_stream(SP11_STREAM_CPS, started, graph->ar_graph);
}
