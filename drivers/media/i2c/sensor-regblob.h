/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sensor register blob (SRB), format versions 1 and 2.
 *
 * Some camera sensors have no public register documentation. Their register
 * sequences exist only as vendor data on the device itself, so they cannot be
 * part of the kernel source. A driver that includes this header loads those
 * sequences at probe time from a firmware file instead. The file is generated
 * locally from data the owner of the device already has; nothing in this
 * header or in the drivers contains a sensor register value.
 *
 * All integers are little endian. All offsets are counted from the start of
 * the file and must point inside the first "total_size" bytes.
 *
 * Header, 64 bytes:
 *
 *	 0  u8[8]  magic        "SPSRBLOB"
 *	 8  le16   version      1 or 2
 *	10  le16   header_size  64
 *	12  le32   total_size   size of the whole blob, header included
 *	16  le32   crc32        zlib CRC-32 of bytes [header_size, total_size)
 *	20  le16   n_modes      number of mode records
 *	22  le16   n_ctrls      number of control records
 *	24  le32   init_off     offset of the init operation list
 *	28  le32   init_count   number of operations in it
 *	32  le32   on_off       stream-on operation list
 *	36  le32   on_count
 *	40  le32   off_off      stream-off operation list
 *	44  le32   off_count
 *	48  le32   ctrl_off     offset of n_ctrls control records
 *	52  le32   mode_off     offset of n_modes mode records
 *	56  le16   id_reg       register that holds the chip identifier
 *	58  u8     id_width     identifier width in bytes, 0 = do not check
 *	59  u8     id_flags     bit 0: register is little endian
 *	60  le32   id_value     expected identifier
 *
 * Operation, 4 bytes: le16 reg_or_ms, u8 value, u8 op.
 *	op 0: write the 8 bit value to the 16 bit register address
 *	op 1: sleep reg_or_ms milliseconds (at most SRB_MAX_DELAY_MS)
 *	op 2: poll (version 2 only), see below
 *
 * Version 2 adds the poll operation. It takes two consecutive 4 byte slots,
 * both counted in the operation count of the list:
 *
 *	slot 0: le16 reg, u8 expected, u8 op = 2
 *	slot 1: u8 mask, le24 timeout_us
 *
 * The register is read repeatedly until (value & mask) == (expected & mask).
 * The mask must not be zero and the timeout is 1 to SRB_MAX_POLL_US
 * microseconds. When the timeout expires the operation list fails with
 * -ETIMEDOUT and an error message; the rest of the list is not run. Slot 1
 * is not an operation of its own. A version 1 file with op 2 is invalid; a
 * version 2 file has all the other records exactly as version 1.
 *
 * Control record, 20 bytes:
 *	 0  u8    kind   1 exposure (lines), 2 analogue gain, 3 frame length
 *	 1  u8    width  register width in bytes, 1 to 3
 *	 2  u8    flags  bit 0: register is little endian
 *	 3  u8    reserved, zero
 *	 4  le16  reg
 *	 6  le16  step
 *	 8  le32  min
 *	12  le32  max
 *	16  le32  default
 *
 * Mode record, 40 bytes:
 *	 0  le16  width            active width in pixels
 *	 2  le16  height           active height in lines
 *	 4  le32  mbus_code        MEDIA_BUS_FMT_* code, 0 = the driver default
 *	 8  le32  link_freq        V4L2 link frequency in Hz
 *	12  le32  pixel_rate       pixel rate in Hz
 *	16  le16  hts              line length in pixel clocks
 *	18  le16  vts              default frame length in lines
 *	20  le16  vts_min          minimum frame length in lines
 *	22  le16  reserved, zero
 *	24  le32  ops_off          operation list of this mode
 *	28  le32  ops_count
 *	32  u8    lanes            data lanes (D-PHY) or trios (C-PHY)
 *	33  u8    flags            reserved, zero
 *	34  le16  reserved, zero
 *	36  le32  reserved, zero
 */

#ifndef _SENSOR_REGBLOB_H
#define _SENSOR_REGBLOB_H

#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/ktime.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define SRB_MAGIC		"SPSRBLOB"
#define SRB_VERSION_1		1
#define SRB_VERSION_2		2
#define SRB_HDR_SIZE		64
#define SRB_OP_SIZE		4
#define SRB_CTRL_SIZE		20
#define SRB_MODE_SIZE		40
#define SRB_MAX_DELAY_MS	1000
#define SRB_MAX_POLL_US		10000000

#define SRB_OP_WRITE8		0
#define SRB_OP_DELAY_MS		1
#define SRB_OP_POLL		2

#define SRB_CTRL_EXPOSURE	1
#define SRB_CTRL_ANALOG_GAIN	2
#define SRB_CTRL_FRAME_LENGTH	3

struct srb_ops {
	const u8 *p;
	u32 count;
};

struct srb_ctrl {
	u8 kind;
	u8 width;
	bool little_endian;
	u16 reg;
	u16 step;
	u32 min;
	u32 max;
	u32 def;
};

struct srb_mode {
	u16 width;
	u16 height;
	u32 mbus_code;
	u32 link_freq;
	u32 pixel_rate;
	u16 hts;
	u16 vts;
	u16 vts_min;
	u8 lanes;
	struct srb_ops ops;
};

struct srb {
	const u8 *data;
	u16 version;
	u32 total;
	u16 n_modes;
	u16 n_ctrls;
	const u8 *modes;
	const u8 *ctrls;
	struct srb_ops init;
	struct srb_ops on;
	struct srb_ops off;
	u16 id_reg;
	u8 id_width;
	bool id_little_endian;
	u32 id_value;
};

/* Check one operation list and return a pointer to it. */
static inline int srb_get_ops(const u8 *data, u32 total, u16 version, u32 off,
			      u32 count, struct srb_ops *ops)
{
	size_t bytes;
	u32 i;

	ops->p = NULL;
	ops->count = 0;
	if (!count)
		return 0;

	if (check_mul_overflow((size_t)count, (size_t)SRB_OP_SIZE, &bytes))
		return -EINVAL;
	if (off < SRB_HDR_SIZE || off > total || bytes > total - off)
		return -EINVAL;

	for (i = 0; i < count; i++) {
		const u8 *op = data + off + i * SRB_OP_SIZE;
		u32 timeout;

		switch (op[3]) {
		case SRB_OP_WRITE8:
			break;
		case SRB_OP_DELAY_MS:
			if (get_unaligned_le16(op) > SRB_MAX_DELAY_MS)
				return -EINVAL;
			break;
		case SRB_OP_POLL:
			/* Two slots: the second one holds mask and timeout */
			if (version < SRB_VERSION_2 || i + 1 >= count)
				return -EINVAL;
			timeout = get_unaligned_le24(op + SRB_OP_SIZE + 1);
			if (!op[SRB_OP_SIZE] || !timeout ||
			    timeout > SRB_MAX_POLL_US)
				return -EINVAL;
			i++;
			break;
		default:
			return -EINVAL;
		}
	}

	ops->p = data + off;
	ops->count = count;

	return 0;
}

/**
 * srb_parse - validate a blob and index its sections
 * @dev: device used for messages
 * @fw: the firmware file; it must stay loaded while @b is used
 * @b: filled in on success
 *
 * Every offset and count is checked against the file size, the CRC is
 * verified and each mode and control record is range checked, so the
 * accessors below cannot read outside the file.
 */
static inline int srb_parse(struct device *dev, const struct firmware *fw,
			    struct srb *b)
{
	const u8 *d = fw->data;
	u32 total, off, count, i;
	u16 version;
	size_t bytes;
	int ret;

	memset(b, 0, sizeof(*b));

	if (fw->size < SRB_HDR_SIZE || memcmp(d, SRB_MAGIC, 8)) {
		dev_err(dev, "register blob: bad magic\n");
		return -EINVAL;
	}
	version = get_unaligned_le16(d + 8);
	if ((version != SRB_VERSION_1 && version != SRB_VERSION_2) ||
	    get_unaligned_le16(d + 10) != SRB_HDR_SIZE) {
		dev_err(dev, "register blob: unsupported version %u\n",
			version);
		return -EINVAL;
	}

	total = get_unaligned_le32(d + 12);
	if (total < SRB_HDR_SIZE || total > fw->size) {
		dev_err(dev, "register blob: bad size %u (file %zu)\n", total,
			fw->size);
		return -EINVAL;
	}
	if ((crc32_le(~0, d + SRB_HDR_SIZE, total - SRB_HDR_SIZE) ^ ~0) !=
	    get_unaligned_le32(d + 16)) {
		dev_err(dev, "register blob: CRC mismatch\n");
		return -EINVAL;
	}

	b->data = d;
	b->version = version;
	b->total = total;
	b->n_modes = get_unaligned_le16(d + 20);
	b->n_ctrls = get_unaligned_le16(d + 22);

	ret = srb_get_ops(d, total, version, get_unaligned_le32(d + 24),
			  get_unaligned_le32(d + 28), &b->init);
	if (!ret)
		ret = srb_get_ops(d, total, version, get_unaligned_le32(d + 32),
				  get_unaligned_le32(d + 36), &b->on);
	if (!ret)
		ret = srb_get_ops(d, total, version, get_unaligned_le32(d + 40),
				  get_unaligned_le32(d + 44), &b->off);
	if (ret)
		goto bad;

	if (b->n_ctrls) {
		off = get_unaligned_le32(d + 48);
		bytes = (size_t)b->n_ctrls * SRB_CTRL_SIZE;
		if (off < SRB_HDR_SIZE || off > total || bytes > total - off)
			goto bad_layout;
		b->ctrls = d + off;
		for (i = 0; i < b->n_ctrls; i++) {
			const u8 *c = b->ctrls + i * SRB_CTRL_SIZE;

			if (c[0] < SRB_CTRL_EXPOSURE ||
			    c[0] > SRB_CTRL_FRAME_LENGTH ||
			    c[1] < 1 || c[1] > 3 || !get_unaligned_le16(c + 6) ||
			    get_unaligned_le32(c + 8) > get_unaligned_le32(c + 12))
				goto bad_layout;
		}
	}

	if (b->n_modes) {
		off = get_unaligned_le32(d + 52);
		bytes = (size_t)b->n_modes * SRB_MODE_SIZE;
		if (off < SRB_HDR_SIZE || off > total || bytes > total - off)
			goto bad_layout;
		b->modes = d + off;
		for (i = 0; i < b->n_modes; i++) {
			const u8 *m = b->modes + i * SRB_MODE_SIZE;
			struct srb_ops ops;

			count = get_unaligned_le32(m + 28);
			if (srb_get_ops(d, total, version,
					get_unaligned_le32(m + 24), count, &ops))
				goto bad_layout;
			if (!get_unaligned_le16(m) || !get_unaligned_le16(m + 2) ||
			    !get_unaligned_le32(m + 8) ||
			    !get_unaligned_le32(m + 12) ||
			    get_unaligned_le16(m + 16) < get_unaligned_le16(m) ||
			    get_unaligned_le16(m + 18) < get_unaligned_le16(m + 2) ||
			    get_unaligned_le16(m + 20) > get_unaligned_le16(m + 18) ||
			    !m[32] || m[32] > 4)
				goto bad_layout;
		}
	}

	b->id_reg = get_unaligned_le16(d + 56);
	b->id_width = d[58];
	b->id_little_endian = d[59] & 1;
	b->id_value = get_unaligned_le32(d + 60);
	if (b->id_width > 4)
		goto bad_layout;

	return 0;

bad_layout:
	ret = -EINVAL;
bad:
	dev_err(dev, "register blob: invalid layout\n");
	return ret;
}

/* Return mode @i, which srb_parse() has already checked. */
static inline void srb_get_mode(const struct srb *b, unsigned int i,
				struct srb_mode *mode)
{
	const u8 *m = b->modes + i * SRB_MODE_SIZE;

	mode->width = get_unaligned_le16(m);
	mode->height = get_unaligned_le16(m + 2);
	mode->mbus_code = get_unaligned_le32(m + 4);
	mode->link_freq = get_unaligned_le32(m + 8);
	mode->pixel_rate = get_unaligned_le32(m + 12);
	mode->hts = get_unaligned_le16(m + 16);
	mode->vts = get_unaligned_le16(m + 18);
	mode->vts_min = get_unaligned_le16(m + 20);
	mode->lanes = m[32];
	srb_get_ops(b->data, b->total, b->version, get_unaligned_le32(m + 24),
		    get_unaligned_le32(m + 28), &mode->ops);
}

/* Return control record @i, which srb_parse() has already checked. */
static inline void srb_get_ctrl(const struct srb *b, unsigned int i,
				struct srb_ctrl *ctrl)
{
	const u8 *c = b->ctrls + i * SRB_CTRL_SIZE;

	ctrl->kind = c[0];
	ctrl->width = c[1];
	ctrl->little_endian = c[2] & 1;
	ctrl->reg = get_unaligned_le16(c + 4);
	ctrl->step = get_unaligned_le16(c + 6);
	ctrl->min = get_unaligned_le32(c + 8);
	ctrl->max = get_unaligned_le32(c + 12);
	ctrl->def = get_unaligned_le32(c + 16);
}

/* Run one poll operation, @op points at its first slot. */
static inline int srb_poll(struct device *dev, const u8 *op,
			   int (*read)(void *ctx, u16 reg, u8 *val), void *ctx)
{
	u16 reg = get_unaligned_le16(op);
	u8 expected = op[2];
	u8 mask = op[SRB_OP_SIZE];
	u32 timeout = get_unaligned_le24(op + SRB_OP_SIZE + 1);
	ktime_t end = ktime_add_us(ktime_get(), timeout);
	u8 val;
	int ret;

	for (;;) {
		ret = read(ctx, reg, &val);
		if (ret)
			return ret;
		if ((val & mask) == (expected & mask))
			return 0;
		if (ktime_after(ktime_get(), end))
			break;
		usleep_range(100, 200);
	}

	dev_err(dev, "register blob: poll of register 0x%04x timed out after %u us (read 0x%02x, wanted 0x%02x under mask 0x%02x)\n",
		reg, timeout, val, expected, mask);

	return -ETIMEDOUT;
}

/**
 * srb_apply - run an operation list
 * @dev: device used for messages
 * @ops: list returned by srb_parse() or srb_get_mode()
 * @write: callback that writes one 8 bit register
 * @read: callback that reads one 8 bit register, used by poll operations;
 *	  may be NULL for a list that has none (a poll then fails with
 *	  -EOPNOTSUPP)
 * @ctx: passed to @write and @read
 *
 * The first failing operation ends the list and its error is returned. A poll
 * that times out returns -ETIMEDOUT after an error message.
 */
static inline int srb_apply(struct device *dev, const struct srb_ops *ops,
			    int (*write)(void *ctx, u16 reg, u8 val),
			    int (*read)(void *ctx, u16 reg, u8 *val), void *ctx)
{
	u32 i;
	int ret;

	for (i = 0; i < ops->count; i++) {
		const u8 *op = ops->p + i * SRB_OP_SIZE;
		u16 reg = get_unaligned_le16(op);

		switch (op[3]) {
		case SRB_OP_DELAY_MS:
			msleep(reg);
			break;
		case SRB_OP_POLL:
			if (!read)
				return -EOPNOTSUPP;
			ret = srb_poll(dev, op, read, ctx);
			if (ret)
				return ret;
			i++;	/* the second slot belongs to this poll */
			break;
		default:
			ret = write(ctx, reg, op[2]);
			if (ret)
				return ret;
			break;
		}
	}

	return 0;
}

#endif /* _SENSOR_REGBLOB_H */
