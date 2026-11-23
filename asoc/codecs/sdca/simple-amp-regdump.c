// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022, 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#ifdef CONFIG_DEBUG_FS
#include <linux/fs.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <soc/soundwire.h>
#include "simple-amp.h"

#define SWR_SLV_MAX_REG_ADDR    0x2009
#define SWR_SLV_START_REG_ADDR  0x40
#define SWR_SLV_MAX_BUF_LEN     20
#define BYTES_PER_LINE          12
#define SWR_SLV_RD_BUF_LEN      8
#define SWR_SLV_WR_BUF_LEN      32
#define SWR_SLV_MAX_DEVICES     2


static int codec_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int get_parameters(char *buf, u32 *param1, int num_of_par)
{
	char *token;
	int base, cnt;

	token = strsep(&buf, " ");
	for (cnt = 0; cnt < num_of_par; cnt++) {
		if (token) {
			if ((token[1] == 'x') || (token[1] == 'X'))
				base = 16;
			else
				base = 10;

			if (kstrtou32(token, base, &param1[cnt]) != 0)
				return -EINVAL;

			token = strsep(&buf, " ");
		} else {
			return -EINVAL;
		}
	}
	return 0;
}

static bool is_swr_slave_reg_readable(int reg)
{
	int ret = true;

	if (((reg > 0x46) && (reg < 0x4A)) ||
	    ((reg > 0x55) && (reg < 0x60)) ||
	    ((reg > 0x60) && (reg < 0x70)) ||
	    ((reg > 0x70) && (reg < 0xC0)) ||
	    ((reg > 0xC1) && (reg < 0xC8)) ||
	    ((reg > 0xC8) && (reg < 0xD0)) ||
	    ((reg > 0xD0) && (reg < 0xE0)) ||
	    ((reg > 0xE0) && (reg < 0xF0)) ||
	    ((reg > 0xF0) && (reg < 0x100)) ||
	    ((reg > 0x105) && (reg < 0x120)) ||
	    ((reg > 0x205) && (reg < 0x220)) ||
	    ((reg > 0x305) && (reg < 0x320)) ||
	    ((reg > 0x405) && (reg < 0x420)) ||
	    ((reg > 0x505) && (reg < 0x520)) ||
	    ((reg > 0x605) && (reg < 0x620)) ||
	    ((reg > 0x127) && (reg < 0x130)) ||
	    ((reg > 0x227) && (reg < 0x230)) ||
	    ((reg > 0x327) && (reg < 0x330)) ||
	    ((reg > 0x427) && (reg < 0x430)) ||
	    ((reg > 0x527) && (reg < 0x530)) ||
	    ((reg > 0x627) && (reg < 0x630)) ||
	    ((reg > 0x137) && (reg < 0x200)) ||
	    ((reg > 0x237) && (reg < 0x300)) ||
	    ((reg > 0x337) && (reg < 0x400)) ||
	    ((reg > 0x437) && (reg < 0x500)) ||
	    ((reg > 0x537) && (reg < 0x600)) ||
	    ((reg > 0x637) && (reg < 0xF00)) ||
	    ((reg > 0xF05) && (reg < 0xF20)) ||
	    ((reg > 0xF25) && (reg < 0xF30)) ||
	    ((reg > 0xF35) && (reg < 0x2000)))
		ret = false;

	return ret;
}

static ssize_t swr_slave_reg_show(struct swr_device *pdev, char __user *ubuf,
					size_t count, loff_t *ppos)
{
	int i, reg_val, len;
	ssize_t total = 0;
	char tmp_buf[SWR_SLV_MAX_BUF_LEN];

	if (!ubuf || !ppos)
		return 0;

	for (i = (((int) *ppos/BYTES_PER_LINE) + SWR_SLV_START_REG_ADDR);
		i <= SWR_SLV_MAX_REG_ADDR; i++) {
		if (!is_swr_slave_reg_readable(i))
			continue;
		swr_read(pdev, pdev->dev_num, i, &reg_val, 1);
		len = snprintf(tmp_buf, sizeof(tmp_buf), "0x%.3x: 0x%.2x\n", i,
			       (reg_val & 0xFF));
		if (len < 0) {
			pr_err_ratelimited("%s: fail to fill the buffer\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		if ((total + len) >= count - 1)
			break;
		if (copy_to_user((ubuf + total), tmp_buf, len)) {
			pr_err_ratelimited("%s: fail to copy reg dump\n", __func__);
			total = -EFAULT;
			goto copy_err;
		}
		total += len;
		*ppos += len;
	}

copy_err:
	*ppos = SWR_SLV_MAX_REG_ADDR * BYTES_PER_LINE;
	return total;
}

static ssize_t codec_debug_dump(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct swr_device *pdev;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	return swr_slave_reg_show(pdev, ubuf, count, ppos);
}

static ssize_t codec_debug_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char lbuf[SWR_SLV_RD_BUF_LEN];
	struct swr_device *pdev = NULL;
	struct simple_amp_priv *simple_amp = NULL;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	simple_amp = swr_get_dev_data(pdev);
	if (!simple_amp)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	snprintf(lbuf, sizeof(lbuf), "0x%x\n",
			(simple_amp->read_data & 0xFF));

	return simple_read_from_buffer(ubuf, count, ppos, lbuf,
					       strnlen(lbuf, 7));
}

static ssize_t codec_debug_peek_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev = NULL;
	struct simple_amp_priv *simple_amp = NULL;

	if (!cnt || !file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	simple_amp = swr_get_dev_data(pdev);
	if (!simple_amp)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 1);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) && (rc == 0)))
		return -EINVAL;
	swr_read(pdev, pdev->dev_num, param[0], &simple_amp->read_data, 1);
	if (rc == 0)
		rc = cnt;
	else
		pr_err_ratelimited("%s: rc = %d\n", __func__, rc);

	return rc;
}

static ssize_t codec_debug_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char lbuf[SWR_SLV_WR_BUF_LEN];
	int rc = 0;
	u32 param[5];
	struct swr_device *pdev;

	if (!file || !ppos || !ubuf)
		return -EINVAL;

	pdev = file->private_data;
	if (!pdev)
		return -EINVAL;

	if (cnt > sizeof(lbuf) - 1)
		return -EINVAL;

	rc = copy_from_user(lbuf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	lbuf[cnt] = '\0';
	rc = get_parameters(lbuf, param, 2);
	if (!((param[0] <= SWR_SLV_MAX_REG_ADDR) &&
		(param[1] <= 0xFF) && (rc == 0)))
		return -EINVAL;
	swr_write(pdev, pdev->dev_num, param[0], &param[1]);
	if (rc == 0)
		rc = cnt;
	else
		pr_err_ratelimited("%s: rc = %d\n", __func__, rc);

	return rc;
}

static const struct file_operations codec_debug_write_ops = {
	.open = codec_debug_open,
	.write = codec_debug_write,
};

static const struct file_operations codec_debug_read_ops = {
	.open = codec_debug_open,
	.read = codec_debug_read,
	.write = codec_debug_peek_write,
};

static const struct file_operations codec_debug_dump_ops = {
	.open = codec_debug_open,
	.read = codec_debug_dump,
};

void simple_amp_regdump_register(struct simple_amp_priv *simple_amp,
		struct swr_device *pdev)
{
	if (!simple_amp->debugfs_dent) {
		simple_amp->debugfs_dent = debugfs_create_dir(
				dev_name(&pdev->dev), 0);
		if (!IS_ERR(simple_amp->debugfs_dent)) {
			simple_amp->debugfs_peek =
				debugfs_create_file("swrslave_peek",
						S_IFREG | 0444,
						simple_amp->debugfs_dent,
						(void *) pdev,
						&codec_debug_read_ops);

			simple_amp->debugfs_poke =
				debugfs_create_file("swrslave_poke",
						S_IFREG | 0444,
						simple_amp->debugfs_dent,
						(void *) pdev,
						&codec_debug_write_ops);

			simple_amp->debugfs_reg_dump =
				debugfs_create_file(
						"swrslave_reg_dump",
						S_IFREG | 0444,
						simple_amp->debugfs_dent,
						(void *) pdev,
						&codec_debug_dump_ops);
		}
	}
}
#endif
