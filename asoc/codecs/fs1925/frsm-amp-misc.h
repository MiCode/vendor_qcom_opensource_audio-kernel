/* SPDX-License-Identifier: GPL-2.0+ */
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2024-02-01 File created.
 */

#ifndef __FRSM_AMP_MISC_H__
#define __FRSM_AMP_MISC_H__

#include "frsm-amp-drv.h"

#define FRSM_MISC_NAME        "fs16xx"

static int frsm_misc_open(struct inode *node, struct file *fp)
{
	if (fp->private_data == NULL)
		return -EINVAL;

	return 0;
}

static int frsm_misc_release(struct inode *node, struct file *fp)
{
	return 0;
}

static ssize_t frsm_misc_read(struct file *fp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct frsm_amp_reg *amp_reg;
	struct frsm_amp *frsm_amp;
	struct frsm_argv argv;
	int size;
	int ret;

	if (fp == NULL || fp->private_data == NULL)
		return -EINVAL;

	frsm_amp = container_of((struct miscdevice *)fp->private_data,
			struct frsm_amp, misc_dev);

	size = count + sizeof(struct frsm_amp_reg);
	amp_reg = kzalloc(size, GFP_KERNEL);
	if (amp_reg == NULL)
		return -ENOMEM;

	amp_reg->addr = frsm_amp->addr;
	amp_reg->size = count;
	argv.buf = amp_reg;
	argv.size = size;
	ret = frsm_amp_notify(frsm_amp,
			EVENT_AMP_REG_READ, &argv);
	if (ret > 0 && copy_to_user(buf, amp_reg->buf, count))
		ret = -EFAULT;

	dev_dbg(frsm_amp->dev, "read %02xh %d bytes\n", amp_reg->addr, ret);
	kfree(amp_reg);

	return ret <= 0 ? -EFAULT : count;
}

static ssize_t frsm_misc_write(struct file *fp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct frsm_amp_reg *amp_reg;
	struct frsm_amp *frsm_amp;
	struct frsm_argv argv;
	int size;
	int ret;

	if (fp == NULL || fp->private_data == NULL)
		return -EINVAL;

	frsm_amp = container_of((struct miscdevice *)fp->private_data,
			struct frsm_amp, misc_dev);

	size = sizeof(struct frsm_amp_reg) + count - 1;
	amp_reg = kzalloc(size, GFP_KERNEL);
	if (amp_reg == NULL)
		return -ENOMEM;

	ret = copy_from_user(&frsm_amp->addr, buf, sizeof(uint8_t));
	amp_reg->addr = frsm_amp->addr;
	amp_reg->size = count - 1; /* sizeof(addr) */
	ret |= copy_from_user(amp_reg->buf, buf + 1, count - 1);
	if (!ret) {
		argv.buf = amp_reg;
		argv.size = size;
		ret = frsm_amp_notify(frsm_amp,
				EVENT_AMP_REG_WRITE, &argv);
	} else {
		ret = -EFAULT;
	}

	dev_dbg(frsm_amp->dev, "write %02xh %d bytes\n", amp_reg->addr, ret);
	kfree(amp_reg);

	return ret <= 0 ? -EFAULT : count;
}

static const struct file_operations frsm_misc_fops = {
	.owner = THIS_MODULE,
	.open = frsm_misc_open,
	.read = frsm_misc_read,
	.write = frsm_misc_write,
	.release = frsm_misc_release,
};

static int frsm_amp_misc_init(struct frsm_amp *frsm_amp)
{
	int ret;

	if (frsm_amp == NULL || frsm_amp->dev == NULL)
		return -EINVAL;

	if (!of_property_read_bool(frsm_amp->dev->of_node,
			"frsm,misc-enable"))
		return 0;

	frsm_amp->misc_dev.name = FRSM_MISC_NAME,
	frsm_amp->misc_dev.minor = MISC_DYNAMIC_MINOR,
	frsm_amp->misc_dev.fops = &frsm_misc_fops,

	ret = misc_register(&frsm_amp->misc_dev);
	if (ret) {
		dev_err(frsm_amp->dev, "Failed to init miscdevice:%d\n", ret);
		return ret;
	}

	set_bit(FRSM_HAS_MISC, &frsm_amp->func);

	return 0;
}

static void frsm_amp_misc_deinit(struct frsm_amp *frsm_amp)
{
	if (frsm_amp && test_bit(FRSM_HAS_MISC, &frsm_amp->func))
		misc_deregister(&frsm_amp->misc_dev);
}

#endif // __FRSM_AMP_MISC_H__
