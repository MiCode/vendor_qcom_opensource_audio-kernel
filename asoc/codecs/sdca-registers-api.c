// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */
#include <asoc/sdca-registers-api.h>
#include <linux/component.h>

#define RW_BUF_LEN    (20)
#define SDCA_REG_BASE    (0x40000000)
#define BYTES_PER_LINE   (20)

static ssize_t codec_debug_address_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[RW_BUF_LEN];
	struct sdca_regdump_info *regdump_info = NULL;
	struct snd_soc_component *component = NULL;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	regdump_info = file->private_data;
	if (!regdump_info)
		return -EINVAL;

	component = regdump_info->component;
	if (!component)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	snprintf(buf, sizeof(buf), "0x%x\n", regdump_info->reg_address);
	return simple_read_from_buffer(ubuf, count, ppos, buf,
						RW_BUF_LEN);
}

static ssize_t codec_debug_address_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char buf[RW_BUF_LEN];
	int rc = 0, ret = 0;
	struct sdca_regdump_info *regdump_info = NULL;
	struct snd_soc_component *component = NULL;

	if (!file || !ppos || !ubuf)
		return -EINVAL;

	regdump_info = file->private_data;
	if (!regdump_info)
		return -EINVAL;

	component = regdump_info->component;
	if (!component)
		return -EINVAL;

	if (cnt > sizeof(buf) - 1)
		return -EINVAL;

	rc = copy_from_user(buf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	buf[cnt] = '\0';
	ret = kstrtouint(buf, 0, &regdump_info->reg_address);
	if (ret) {
		dev_dbg(component->dev, "%s: convert data string failed\n", __func__);
		return -EINVAL;
	}
	return cnt;
}

static ssize_t codec_debug_data_read(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[RW_BUF_LEN];
	struct sdca_regdump_info *regdump_info = NULL;
	struct snd_soc_component *component = NULL;
	int reg_val = 0;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	regdump_info = file->private_data;
	if (!regdump_info)
		return -EINVAL;

	component = regdump_info->component;
	if (!component)
		return -EINVAL;

	if (*ppos < 0)
		return -EINVAL;

	if (regdump_info->reg_address > SDCA_REG_BASE) {
		if (regdump_info->sdca_readable_register(regdump_info->reg_address)) {
			reg_val = snd_soc_component_read(component, regdump_info->reg_address);
		} else {
			dev_err(component->dev, "%s: 0x%0x isnt't readable\n",
				__func__, regdump_info->reg_address);
			return -EINVAL;
		}
	} else {
		dev_err(component->dev, "%s: invalid reg_address\n",
			__func__);
		return -EINVAL;
	}

	snprintf(buf, sizeof(buf), "0x%.8x: 0x%.2x\n", regdump_info->reg_address, reg_val);
	return simple_read_from_buffer(ubuf, count, ppos, buf,
						RW_BUF_LEN);
}

static ssize_t codec_debug_data_write(struct file *file,
	const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	char buf[RW_BUF_LEN];
	struct sdca_regdump_info *regdump_info = NULL;
	struct snd_soc_component *component = NULL;
	int data = 0, rc = 0, ret = 0;

	regdump_info = file->private_data;
	if (!regdump_info)
		return -EINVAL;

	component = regdump_info->component;
	if (!component)
		return -EINVAL;


	if (cnt > sizeof(buf) - 1)
		return -EINVAL;

	rc = copy_from_user(buf, ubuf, cnt);
	if (rc)
		return -EFAULT;

	buf[cnt] = '\0';
	ret = kstrtouint(buf, 0, &data);
	if (ret) {
		dev_dbg(component->dev, "%s: convert data string failed\n", __func__);
		return -EINVAL;
	}

	if (regdump_info->reg_address > SDCA_REG_BASE) {
		if (regdump_info->sdca_writeable_register(regdump_info->reg_address)) {
			snd_soc_component_update_bits(component, regdump_info->reg_address,
				0xFF, data);
		} else {
			dev_err(component->dev, "%s: 0x%0x isnt't writeable\n",
					__func__, regdump_info->reg_address);
			return -EINVAL;
		}
	} else {
		dev_err(component->dev, "%s: the reg isn't in the range\n",
				__func__);
		return -EINVAL;
	}

	return cnt;
}

static ssize_t codec_debug_registers_dump(struct file *file, char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct sdca_regdump_info *regdump_info = NULL;
	struct snd_soc_component *component = NULL;
	char tmp_buf[RW_BUF_LEN];
	int i = 0, reg_val = 0, len = 0;
	ssize_t total = 0;

	if (!count || !file || !ppos || !ubuf)
		return -EINVAL;

	regdump_info = file->private_data;
	if (!regdump_info)
		return -EINVAL;

	component = regdump_info->component;
	if (!component)
		return -EINVAL;

	for (i = ((int) *ppos/BYTES_PER_LINE); i < regdump_info->reg_num; i++) {
		if (regdump_info->sdca_readable_register(regdump_info->reg_array[i])) {
			reg_val = snd_soc_component_read(component, regdump_info->reg_array[i]);
			len = scnprintf(tmp_buf, BYTES_PER_LINE, "0x%.8x: 0x%.2x\n",
						regdump_info->reg_array[i], (reg_val & 0xFF));
			if (((total + len) >= count - 1) || (len < 0))
				break;

			if (copy_to_user((ubuf + total), tmp_buf, len)) {
				total = -EFAULT;
				goto copy_err;
			}
			total += len;
			*ppos += len;
		}
	}

copy_err:
	*ppos = i * BYTES_PER_LINE;
	return total;
}

static const struct file_operations codec_debug_address_ops = {
	.open = simple_open,
	.read = codec_debug_address_read,
	.write = codec_debug_address_write,
};

static const struct file_operations codec_debug_data_ops = {
	.open = simple_open,
	.read = codec_debug_data_read,
	.write = codec_debug_data_write,
};

static const struct file_operations codec_debug_registers_ops = {
	.open = simple_open,
	.read = codec_debug_registers_dump,
};

void sdca_devices_debugfs_dentry_create(
		struct sdca_debugfs_info *debugfs_info,
		struct sdca_regdump_info *regdump_info)
{
	struct snd_soc_component *component =
					regdump_info->component;

	if ((!regdump_info->sdca_readable_register) ||
		(!regdump_info->sdca_writeable_register)) {
		dev_err(component->dev, "%s: read/write access check table is NULL\n",
			__func__);
		return;
	}
	debugfs_info->sdca_dentry = debugfs_create_dir(
					dev_name(component->dev), 0);

	if (!IS_ERR(debugfs_info->sdca_dentry)) {
		debugfs_info->address =
				debugfs_create_file("address",
				S_IFREG | 0444,
				debugfs_info->sdca_dentry,
				(void *) regdump_info,
				&codec_debug_address_ops);

		debugfs_info->data =
				debugfs_create_file("data",
				S_IFREG | 0444,
				debugfs_info->sdca_dentry,
				(void *) regdump_info,
				&codec_debug_data_ops);

		debugfs_info->registers =
				debugfs_create_file("registers",
				S_IFREG | 0444,
				debugfs_info->sdca_dentry,
				(void *) regdump_info,
				&codec_debug_registers_ops);
	}

}
EXPORT_SYMBOL_GPL(sdca_devices_debugfs_dentry_create);

void sdca_devices_debugfs_dentry_remove(struct sdca_debugfs_info *debugfs_info)
{
	if (debugfs_info) {
		debugfs_remove_recursive(debugfs_info->sdca_dentry);
		debugfs_info->sdca_dentry = NULL;
	}
}
EXPORT_SYMBOL_GPL(sdca_devices_debugfs_dentry_remove);

MODULE_LICENSE("GPL");

