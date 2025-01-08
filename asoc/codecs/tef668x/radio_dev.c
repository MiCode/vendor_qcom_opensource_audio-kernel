#include "radio_dev.h"

static int radio_nr = -1;
static struct radio_device *g_rdev = NULL; // use list later

uint32_t log_debug = 1;
module_param(log_debug, uint, 0664);
MODULE_PARM_DESC(log_debug, "radio log debug");

static int rdev_vidioc_querycap(struct file *file, void *priv, struct v4l2_capability *capability)
{
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);
	strlcpy(capability->driver, RADIO_DRV_NAME, sizeof(capability->driver));
	strlcpy(capability->card, RADIO_DRV_CARD_NAME, sizeof(capability->card));
	sprintf(capability->bus_info, "I2C");

	return 0;
}

static int rdev_vidioc_g_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
	int ret = 0;
	uint8_t mode = 0;
	uint8_t band = 0;
	int16_t volume = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s \n", __func__);

	mutex_lock(&rdev->rdev_lock);

	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		ret = rdev->control.get_mute(rdev, &mode);
		ctrl->value = mode;
		rdev_debug(rdev->dev, "%s: mute = %hhu\n", __func__, mode);
		break;
	case V4L2_CID_RF_TUNER_BANDWIDTH:
		ret = rdev->control.get_band(rdev, &band);
		ctrl->value = band;
		rdev_debug(rdev->dev, "%s: band = %hhu\n", __func__, band);
		break;
	case V4L2_CID_AUDIO_VOLUME:
		ret = rdev->control.get_volume(rdev, &volume);
		ctrl->value = volume;
		rdev_debug(rdev->dev, "%s: volume = %hd\n", __func__, volume);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&rdev->rdev_lock);
	if (ret < 0)
		rdev_err(rdev->dev, "%s: get control failed, ret = %d, ctrl id = %u\n", __func__,
			 ret, ctrl->id);

	return ret;
}

static int rdev_vidioc_s_ctrl(struct file *file, void *priv, struct v4l2_control *ctrl)
{
	int ret = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	mutex_lock(&rdev->rdev_lock);
	switch (ctrl->id) {
	case V4L2_CID_AUDIO_MUTE:
		ret = rdev->control.set_mute(rdev, !!ctrl->value);
		break;
	case V4L2_CID_RF_TUNER_BANDWIDTH:
		ret = rdev->control.set_band(rdev, ctrl->value);
		break;
	case V4L2_CID_AUDIO_VOLUME:
		ret = rdev->control.set_volume(rdev, ctrl->value);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&rdev->rdev_lock);

	if (ret < 0)
		rdev_err(rdev->dev, "%s: set control failed,  ret = %d\n", __func__, ret);

	return ret;
}

/* refer to fm_v4l2_vidioc_g_audio */
static int rdev_vidioc_g_audio(struct file *file, void *priv, struct v4l2_audio *audio)
{
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	memset(audio, 0, sizeof(*audio));
	strscpy(audio->name, "Radio", sizeof(audio->name));
	audio->capability = V4L2_AUDCAP_STEREO;

	return 0;
}

/* refer to fm_v4l2_vidioc_s_audio */
static int rdev_vidioc_s_audio(struct file *file, void *priv, const struct v4l2_audio *audio)
{
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	if (audio->index != 0)
		return -EINVAL;

	return 0;
}

/* xiaomi need check, please refer to fm_v4l2_vidioc_g_tuner */
static int rdev_vidioc_g_tuner(struct file *file, void *priv, struct v4l2_tuner *v_tuner)
{
	int ret = 0;
	struct radio_device *rdev = video_drvdata(file);
	uint16_t bottom_freq;
	uint16_t top_freq;
	uint16_t stereo_mono_mode;

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}

	if (v_tuner->index != 0) {
		rdev_err(rdev->dev, "%s: get tuner failed\n", __func__);
		return -EINVAL;
	}

	rdev_debug(rdev->dev, "%s\n", __func__);
	mutex_lock(&rdev->rdev_lock);
	ret = g_rdev->control.get_freq_range(g_rdev, &bottom_freq, &top_freq);
	if (ret != 0)
		return ret;
	ret = g_rdev->control.get_stereo_mono(g_rdev, &stereo_mono_mode);
	if (ret != 0)
		return ret;
	mutex_unlock(&rdev->rdev_lock);

	strcpy(v_tuner->name, RADIO_DRV_NAME);
	v_tuner->type = V4L2_TUNER_RADIO;
	v_tuner->capability = V4L2_TUNER_CAP_STEREO | V4L2_TUNER_CAP_RDS | V4L2_TUNER_CAP_LOW |
			      V4L2_TUNER_CAP_HWSEEK_BOUNDED | V4L2_TUNER_CAP_HWSEEK_WRAP;
	v_tuner->rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
	v_tuner->audmode = (stereo_mono_mode ? V4L2_TUNER_MODE_STEREO : V4L2_TUNER_MODE_MONO);
	v_tuner->signal = 0; //need check, no rssi cmd in tef668x
	v_tuner->afc = 0;
	v_tuner->rangelow = bottom_freq * FREQ_MUL;
	v_tuner->rangehigh = top_freq * FREQ_MUL;

	return 0;
}

/* xiaomi need check, please refer to fm_v4l2_vidioc_s_tuner */
static int rdev_vidioc_s_tuner(struct file *file, void *priv, const struct v4l2_tuner *ptuner)
{
	int ret = 0;
	uint16_t aud_mode;
	uint16_t rds_mode;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);
	if (ptuner->index != 0)
		return -EINVAL;

	aud_mode = (ptuner->audmode == V4L2_TUNER_MODE_STEREO) ? RADIO_STEREO_MODE :
								 RADIO_MONO_MODE;
	rds_mode = (ptuner->rxsubchans & V4L2_TUNER_SUB_RDS) ? RADIO_RDS_ENABLE : RADIO_RDS_DISABLE;

	mutex_lock(&rdev->rdev_lock);
	ret = g_rdev->control.set_stereo_mono(g_rdev, aud_mode);
	mutex_unlock(&rdev->rdev_lock);
	if (ret < 0) {
		rdev_err(rdev->dev, "%s: Failed to set stereo/mono mode\n", __func__);
		return ret;
	}
	mutex_lock(&rdev->rdev_lock);
	ret = g_rdev->control.set_rds_mode(g_rdev, rds_mode);
	mutex_unlock(&rdev->rdev_lock);
	if (ret < 0)
		rdev_err(rdev->dev, "%s: Failed to set RX RDS mode\n", __func__);

	return ret;
}

static int rdev_vidioc_s_frequency(struct file *file, void *priv, const struct v4l2_frequency *freq)
{
	struct radio_device *rdev = video_drvdata(file);
	int ret = 0;

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s : set freq: %d\n", __func__, freq->frequency);

	mutex_lock(&rdev->rdev_lock);
	ret = rdev->control.set_freq(rdev, freq->frequency);
	mutex_unlock(&rdev->rdev_lock);

	if (ret < 0)
		rdev_err(rdev->dev, "%s: failed,  ret = %d\n", __func__, ret);

	return ret;
}

static int rdev_vidioc_g_frequency(struct file *file, void *priv, struct v4l2_frequency *freq)
{
	struct radio_device *rdev = video_drvdata(file);
	uint16_t cur_freq = 0;
	int ret = 0;

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&rdev->rdev_lock);
	freq->type = V4L2_TUNER_RADIO;
	ret = rdev->control.get_freq(rdev, &cur_freq);
	freq->frequency = cur_freq;
	mutex_unlock(&rdev->rdev_lock);

	rdev_debug(rdev->dev, "%s: freq = %hu\n", __func__, cur_freq);

	if (ret < 0)
		rdev_err(rdev->dev, "%s: failed,  ret = %d\n", __func__, ret);

	return ret;
}

static int rdev_vidioc_s_hw_freq_seek(struct file *file, void *priv,
				      const struct v4l2_hw_freq_seek *seek)
{
	int ret = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s: seek upward = %d, wrap_around = %d\n", __func__,
		   seek->seek_upward, seek->wrap_around);

	mutex_lock(&rdev->rdev_lock);
	ret = rdev->control.auto_seek(rdev, !!seek->seek_upward);
	mutex_unlock(&rdev->rdev_lock);

	if (ret < 0)
		rdev_err(rdev->dev, "%s: failed, ret = %d\n", __func__, ret);

	return ret;
}

/* TODO: refer to fm_v4l2_fops_read*/
static ssize_t rdev_fops_read(struct file *file, char __user *buf, size_t count, loff_t *f_pos)
{
	int ret = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	mutex_lock(&rdev->rdev_lock);
	// TODO : rds text read
	mutex_unlock(&rdev->rdev_lock);

	return ret;
}

/* TODO: refer to fm_v4l2_fops_write*/
static ssize_t rdev_fops_write(struct file *file, const char __user *buf, size_t count,
			       loff_t *ppos)
{
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);
	//ret = copy_from_user(buffer, buf, sizeof(buffer));
	mutex_lock(&rdev->rdev_lock);
	// TODO
	mutex_unlock(&rdev->rdev_lock);

	return count;
}

static unsigned int rdev_fops_poll(struct file *file, struct poll_table_struct *pts)
{
	unsigned int mask = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		return -EFAULT;
	}

	poll_wait(file, &rdev->read_queue, pts);

	mutex_lock(&rdev->rdev_lock);

	if (rdev->rd_index != rdev->wr_index)
		mask = (POLLIN | POLLRDNORM);

	mutex_unlock(&rdev->rdev_lock);

	return mask;
}

static int rdev_fops_open(struct file *file)
{
	int ret = 0;
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	return ret;
}

static int rdev_fops_release(struct file *file)
{
	struct radio_device *rdev = video_drvdata(file);

	if (!rdev) {
		rdev_err(rdev->dev, "%s: invalid param\n", __func__);
		return -EINVAL;
	}
	rdev_debug(rdev->dev, "%s\n", __func__);

	return 0;
}
/** define the v4l2 struct data **/
static const struct v4l2_ioctl_ops rdev_ioctl_ops = {
	.vidioc_querycap = rdev_vidioc_querycap,
	.vidioc_g_ctrl = rdev_vidioc_g_ctrl,
	.vidioc_s_ctrl = rdev_vidioc_s_ctrl,
	.vidioc_g_audio = rdev_vidioc_g_audio,
	.vidioc_s_audio = rdev_vidioc_s_audio,
	.vidioc_g_tuner = rdev_vidioc_g_tuner,
	.vidioc_s_tuner = rdev_vidioc_s_tuner,
	.vidioc_g_frequency = rdev_vidioc_g_frequency,
	.vidioc_s_frequency = rdev_vidioc_s_frequency,
	.vidioc_s_hw_freq_seek = rdev_vidioc_s_hw_freq_seek,
};

static const struct v4l2_file_operations rdev_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = rdev_fops_open,
	.read = rdev_fops_read,
	.write = rdev_fops_write,
	.poll = rdev_fops_poll,
	.release = rdev_fops_release,
};

static struct video_device rdev_video_dev = {
	.fops = &rdev_fops,
	.name = RADIO_DRV_NAME,
	.release = video_device_release_empty,
	.ioctl_ops = &rdev_ioctl_ops,
	.vfl_dir = VFL_DIR_M2M,
	.device_caps = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_TUNER | V4L2_CAP_RADIO |
		       V4L2_CAP_MODULATOR | V4L2_CAP_AUDIO | V4L2_CAP_READWRITE |
		       V4L2_CAP_RDS_CAPTURE,
};

static ssize_t rdev_mute_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			       size_t count)
{
	uint8_t mode = 0;
	int ret = kstrtou8(buf, 0, &mode);
	if (ret)
		return ret;
	if (g_rdev->control.set_mute) {
		rdev_info(g_rdev->dev, "set mute = %hhu\n", mode);
		g_rdev->control.set_mute(g_rdev, !!mode);
	}
	return count;
}

static ssize_t rdev_mute_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;
	uint8_t mode = 0;
	if (g_rdev->control.get_mute) {
		g_rdev->control.get_mute(g_rdev, &mode);
		rdev_info(g_rdev->dev, "mute status = %hhu\n", mode);
		len = scnprintf(buf, RADIO_DEV_MAX_BUF, "mute = %hhu\n", mode);
	}
	return len;
}

static ssize_t rdev_freq_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
			       size_t count)
{
	uint16_t freq = 0;
	int ret = kstrtou16(buf, 0, &freq);
	if (ret)
		return ret;
	if (g_rdev->control.set_freq) {
		rdev_info(g_rdev->dev, "buf %s freq %hu", buf, freq);
		g_rdev->control.set_freq(g_rdev, freq);
	}

	return count;
}

static ssize_t rdev_freq_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;
	uint16_t cur_freq = 0;
	if (g_rdev->control.get_freq) {
		g_rdev->control.get_freq(g_rdev, &cur_freq);
		rdev_info(g_rdev->dev, "freq = %hu\n", cur_freq);
		len = scnprintf(buf, RADIO_DEV_MAX_BUF, "freq = %hu\n", cur_freq);
	}
	return len;
}

static ssize_t rdev_volume_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
				 size_t count)
{
	int16_t vol = 0;
	int ret = kstrtos16(buf, 0, &vol);
	if (ret)
		return ret;
	if (g_rdev->control.set_volume) {
		rdev_info(g_rdev->dev, "volume %hd\n", vol);
		g_rdev->control.set_volume(g_rdev, vol);
	}

	return count;
}

static ssize_t rdev_volume_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;
	int16_t vol = 0;
	if (g_rdev->control.get_volume) {
		g_rdev->control.get_volume(g_rdev, &vol);
		rdev_info(g_rdev->dev, "volume = %hd\n", vol);
		len = scnprintf(buf, RADIO_DEV_MAX_BUF, "volume = %hd\n", vol);
	}
	return len;
}

static ssize_t rdev_auto_seek_store(struct kobject *kobj, struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	uint8_t dir = 0;
	int ret = kstrtou8(buf, 0, &dir);
	if (ret)
		return ret;
	if (g_rdev->control.auto_seek) {
		rdev_info(g_rdev->dev, "auto seek direction = %hhu\n", dir);
		g_rdev->control.auto_seek(g_rdev, !!dir);
	}

	return count;
}

static ssize_t rdev_auto_seek_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;
	uint16_t cur_freq = 0;
	if (g_rdev->control.get_freq) {
		g_rdev->control.get_freq(g_rdev, &cur_freq);
		rdev_info(g_rdev->dev, "freq = %hu\n", cur_freq);
		len = scnprintf(buf, RADIO_DEV_MAX_BUF, "freq = %hu\n", cur_freq);
	}
	return len;
}

static ssize_t rdev_step_seek_store(struct kobject *kobj, struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	uint8_t dir = 0;
	int ret = kstrtou8(buf, 0, &dir);
	if (ret)
		return ret;

	if (g_rdev->control.step_seek) {
		rdev_info(g_rdev->dev, "step seek direction = %hhu\n", dir);
		g_rdev->control.step_seek(g_rdev, !!dir);
	}

	return count;
}

static ssize_t rdev_step_seek_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	ssize_t len = 0;
	uint16_t cur_freq = 0;
	if (g_rdev->control.get_freq) {
		g_rdev->control.get_freq(g_rdev, &cur_freq);
		rdev_info(g_rdev->dev, "freq = %hu\n", cur_freq);
		len = scnprintf(buf, RADIO_DEV_MAX_BUF, "freq = %hu\n", cur_freq);
	}
	return len;
}

static ssize_t rdev_reset_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf,
				size_t count)
{
	uint8_t mode = 0;
	int ret = kstrtou8(buf, 0, &mode);
	if (ret)
		return ret;
	if (g_rdev->control.reset) {
		rdev_info(g_rdev->dev, "rdev reset radio device \n");
		g_rdev->control.reset(g_rdev);
	}
	return count;
}

static struct kobj_attribute mute_attr = __ATTR(mute, 0644, rdev_mute_show, rdev_mute_store);
static struct kobj_attribute freq_attr = __ATTR(freq, 0644, rdev_freq_show, rdev_freq_store);
static struct kobj_attribute volume_attr =
	__ATTR(volume, 0644, rdev_volume_show, rdev_volume_store);
static struct kobj_attribute auto_seek_attr =
	__ATTR(auto_seek, 0644, rdev_auto_seek_show, rdev_auto_seek_store);
static struct kobj_attribute step_seek_attr =
	__ATTR(step_seek, 0644, rdev_step_seek_show, rdev_step_seek_store);
static struct kobj_attribute reset_attr = __ATTR(reset, 0644, NULL, rdev_reset_store);

static struct attribute *radio_dev_attrs[] = { &mute_attr.attr,
					       &freq_attr.attr,
					       &volume_attr.attr,
					       &auto_seek_attr.attr,
					       &step_seek_attr.attr,
					       &reset_attr.attr,
					       NULL };

static const struct attribute_group radio_dev_attr_group = {
	.attrs = radio_dev_attrs,
};

static int rdev_sysfs_create(struct radio_device *rdev)
{
	int ret = -1;

	rdev->kobj = kobject_create_and_add("radio_dev", kernel_kobj);
	if (!(rdev->kobj)) {
		rdev_err(rdev->dev, "radio dev: failed to create sysfs kobject\n");
		return -EINVAL;
	}
	ret = sysfs_create_group(rdev->kobj, &radio_dev_attr_group);
	if (ret < 0) {
		rdev_err(rdev->dev, "error creating attr files");
		goto out;
	}
	return 0;

out:
	kobject_put(rdev->kobj);
	return ret;
}

static void rdev_sysfs_destroy(struct radio_device *rdev)
{
	if ((!rdev) || (!(rdev->kobj)))
		return;
	sysfs_remove_group(rdev->kobj, &radio_dev_attr_group);
	kobject_put(rdev->kobj);
}

int radiodev_controller_register(struct radio_device *rdev)
{
	int ret = -1;

	if (!rdev)
		return -EPERM;

	rdev_info(rdev->dev, "%s\n", __func__);

	mutex_init(&rdev->rdev_lock);
	g_rdev = rdev;

	strlcpy(rdev->v4l2_device.name, RADIO_DRV_NAME, sizeof(rdev->v4l2_device.name));

	ret = v4l2_device_register(NULL, &rdev->v4l2_device);
	if (ret < 0) {
		rdev_err(rdev->dev, "%s: v4l2_device_register failed, ret:%d\n", __func__, ret);
		goto out0;
	}

	video_set_drvdata(&rdev_video_dev, rdev);
	rdev_video_dev.v4l2_dev = &rdev->v4l2_device;

	ret = video_register_device(&rdev_video_dev, VFL_TYPE_RADIO, radio_nr);
	if (ret) {
		rdev_err(rdev->dev, "%s: video_register_device failed, ret:%d\n", __func__, ret);
		goto out1;
	}

	rdev->video_dev = &rdev_video_dev;

	ret = rdev_sysfs_create(rdev);
	if (ret < 0) {
		rdev_err(rdev->dev, "%s: rdev_sysfs_create failed, ret:%d\n", __func__, ret);
		goto out2;
	}

	rdev_info(rdev->dev, "%s end\n", __func__);

	return 0;
out2:
	video_unregister_device(rdev->video_dev);
	rdev->video_dev = NULL;
out1:
	v4l2_device_unregister(&rdev->v4l2_device);
out0:
	g_rdev = NULL;
	mutex_destroy(&rdev->rdev_lock);
	return ret;
}

void radiodev_controller_unregister(struct radio_device *rdev)
{
	if (!rdev)
		return;

	rdev_info(rdev->dev, "%s\n", __func__);
	/*rm attr node*/
	rdev_sysfs_destroy(rdev);
	video_unregister_device(rdev->video_dev);
	rdev->video_dev = NULL;
	v4l2_device_unregister(&rdev->v4l2_device);
	g_rdev = NULL;
	mutex_destroy(&rdev->rdev_lock);
}
