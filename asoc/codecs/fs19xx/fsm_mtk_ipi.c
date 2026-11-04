/**
 * Copyright (C) Fourier Semiconductor Inc. 2016-2020. All rights reserved.
 * 2020-08-20 File created.
 */

#include "fsm_public.h"
#ifdef CONFIG_FSM_MTK
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <sound/soc.h>

//static atomic_t fsm_afe_state;
static int g_fsm_re25[FSM_DEV_MAX] = { 0 };
static bool read_re25_done = false;
struct mutex fsm_q6afe_lock;

#ifdef FSM_RUNIN_TEST
static atomic_t fsm_module_switch;
static atomic_t fsm_runin_test;


static int fsm_afe_test_ctrl(int index)
{
	struct fsm_afe afe;
	fsm_config_t *cfg;
	int runin_test;
	int ret;

	switch (index) {
		case FSM_TC_DISABLE_ALL:
			runin_test = 0; // disable all
			break;
		case FSM_TC_DISABLE_EQ:
			runin_test = 3; // disable eq only
			break;
		case FSM_TC_DISABLE_PROT:
			runin_test = 5; // disable protection only
			break;
		case FSM_TC_ENABLE_ALL:
		default:
			runin_test = 1; // enable all
			break;
	}
	cfg = fsm_get_config();
	pr_info("spkon:%d, testcase: %d", cfg->speaker_on, runin_test);
	if (!cfg->speaker_on) {
		atomic_set(&fsm_runin_test, index);
		return 0;
	}
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_MODULE_ENABLE;
	afe.op_set = true;
	ret = fsm_afe_send_apr(&afe, &runin_test, sizeof(runin_test));
	if (ret) {
		pr_err("send apr failed:%d", ret);
		return ret;
	}
	atomic_set(&fsm_runin_test, index);

	return ret;
}

static int fsm_afe_module_ctrl(int index)
{
	struct fsm_afe afe;
	int enable;
	int ret;

	afe.module_id = AFE_MODULE_ID_FSADSP_TX;
	afe.port_id = fsm_afe_get_tx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_TX_ENABLE;
	afe.op_set = true;
	enable = !!index;
	ret = fsm_afe_send_apr(&afe, &enable, sizeof(enable));
	if (ret) {
		pr_err("send apr failed:%d", ret);
		return ret;
	}
	// fsm_delay_ms(50);
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_RX_ENABLE;
	afe.op_set = true;
	enable = !!index;
	ret = fsm_afe_send_apr(&afe, &enable, sizeof(enable));
	if (ret) {
		pr_err("send apr failed:%d", ret);
		return ret;
	}
	atomic_set(&fsm_module_switch, index);
	ret = fsm_afe_test_ctrl(atomic_read(&fsm_runin_test));
	if (ret) {
		pr_err("test ctrl failed:%d", ret);
		return ret;
	}

	return ret;
}
#endif

extern int mtk_spk_send_ipi_buf_to_dsp(
		void *data_buffer, uint32_t data_size);
extern int mtk_spk_recv_ipi_buf_from_dsp(
		int8_t *buffer, int16_t size, uint32_t *buf_len);

int fsm_afe_send_apr(struct fsm_afe *afe, void *buf, uint32_t length)
{
	int rcv_length = 0;
	uint32_t *buf32;
	int ret;

	if (afe == NULL || buf == NULL) {
		pr_err("Bad parameters");
		return -EINVAL;
	}

	pr_info("param_id:0x%x, length:%d", afe->param_id, length);

	mutex_lock(&fsm_q6afe_lock);
	length += sizeof(uint32_t);
	buf32 = fsm_alloc_mem(length);
	if (buf32 == NULL) {
		pr_err("allocate memory failed");
		mutex_unlock(&fsm_q6afe_lock);
		return -ENOMEM;
	}

	buf32[0] = afe->param_id;
	if (afe->op_set) {
		memcpy(&buf32[1], buf, length);
		ret = mtk_spk_send_ipi_buf_to_dsp((void *)buf32, length);
		if (ret)
			pr_err("send ipi buf failed:%d", ret);
	} else {
		ret = mtk_spk_send_ipi_buf_to_dsp((void *)buf32,
				sizeof(uint32_t));
		ret |= mtk_spk_recv_ipi_buf_from_dsp((void *)buf32,
				length, &rcv_length);
		if (rcv_length <= length) {
			if (!ret)
				memcpy(buf, buf32, rcv_length);
		} else {
			pr_err("recv size error:%d", rcv_length);
			ret = -EINVAL;
		}
		if (ret)
			pr_err("recv ipi buf failed:%d", ret);
	}

	if (buf32)
		fsm_free_mem((void **)&buf32);
	mutex_unlock(&fsm_q6afe_lock);

	return ret;
}

#ifdef CONFIG_FSM_CROSSFADE
#define CAPI_V2_PARAM_FSADSP_CROSSFADE 0x10001FB8

struct fsm_crossfade_params {
	int mode;
	int targetdb;
	int type1;
	int transitiont1;
	int type2;
	int transitiont2;
	int holdt;
	int ch_section[4];
};

static int dir_new, dir_old;

int fsm_afe_swap_channels(bool force)
{
	struct fsm_crossfade_params *params;
	struct fsm_afe afe;
	int payload[32];
	int size;
	int ret;

	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id = CAPI_V2_PARAM_FSADSP_CROSSFADE;
	afe.op_set = true;

//	if (afe_get_topology(afe.port_id) <= 0) {
//		pr_info("None topology is found");
//		return 0;
//	}

	if (!force && dir_old == dir_new) {
		pr_info("Same direction: %d", dir_new);
		return 0;
	}

	memset(payload, 0, sizeof(payload));
	params = (struct fsm_crossfade_params *)payload;
	size = sizeof(struct fsm_crossfade_params);

	params->mode = 3; // Cross Fade
	params->type1 = 0; // Linear
	params->transitiont1 = 10; // time1: 0.01 * 1000
	switch (dir_new) {
	case 0:
		params->ch_section[0] = 0;
		params->ch_section[1] = 1;
		params->ch_section[2] = 2;
		params->ch_section[3] = 3;
		break;
	case 90:
		params->ch_section[0] = 1;
		params->ch_section[1] = 3;
		params->ch_section[2] = 0;
		params->ch_section[3] = 2;
		break;
	case 180:
		params->ch_section[0] = 3;
		params->ch_section[1] = 2;
		params->ch_section[2] = 1;
		params->ch_section[3] = 0;
		break;
	case 270:
		params->ch_section[0] = 2;
		params->ch_section[1] = 0;
		params->ch_section[2] = 3;
		params->ch_section[3] = 1;
		break;
	default:
		pr_err("Invalid direction: %d", dir_new);
		return -EINVAL;
	}

	ret = fsm_afe_send_apr(&afe, (uint8_t *)payload, size);
	if (ret) {
		pr_err("send apr failed: %d", ret);
		return ret;
	}

	dir_old = dir_new;

	return 0;
}

int fsm_afe_handle_direction(bool is_set, int *dir)
{
	int ret;

	if (!is_set) {
		*dir = dir_old;
		return 0;
	}

	dir_new = *dir;
	ret = fsm_afe_swap_channels(false);

	return ret;
}
EXPORT_SYMBOL(fsm_afe_handle_direction);
#endif

int fsm_afe_read_re25(uint32_t *re25, int count)
{
	struct file *fp;
	//mm_segment_t fs;
	loff_t pos = 0;
	int i, len, result;

	if (read_re25_done) {
		memcpy(re25, g_fsm_re25, sizeof(int) * count);
		return 0;
	}

	if (re25 == NULL || count <= 0) {
		pr_err("bad parameters");
		return -EINVAL;
	}
	fp = filp_open(FSM_CALIB_FILE, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		pr_err("open %s fail!", FSM_CALIB_FILE);
		// set_fs(fs);
		return -EINVAL;
	}
	len = count * sizeof(int);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	result = kernel_read(fp, pos, (char *)re25, len);
#else
	result = kernel_read(fp, (char *)re25, len, &pos);
	//fs = get_fs();
	//set_fs(KERNEL_DS);
	//result = vfs_read(fp, (char *)re25, len, &pos);
	//set_fs(fs);
#endif
	filp_close(fp, NULL);
	if (result <= 0) {
		pr_err("read read fail:%d", result);
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		g_fsm_re25[i] = re25[i];
		pr_info("re25.%d: %d", i, re25[i]);
	}
	read_re25_done = true;
	pr_info("read %s success!", FSM_CALIB_FILE);

	return 0;
}

int fsm_afe_write_re25(uint32_t *re25, int count)
{
	struct file *fp;
	//mm_segment_t fs;
	loff_t pos = 0;
	int i, len, result;

	if (re25 == NULL || count <= 0) {
		pr_err("bad parameters");
		return -EINVAL;
	}
	for (i = 0; i < count; i++) {
		pr_info("re25.%d: %d", i, re25[i]);
	}
	fp = filp_open(FSM_CALIB_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		pr_err("open %s fail!", FSM_CALIB_FILE);
		// set_fs(fs);
		return PTR_ERR(fp);
	}
	len = count * sizeof(int);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	result = kernel_write(fp, (char *)re25, len, pos);
#else
	result = kernel_write(fp, (char *)re25, len, &pos);
	//fs = get_fs();
	//set_fs(KERNEL_DS);
	//result = vfs_write(fp, (char *)re25, len, &pos);
	//set_fs(fs);
#endif
	if (result != len) {
		pr_err("write file fail:%d, len:%d", result, len);
		filp_close(fp, NULL);
		return -ENOMEM;
	}
	filp_close(fp, NULL);
	// set_fs(fs);
	pr_info("write %s success!", FSM_CALIB_FILE);

	return 0;
}

int fsm_afe_save_re25(struct fsadsp_cmd_re25 *cmd_re25)
{
	int payload[FSM_CALIB_PAYLOAD_SIZE];
	uint32_t re25[FSM_DEV_MAX] = { 0 };
	struct preset_file *preset;
	fsm_dev_t *fsm_dev;
	struct fsm_afe afe;
	int dev, index;
	int ret;

	fsm_reset_re25_data();
	preset = (struct preset_file *)fsm_get_presets();
	if (preset == NULL) {
		pr_err("not found firmware");
		return -EINVAL;
	}
	memset(cmd_re25, 0, sizeof(struct fsadsp_cmd_re25));
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_CALIB;
	afe.op_set = false;
	ret = fsm_afe_send_apr(&afe, payload, sizeof(payload));
	if (ret) {
		pr_err("send apr failed:%d", ret);
		return ret;
	}
	cmd_re25->ndev = preset->hdr.ndev;
	pr_info("ndev:%d", cmd_re25->ndev);
	for (dev = 0; dev < cmd_re25->ndev; dev++) {
		fsm_dev = fsm_get_fsm_dev_by_id(dev);
		if (fsm_dev == NULL || fsm_skip_device(fsm_dev)) {
			continue;
		}
		index = fsm_get_index_by_position(fsm_dev->pos_mask);
		re25[index] = payload[6*index];
		fsm_dev->re25 = re25[index];
		cmd_re25->cal_data[index].rstrim  = LOW8(fsm_dev->rstrim);
		cmd_re25->cal_data[index].channel = fsm_dev->pos_mask;
		cmd_re25->cal_data[index].re25    = fsm_dev->re25;
		pr_info("re25.%d[%X]:%d", index,
				fsm_dev->pos_mask, fsm_dev->re25);
	}
	ret = fsm_afe_write_re25(re25, cmd_re25->ndev);
	if (ret) {
		pr_err("write re25 fail:%d", ret);
		return ret;
	}
	memcpy(g_fsm_re25, re25, sizeof(int) * cmd_re25->ndev);

	return ret;
}

int fsm_afe_mod_ctrl(bool enable)
{
	fsm_config_t *cfg = fsm_get_config();
	struct fsadsp_cmd_re25 *params;
	uint32_t re25[FSM_DEV_MAX] = { 0 };
	struct preset_file *preset;
	struct fsm_afe afe;
	fsm_dev_t *fsm_dev;
	int param_size;
	int index;
	int dev;
	int ret;

	if (!enable || cfg->dev_count == 0) {
		return 0;
	}
	param_size = sizeof(struct fsadsp_cmd_re25);
	// + cfg->dev_count * sizeof(struct fsadsp_cal_data);
	params = (struct fsadsp_cmd_re25 *)fsm_alloc_mem(param_size);
	if (params == NULL) {
		pr_err("allocate memory failed");
		return -EINVAL;
	}
	memset(params, 0, param_size);
	params->version = FSADSP_RE25_CMD_VERSION_V1;
	preset = (struct preset_file *)fsm_get_presets();
	if (preset == NULL) {
		pr_err("not found firmware");
		return -EINVAL;
	}
	params->ndev = preset->hdr.ndev;
	ret = fsm_afe_read_re25(&re25[0], params->ndev);
	if (ret) {
		pr_err("read back re25 fail:%d", ret);
	}
	for (dev = 0; dev < cfg->dev_count; dev++) {
		fsm_dev = fsm_get_fsm_dev_by_id(dev);
		if (fsm_dev == NULL || fsm_skip_device(fsm_dev))
			continue;
		index = fsm_get_index_by_position(fsm_dev->pos_mask);
		fsm_dev->re25 = re25[index];
		params->cal_data[index].rstrim  = LOW8(fsm_dev->rstrim);
		params->cal_data[index].channel = fsm_dev->pos_mask;
		params->cal_data[index].re25    = fsm_dev->re25;
		pr_info("re25.%d[%X]:%d", index, fsm_dev->pos_mask,
				fsm_dev->re25);
	}
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_RE25;
	afe.op_set = true;
	ret = fsm_afe_send_apr(&afe, params, param_size);
	fsm_free_mem((void **)&params);
	if (ret) {
		pr_err("send re25 failed:%d", ret);
		return ret;
	}
#ifdef FSM_RUNIN_TEST
	ret = fsm_afe_test_ctrl(atomic_read(&fsm_runin_test));
	if (ret) {
		pr_err("test ctrl failed:%d", ret);
		return ret;
	}
#endif
#ifdef CONFIG_FSM_CROSSFADE
	ret = fsm_afe_swap_channels(true);
#endif

	return ret;
}

void fsm_reset_re25_data(void)
{
	memset(g_fsm_re25, 0, sizeof(g_fsm_re25));
	read_re25_done = false;
}

int fsm_set_re25_data(struct fsm_re25_data *data)
{
	fsm_config_t *cfg = fsm_get_config();
	int ret = 0;
	int dev;

	if (cfg == NULL || data == NULL) {
		return -EINVAL;
	}
	if (data->count <= 0) {
		pr_err("invalid dev count");
		return -EINVAL;
	}
	for (dev = 0; dev < data->count; dev++) {
		g_fsm_re25[dev] = data->re25[dev];
	}
	read_re25_done = true;
	if (cfg->speaker_on) {
		ret = fsm_afe_mod_ctrl(true);
		if (ret) {
			pr_err("update re25 failed:%d", ret);
		}
	}

	return ret;
}

int fsm_afe_get_livedata(void *ldata, int size)
{
	struct fsm_afe afe;
	int ret;

	if (ldata == NULL) {
		return -EINVAL;
	}
	if (fsm_get_config()->force_calib)
		memset(g_fsm_re25, 0 , sizeof(g_fsm_re25));
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_CALIB;
	afe.op_set = false;
	ret = fsm_afe_send_apr(&afe, ldata, size);
	if (ret) {
		pr_err("send apr failed:%d", ret);
	}
	return ret;
}

int fsm_afe_send_preset(char *preset)
{
	const struct firmware *firmware;
	struct device *dev;
	struct fsm_afe afe;
	int ret;

	if ((dev = fsm_get_pdev()) == NULL) {
		pr_err("bad dev parameter");
		return -EINVAL;
	}
	ret = request_firmware(&firmware, preset, dev);
	if (ret) {
		pr_err("request firmware failed");
		return ret;
	}
	if (firmware->data == NULL && firmware->size <= 0) {
		pr_err("can't read firmware");
		return -EINVAL;
	}
	pr_debug("sending rx %s", preset);
	afe.module_id = AFE_MODULE_ID_FSADSP_RX;
	afe.port_id = fsm_afe_get_rx_port();
	afe.param_id  = CAPI_V2_PARAM_FSADSP_MODULE_PARAM;
	afe.op_set = true;
	ret = fsm_afe_send_apr(&afe, (void *)firmware->data,
			(uint32_t)firmware->size);
	if (ret) {
		pr_err("send apr failed:%d", ret);
	}
	release_firmware(firmware);

	return ret;
}

#ifdef FSM_RUNIN_TEST
static const char *fsm_afe_switch_text[] = {
	"Off", "On"
};

static const struct soc_enum fsm_afe_switch_enum[] = {
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(fsm_afe_switch_text),
			fsm_afe_switch_text)
};

static int fsm_afe_module_switch_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int index = atomic_read(&fsm_module_switch);

	pUcontrol->value.integer.value[0] = index;
	pr_info("Switch %s", fsm_afe_switch_text[index]);

	return 0;
}

static int fsm_afe_module_switch_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int index = ucontrol->value.integer.value[0];
	int ret;

	pr_info("Switch %s", fsm_afe_switch_text[index]);
	ret = fsm_afe_module_ctrl(index);
	if (ret) {
		pr_err("module ctrl failed:%d", ret);
		return ret;
	}

	return 0;
}

static int fsm_afe_runin_test_get(struct snd_kcontrol *pKcontrol,
				struct snd_ctl_elem_value *pUcontrol)
{
	int index = atomic_read(&fsm_runin_test);

	pUcontrol->value.integer.value[0] = index;
	pr_info("case: %d", index);

	return 0;
}

static int fsm_afe_runin_test_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	int index = ucontrol->value.integer.value[0];
	int ret;

	pr_info("case: %d", index);
	ret = fsm_afe_test_ctrl(index);
	if (ret) {
		pr_err("test ctrl failed:%d", ret);
		return ret;
	}

	return 0;
}

static const struct snd_kcontrol_new fsm_afe_controls[] = {
	SOC_ENUM_EXT("FSM_Module_Switch", fsm_afe_switch_enum[0],
			fsm_afe_module_switch_get, fsm_afe_module_switch_set),
	SOC_SINGLE_EXT("FSM_Runin_Test", SND_SOC_NOPM, 0, FSM_TC_MAX, 0,
			fsm_afe_runin_test_get, fsm_afe_runin_test_set),
};
#endif

void fsm_afe_init_controls(struct snd_soc_codec *codec)
{
	mutex_init(&fsm_q6afe_lock);
#ifdef FSM_RUNIN_TEST
	atomic_set(&fsm_module_switch, 0);
	atomic_set(&fsm_runin_test, 0);
	snd_soc_add_codec_controls(codec, fsm_afe_controls,
			ARRAY_SIZE(fsm_afe_controls));
#endif
}
#endif
