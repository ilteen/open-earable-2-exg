/*
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "hw_codec.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(hw_codec_stub, CONFIG_MODULE_HW_CODEC_LOG_LEVEL);

ZBUS_SUBSCRIBER_DEFINE(volume_evt_sub, CONFIG_VOLUME_MSG_SUB_QUEUE_SIZE);

static enum audio_mode current_mode = AUDIO_MODE_NORMAL;

int hw_codec_volume_set(uint8_t set_val)
{
	ARG_UNUSED(set_val);
	return 0;
}

int hw_codec_volume_adjust(int8_t adjustment)
{
	ARG_UNUSED(adjustment);
	return 0;
}

int hw_codec_volume_decrease(void)
{
	return 0;
}

int hw_codec_volume_increase(void)
{
	return 0;
}

int hw_codec_volume_mute(void)
{
	return 0;
}

int hw_codec_volume_unmute(void)
{
	return 0;
}

int hw_codec_default_conf_enable(void)
{
	return 0;
}

int hw_codec_soft_reset(void)
{
	return 0;
}

int hw_codec_init(void)
{
	LOG_INF("HW codec disabled: using stub");
	return 0;
}

int hw_codec_stop_audio(void)
{
	return 0;
}

int hw_codec_set_audio_mode(enum audio_mode mode)
{
	current_mode = mode;
	return 0;
}

enum audio_mode hw_codec_get_audio_mode(void)
{
	return current_mode;
}
