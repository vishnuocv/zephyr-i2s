/* pmod_i2s.c — PMOD I2S audio output driver implementation */

#include "pmod_i2s.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(pmod_i2s, LOG_LEVEL_INF);

K_MEM_SLAB_DEFINE(tx_slab, PMOD_BLOCK_SIZE_BYTES, PMOD_NUM_BLOCKS, 16);

/* ---- Private state ------------------------------------------------------ */

static const struct device *i2s_dev;
static uint32_t stat_blocks;
static uint32_t stat_recovers;

/* ---- TX recover --------------------------------------------------------- */

static int tx_recover(void)
{
	stat_recovers++;
	LOG_WRN("SAI recover #%u (block=%u)", stat_recovers, stat_blocks);

	int ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_DROP);
	if (ret < 0) {
		LOG_ERR("I2S DROP failed: %d", ret);
		return ret;
	}

	k_sleep(K_MSEC(5));

	for (int i = 0; i < 2; i++) {
		void *blk;
		if (k_mem_slab_alloc(&tx_slab, &blk, K_MSEC(50)) < 0)
			continue;
		memset(blk, 0, PMOD_BLOCK_SIZE_BYTES);
		i2s_write(i2s_dev, blk, PMOD_BLOCK_SIZE_BYTES);
	}

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0)
		LOG_ERR("recover START failed: %d", ret);
	return ret;
}

/* ---- Init --------------------------------------------------------------- */

int pmod_i2s_init(void)
{
	i2s_dev = DEVICE_DT_GET(DT_ALIAS(PMOD_I2S_DT_ALIAS));
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready");
		return -ENODEV;
	}

	struct i2s_config cfg = {
		.word_size      = PMOD_BIT_WIDTH,
		.channels       = PMOD_NUM_CHANNELS,
		.format         = I2S_FMT_DATA_FORMAT_I2S,
		.options        = I2S_OPT_BIT_CLK_MASTER |
				  I2S_OPT_FRAME_CLK_MASTER,
		.frame_clk_freq = PMOD_SAMPLE_RATE,
		.mem_slab       = &tx_slab,
		.block_size     = PMOD_BLOCK_SIZE_BYTES,
		.timeout        = 1000,
	};

	int ret = i2s_configure(i2s_dev, I2S_DIR_TX, &cfg);
	if (ret < 0) {
		LOG_ERR("i2s_configure failed: %d", ret);
		return ret;
	}

	/* Prefill TX queue with silence */
	for (int i = 0; i < PMOD_PREFILL_BLOCKS; i++) {
		void *blk;
		if (k_mem_slab_alloc(&tx_slab, &blk, K_FOREVER) == 0) {
			memset(blk, 0, PMOD_BLOCK_SIZE_BYTES);
			ret = i2s_write(i2s_dev, blk, PMOD_BLOCK_SIZE_BYTES);
			if (ret < 0) {
				k_mem_slab_free(&tx_slab, blk);
				LOG_ERR("prefill write failed: %d", ret);
				return ret;
			}
		}
	}

	LOG_INF("PMOD I2S init OK: %u Hz / %u-bit / %u-ch",
		PMOD_SAMPLE_RATE, PMOD_BIT_WIDTH, PMOD_NUM_CHANNELS);
	return 0;
}

/* ---- Start -------------------------------------------------------------- */

int pmod_i2s_start(void)
{
	int ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (ret < 0) {
		LOG_ERR("I2S START failed: %d", ret);
		return ret;
	}
	LOG_INF("PMOD I2S TX started");
	return 0;
}

/* ---- Write -------------------------------------------------------------- */

int pmod_i2s_write_mono32(const int32_t *samples, uint32_t count)
{
	void *blk;
	int ret = k_mem_slab_alloc(&tx_slab, &blk, K_MSEC(15));
	if (ret < 0) {
		LOG_WRN("TX slab alloc failed");
		return tx_recover();
	}

	int16_t *dst = blk;
	for (uint32_t i = 0; i < count; i++) {
		int32_t s = samples[i] >> PMOD_SHIFT_BITS;
		s = (s > 32767) ? 32767 : (s < -32768) ? -32768 : s;
		*dst++ = (int16_t)s;
		*dst++ = (int16_t)s;
	}
	stat_blocks++;

	ret = i2s_write(i2s_dev, blk, PMOD_BLOCK_SIZE_BYTES);
	if (ret < 0) {
		k_mem_slab_free(&tx_slab, blk);
		if (ret == -EBUSY || ret == -EAGAIN) {
			stat_recovers++;
			return ret;
		}
		return tx_recover();
	}
	return 0;
}

/* ---- Stats -------------------------------------------------------------- */

void pmod_i2s_get_stats(struct pmod_i2s_stats *stats)
{
	stats->block_count   = stat_blocks;
	stats->recover_count = stat_recovers;
}
