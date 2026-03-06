#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define SAMPLE_RATE      48000
#define NUM_CHANNELS     2
#define BIT_WIDTH        16
#define BLOCK_SAMPLES    480
#define BLOCK_SIZE_BYTES (BLOCK_SAMPLES * NUM_CHANNELS * (BIT_WIDTH / 8))
#define NUM_BLOCKS       4
#define TABLE_SIZE       48

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* 16-byte alignment for STM32N6 GPDMA */
K_MEM_SLAB_DEFINE(pool_tx, BLOCK_SIZE_BYTES, NUM_BLOCKS, 16);

static int16_t sine_table[TABLE_SIZE];

static void fill_sine_table(void)
{
	for (int i = 0; i < TABLE_SIZE; i++) {
		double rad = 2.0 * M_PI * i / TABLE_SIZE;
		sine_table[i] = (int16_t)(32767 * sin(rad));
	}
	LOG_INF("Sine table generated");
}

static void fill_audio_buffer(int16_t *buffer)
{
	static uint32_t index = 0;

	for (int i = 0; i < BLOCK_SAMPLES; i++) {
		int16_t val = sine_table[index++];
		if (index >= TABLE_SIZE) {
			index = 0;
		}

	/* Stereo: L + R */
	*buffer++ = val;
	*buffer++ = val;
	}
}

int main(void)
{
	const struct device *i2s_dev;
	struct i2s_config i2s_cfg;
	int ret;

	fill_sine_table();
	
	/* Optional: verify DMA device */
	const struct device *dma_dev = DEVICE_DT_GET(DT_NODELABEL(gpdma1));
	if (!device_is_ready(dma_dev)) {
		LOG_ERR("DMA device not ready!");
		return 0;
	}

	LOG_INF("DMA device ready");

	k_sleep(K_MSEC(100));

	i2s_dev = DEVICE_DT_GET(DT_ALIAS(sai_dac));
	if (!device_is_ready(i2s_dev)) {
		LOG_ERR("I2S device not ready!");
		return 0;
	}

	LOG_INF("I2S device found. Configuring...");

	i2s_cfg.word_size      = BIT_WIDTH;
	i2s_cfg.channels       = NUM_CHANNELS;
	i2s_cfg.format         = I2S_FMT_DATA_FORMAT_I2S;
	i2s_cfg.options        = I2S_OPT_BIT_CLK_CONTROLLER |
					I2S_OPT_FRAME_CLK_CONTROLLER;
	i2s_cfg.frame_clk_freq = SAMPLE_RATE;
	i2s_cfg.mem_slab       = &pool_tx;
	i2s_cfg.block_size     = BLOCK_SIZE_BYTES;
	i2s_cfg.timeout        = 1000;

	ret = i2s_configure(i2s_dev, I2S_DIR_TX, &i2s_cfg);

	if (ret < 0) {
		LOG_ERR("Failed to configure I2S: %d", ret);
	return 0;
	}

	/* ---------------------------------------------------- */
	/* IMPORTANT: Queue first buffer BEFORE START           */
	/* ---------------------------------------------------- */

	void *tx_block;

	ret = k_mem_slab_alloc(&pool_tx, &tx_block, K_FOREVER);
	if (ret < 0) {
		LOG_ERR("Failed to allocate initial TX buffer");
		return 0;
	}

	fill_audio_buffer((int16_t *)tx_block);

	ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE_BYTES);
	if (ret < 0) {
		LOG_ERR("Initial I2S write failed: %d", ret);
		return 0;
	}

	LOG_INF("Triggering START...");

	ret = i2s_trigger(i2s_dev, I2S_DIR_TX, I2S_TRIGGER_START);
	
	if (ret < 0) {
		LOG_ERR("Failed to start I2S: %d", ret);
		return 0;
	}

	LOG_INF("Playing sine wave...");

	/* Continuous streaming */
	while (1) {
		ret = k_mem_slab_alloc(&pool_tx, &tx_block, K_FOREVER);
		if (ret < 0) {
			LOG_ERR("Failed to allocate TX buffer");
			continue;
		}

	fill_audio_buffer((int16_t *)tx_block);

	ret = i2s_write(i2s_dev, tx_block, BLOCK_SIZE_BYTES);
	if (ret < 0) {
		LOG_ERR("I2S write error: %d", ret);
		k_mem_slab_free(&pool_tx, tx_block);
	}
	}

	return 0;
}
