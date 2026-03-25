/*
 * PDM mic (ADF1 DMA) -> PMOD I2S2 (SAI2_A) passthrough
 * NUCLEO-N657X0-Q  —  Zephyr v4.3+
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "adf_pdm.h"
#include "pmod_i2s.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static int32_t pcm_buf[ADF_BLOCK_SAMPLES];

int main(void)
{
	int ret;

	/* ADF PDM microphone */
	ret = adf_pdm_init();
	if (ret < 0) { LOG_ERR("adf_pdm_init: %d", ret); return 0; }

	ret = adf_pdm_start();
	if (ret < 0) { LOG_ERR("adf_pdm_start: %d", ret); return 0; }

	/* Wait for initial ADF DMA fill */
	uint32_t t0 = k_uptime_get_32();
	while (adf_pdm_available() < ADF_BLOCK_SAMPLES * PMOD_PREFILL_BLOCKS) {
		if (k_uptime_get_32() - t0 > 2000) {
			LOG_ERR("ADF DMA timeout");
			while (1) k_sleep(K_SECONDS(1));
		}
		k_sleep(K_MSEC(1));
	}
	LOG_INF("ADF ready (%u ms)", (unsigned)(k_uptime_get_32() - t0));

	/* PMOD I2S audio output */
	ret = pmod_i2s_init();
	if (ret < 0) { LOG_ERR("pmod_i2s_init: %d", ret); return 0; }

	ret = pmod_i2s_start();
	if (ret < 0) { LOG_ERR("pmod_i2s_start: %d", ret); return 0; }

	LOG_INF("PDM mic -> I2S passthrough running");

	/* Main loop */
#ifdef CONFIG_APP_AUDIO_DIAG
	uint32_t tick = 0;
#endif
	while (1) {
		ret = adf_pdm_read(pcm_buf, ADF_BLOCK_SAMPLES, K_MSEC(50));
		if (ret == 0)
			pmod_i2s_write_mono32(pcm_buf, ADF_BLOCK_SAMPLES);

#ifdef CONFIG_APP_AUDIO_DIAG
		if (++tick >= 500) {
			tick = 0;
			struct adf_pdm_stats  ast;
			struct pmod_i2s_stats ist;
			adf_pdm_get_stats(&ast);
			pmod_i2s_get_stats(&ist);
			uint32_t now = k_uptime_get_32();
			LOG_INF("adf: tc=%u err=%u | i2s: blk=%u rec=%u | %uHz",
				ast.dma_tc_count, ast.dma_restart_err,
				ist.block_count, ist.recover_count,
				now > 0 ? ist.block_count * 1000 / now : 0);
		}
#endif
	}
}
