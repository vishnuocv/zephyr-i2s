/* adf_pdm.c — STM32N6 ADF PDM microphone driver implementation */

#include "adf_pdm.h"

#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <stm32n6xx_hal.h>
#include <stm32n6xx_ll_rcc.h>

LOG_MODULE_REGISTER(adf_pdm, LOG_LEVEL_INF);

/* ---- Private state ------------------------------------------------------ */

static int32_t dma_buf[ADF_BLOCK_SAMPLES] __attribute__((aligned(32)));
static int32_t ring[ADF_RING_SIZE];

static volatile uint32_t ring_wr;
static volatile uint32_t ring_rd;

static K_SEM_DEFINE(data_ready, 0, 1);

static volatile uint32_t tc_count;
static volatile uint32_t restart_err;

static const struct device *dma_dev;
static struct dma_config    dma_cfg;
static struct dma_block_config dma_blk;
static uint32_t src_addr;

static MDF_HandleTypeDef hadf;

/* ---- Ring buffer -------------------------------------------------------- */

static void ring_push(const int32_t *src, uint32_t count)
{
	uint32_t wr = ring_wr;

	for (uint32_t i = 0; i < count; i++) {
		ring[wr] = src[i];
		wr = (wr + 1U) % ADF_RING_SIZE;
	}
	ring_wr = wr;
}

uint32_t adf_pdm_available(void)
{
	return (ring_wr - ring_rd + ADF_RING_SIZE) % ADF_RING_SIZE;
}

/* ---- DMA callback ------------------------------------------------------- */

static void dma_cb(const struct device *dev, void *user_data,
		    uint32_t channel, int status)
{
	ARG_UNUSED(user_data);

	if (status < 0) {
		restart_err++;
		return;
	}

	tc_count++;
	SCB_InvalidateDCache_by_Addr((void *)dma_buf,
				     ADF_BLOCK_SAMPLES * sizeof(int32_t));
	ring_push(dma_buf, ADF_BLOCK_SAMPLES);

	if (adf_pdm_available() >= ADF_BLOCK_SAMPLES)
		k_sem_give(&data_ready);

	if (dma_reload(dev, channel, src_addr, (uint32_t)dma_buf,
		       ADF_BLOCK_SAMPLES * sizeof(int32_t)) < 0 ||
	    dma_start(dev, channel) < 0)
		restart_err++;
}

/* ---- Init --------------------------------------------------------------- */

int adf_pdm_init(void)
{
	/* Clocks */
	__HAL_RCC_ADF1_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
#ifdef __HAL_RCC_ADF1_CLK_SLEEP_ENABLE
	__HAL_RCC_ADF1_CLK_SLEEP_ENABLE();
#endif
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	LL_RCC_SetADFClockSource(LL_RCC_ADF1_CLKSOURCE_IC7);

	uint32_t clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADF1);
	LOG_INF("ADF1 clock: %u Hz, CCK0=%.3f MHz, PCM=%u Hz",
		(unsigned)clk,
		(double)clk / ADF_CCK_DIVIDER / 1e6,
		(unsigned)(clk / ADF_CCK_DIVIDER / 64U));

	/* GPIO */
	GPIO_InitTypeDef gpio = {
		.Mode      = GPIO_MODE_AF_PP,
		.Pull      = GPIO_NOPULL,
		.Alternate = ADF_GPIO_AF,
	};

	gpio.Pin = ADF_CCK0_PIN; gpio.Speed = ADF_CCK0_SPEED;
	HAL_GPIO_Init(ADF_CCK0_PORT, &gpio);

	gpio.Pin = ADF_CCK1_PIN; gpio.Speed = ADF_CCK1_SPEED;
	HAL_GPIO_Init(ADF_CCK1_PORT, &gpio);

	gpio.Pin = ADF_SDI0_PIN; gpio.Speed = ADF_SDI0_SPEED;
	HAL_GPIO_Init(ADF_SDI0_PORT, &gpio);

	/* HAL MDF: serial interface + output clock */
	hadf.Instance = ADF1_Filter0;
	hadf.hdma     = NULL;

	hadf.Init.CommonParam.InterleavedFilters             = 0U;
	hadf.Init.CommonParam.ProcClockDivider               = 1U;
	hadf.Init.CommonParam.OutputClock.Activation         = ENABLE;
	hadf.Init.CommonParam.OutputClock.Pins               = MDF_OUTPUT_CLOCK_0;
	hadf.Init.CommonParam.OutputClock.Divider            = ADF_CCK_DIVIDER;
	hadf.Init.CommonParam.OutputClock.Trigger.Activation = DISABLE;
	hadf.Init.CommonParam.OutputClock.Trigger.Source      = MDF_CLOCK_TRIG_TRGO;
	hadf.Init.CommonParam.OutputClock.Trigger.Edge       = MDF_CLOCK_TRIG_RISING_EDGE;

	hadf.Init.SerialInterface.Activation  = ENABLE;
	hadf.Init.SerialInterface.Mode        = MDF_SITF_NORMAL_SPI_MODE;
	hadf.Init.SerialInterface.ClockSource = MDF_SITF_CCK0_SOURCE;
	hadf.Init.SerialInterface.Threshold   = 31U;
	hadf.Init.FilterBistream              = MDF_BITSTREAM0_RISING;

	if (HAL_MDF_Init(&hadf) != HAL_OK) {
		LOG_ERR("HAL_MDF_Init failed");
		return -EIO;
	}

	LOG_INF("ADF1 init OK");
	return 0;
}

/* ---- DMA setup ---------------------------------------------------------- */

static int dma_setup(void)
{
	dma_dev = DEVICE_DT_GET(DT_NODELABEL(gpdma1));
	if (!device_is_ready(dma_dev)) {
		LOG_ERR("GPDMA1 not ready");
		return -ENODEV;
	}

	MDF_Filter_TypeDef *f = (MDF_Filter_TypeDef *)hadf.Instance;
	src_addr = (uint32_t)&f->DFLTDR;

	memset(&dma_blk, 0, sizeof(dma_blk));
	dma_blk.source_address  = src_addr;
	dma_blk.dest_address    = (uint32_t)dma_buf;
	dma_blk.block_size      = ADF_BLOCK_SAMPLES * sizeof(int32_t);
	dma_blk.source_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	dma_blk.dest_addr_adj   = DMA_ADDR_ADJ_INCREMENT;

	memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.dma_slot            = GPDMA1_REQUEST_ADF1_FLT0;
	dma_cfg.channel_direction   = PERIPHERAL_TO_MEMORY;
	dma_cfg.complete_callback_en = 1;
	dma_cfg.source_data_size    = 4;
	dma_cfg.dest_data_size      = 4;
	dma_cfg.source_burst_length = 1;
	dma_cfg.dest_burst_length   = 1;
	dma_cfg.block_count         = 1;
	dma_cfg.head_block          = &dma_blk;
	dma_cfg.dma_callback        = dma_cb;

	int ret = dma_config(dma_dev, ADF_DMA_CHANNEL, &dma_cfg);
	if (ret < 0) {
		LOG_ERR("dma_config ch%d failed: %d", ADF_DMA_CHANNEL, ret);
		return ret;
	}

	LOG_INF("DMA ch%d configured (slot=%u, %u bytes/xfer)",
		ADF_DMA_CHANNEL, (unsigned)dma_cfg.dma_slot,
		(unsigned)dma_blk.block_size);
	return 0;
}

/* ---- Start -------------------------------------------------------------- */

int adf_pdm_start(void)
{
	MDF_Filter_TypeDef *f = (MDF_Filter_TypeDef *)hadf.Instance;
	int ret;

	f->DFLTCR   &= ~MDF_DFLTCR_DFLTEN;
	f->DFLTCICR  = ADF_DFLTCICR_VAL;
	f->DFLTRSFR  = ADF_DFLTRSFR_VAL;
	f->DFLTIER   = 0U;
	f->DFLTCR    = MDF_DFLTCR_DMAEN | MDF_DFLTCR_FTH;

	ret = dma_setup();
	if (ret < 0)
		return ret;

	ret = dma_start(dma_dev, ADF_DMA_CHANNEL);
	if (ret < 0) {
		LOG_ERR("dma_start failed: %d", ret);
		return ret;
	}

	SET_BIT(f->DFLTCR, MDF_DFLTCR_DFLTEN);
	hadf.State = HAL_MDF_STATE_ACQUISITION;

	LOG_INF("ADF1 DMA started (CICR=0x%08x RSFR=0x%08x CR=0x%08x)",
		(unsigned)f->DFLTCICR, (unsigned)f->DFLTRSFR,
		(unsigned)f->DFLTCR);
	return 0;
}

/* ---- Read --------------------------------------------------------------- */

int adf_pdm_read(int32_t *buf, uint32_t count, k_timeout_t timeout)
{
	while (adf_pdm_available() < count) {
		if (k_sem_take(&data_ready, timeout) != 0)
			return -EAGAIN;
	}

	uint32_t rd = ring_rd;
	for (uint32_t i = 0; i < count; i++) {
		buf[i] = ring[rd];
		rd = (rd + 1U) % ADF_RING_SIZE;
	}
	ring_rd = rd;
	return 0;
}

/* ---- Stats -------------------------------------------------------------- */

void adf_pdm_get_stats(struct adf_pdm_stats *stats)
{
	stats->dma_tc_count    = tc_count;
	stats->dma_restart_err = restart_err;
}
