/*
 * adf_pdm.h — STM32N6 ADF PDM microphone driver (Zephyr DMA)
 *
 * All definitions, register macros, data structures, and declarations
 * live here. The .c file contains only implementation.
 */

#ifndef ADF_PDM_H
#define ADF_PDM_H

#include <zephyr/kernel.h>
#include <stm32n6xx_hal_mdf.h>
#include <stdint.h>
#include <stdbool.h>

/*
 * Clock configuration
 *
 * IC7 is routed to ADF1 via LL_RCC.  CCK0 = IC7 / ADF_CCK_DIVIDER.
 * PCM sample rate = CCK0 / (ADF_DEC_RATIO * ADF_RSF_DEC_RATIO).
 */
#define ADF_CCK_DIVIDER		8

/*
 * CIC filter parameters
 *
 * These compose into the DFLTCICR register at compile time.
 * Golden value verified against HAL: DFLTCICR = 0x00200f50
 */
#define ADF_CIC_MODE		MDF_ONE_FILTER_SINC5
#define ADF_DATA_SRC		MDF_DATA_SOURCE_BSMX
#define ADF_DEC_RATIO		16	/* CIC decimation ratio, 1–256 */
#define ADF_GAIN		2	/* output scale, 0–15 */
#define ADF_DELAY		0	/* data delay in CCK cycles, 0–127 */

/* 
 * Reshape filter
 *
 * Bypass bit is inverted logic: 0 = enabled, 1 = bypassed.
 */
#define ADF_RSF_ENABLE		1
#define ADF_RSF_DEC		MDF_RSF_DECIMATION_RATIO_4

/* 
 * High-pass filter
 *
 * Bypass bit is inverted logic: 0 = enabled, 1 = bypassed.
 */
#define ADF_HPF_ENABLE		1
#define ADF_HPF_CUTOFF		MDF_HPF_CUTOFF_0_000625FPCM

/* DMA / buffer sizing  */
#define ADF_DMA_CHANNEL		7	/* GPDMA1 channel (avoid SAI TX) */
#define ADF_BLOCK_SAMPLES	960	/* samples per DMA transfer */
#define ADF_RING_BLOCKS		8	/* ring buffer depth in blocks */
#define ADF_RING_SIZE		(ADF_BLOCK_SAMPLES * ADF_RING_BLOCKS)

/*
 * GPIO pins (NUCLEO-N657X0-Q)
 *
 * PE13 = CCK0 (output clock to mic)
 * PE14 = CCK1 (unused, configured for completeness)
 * PB2  = SDI0 (PDM data input)
 */
#define ADF_CCK0_PORT		GPIOE
#define ADF_CCK0_PIN		GPIO_PIN_13
#define ADF_CCK0_SPEED		GPIO_SPEED_FREQ_MEDIUM

#define ADF_CCK1_PORT		GPIOE
#define ADF_CCK1_PIN		GPIO_PIN_14
#define ADF_CCK1_SPEED		GPIO_SPEED_FREQ_LOW

#define ADF_SDI0_PORT		GPIOB
#define ADF_SDI0_PIN		GPIO_PIN_2
#define ADF_SDI0_SPEED		GPIO_SPEED_FREQ_MEDIUM

#define ADF_GPIO_AF		GPIO_AF3_ADF1

/*
 * Register composition (compile-time)
 *
 * DFLTCICR:
 *   [3:0]   data source  (HAL pre-shifted)
 *   [6:4]   CIC mode     (HAL pre-shifted)
 *   [15:8]  dec ratio - 1
 *   [22:16] delay
 *   [23:20] gain/scale
 *
 * DFLTRSFR:
 *   bit 0   reshape bypass  (0=enabled, 1=bypassed)
 *   [6:4]   reshape dec     (HAL pre-shifted)
 *   bit 7   HPF bypass      (0=enabled, 1=bypassed)
 *   [10:8]  HPF cutoff      (HAL pre-shifted)
 */
#define ADF_DFLTCICR_VAL	(ADF_DATA_SRC | ADF_CIC_MODE | \
				 (((ADF_DEC_RATIO) - 1U) << 8) | \
				 ((uint32_t)(ADF_GAIN) << 20) | \
				 ((uint32_t)(ADF_DELAY) << 16))

#define ADF_RSF_BYP		((ADF_RSF_ENABLE) ? 0U : (1UL << 0))
#define ADF_HPF_BYP		((ADF_HPF_ENABLE) ? 0U : (1UL << 7))
#define ADF_DFLTRSFR_VAL	(ADF_RSF_BYP | ADF_RSF_DEC | \
				 ADF_HPF_BYP | ADF_HPF_CUTOFF)

/* DFLTCR bit definitions (fallbacks if CMSIS headers lack them) */
#ifndef MDF_DFLTCR_DFLTEN
#define MDF_DFLTCR_DFLTEN	(1UL << 0)
#endif
#ifndef MDF_DFLTCR_DMAEN
#define MDF_DFLTCR_DMAEN	(1UL << 1)
#endif
#ifndef MDF_DFLTCR_FTH
#define MDF_DFLTCR_FTH		(1UL << 2)
#endif

/* Driver statistics */
struct adf_pdm_stats {
	uint32_t dma_tc_count;		/* DMA transfer-complete count */
	uint32_t dma_restart_err;	/* DMA reload/start failures */
};

/*
 * adf_pdm_init - Initialise ADF1 clocks, GPIO, and serial interface.
 *
 * Enables ADF1/GPIO RCC clocks, configures PDM pins, routes IC7 to ADF1
 * clock mux, and calls HAL_MDF_Init for serial interface + output clock.
 *
 * Return: 0 on success, negative errno on failure.
 */
int adf_pdm_init(void);

/*
 * adf_pdm_start - Configure filter and start DMA acquisition.
 *
 * Writes DFLTCICR/DFLTRSFR directly (composed from defines above),
 * sets DMAEN, starts Zephyr DMA, then enables the filter.
 * Must be called after adf_pdm_init().
 *
 * Return: 0 on success, negative errno on failure.
 */
int adf_pdm_start(void);

/*
 * adf_pdm_read - Read PCM samples from the internal ring buffer.
 *
 * Blocks on a semaphore until @count samples are available or @timeout
 * expires. Samples are 32-bit signed, from the MDF filter output.
 *
 * @buf:     destination buffer (caller-allocated)
 * @count:   number of int32_t samples to read
 * @timeout: max wait (K_FOREVER, K_MSEC(n), K_NO_WAIT)
 *
 * Return: 0 on success, -EAGAIN on timeout.
 */
int adf_pdm_read(int32_t *buf, uint32_t count, k_timeout_t timeout);

/* adf_pdm_available - Return samples currently available in ring buffer */
uint32_t adf_pdm_available(void);

/* adf_pdm_get_stats - Copy current DMA statistics */
void adf_pdm_get_stats(struct adf_pdm_stats *stats);

#endif /* ADF_PDM_H */
