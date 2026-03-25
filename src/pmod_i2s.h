/*
 * pmod_i2s.h — PMOD I2S audio output driver (Zephyr I2S/SAI)
 *
 * All definitions, data structures, and declarations live here.
 * The .c file contains only implementation.
 */

#ifndef PMOD_I2S_H
#define PMOD_I2S_H

#include <zephyr/kernel.h>
#include <stdint.h>

/* I2S audio format */
#define PMOD_SAMPLE_RATE	48000U
#define PMOD_NUM_CHANNELS	2
#define PMOD_BIT_WIDTH		16

/* 
 * Buffer sizing
 *
 * PMOD_BLOCK_SAMPLES must match ADF_BLOCK_SAMPLES when used together.
 */
#define PMOD_BLOCK_SAMPLES	960
#define PMOD_NUM_BLOCKS		16	/* I2S TX queue depth */
#define PMOD_PREFILL_BLOCKS	4	/* silence blocks before START */

/* Derived constants */
#define PMOD_BLOCK_SIZE_BYTES	(PMOD_BLOCK_SAMPLES * PMOD_NUM_CHANNELS * \
				 (PMOD_BIT_WIDTH / 8))

/* 
 * Mono-to-stereo conversion
 *
 * Input: 32-bit signed PCM from MDF filter
 * Output: 16-bit signed stereo (L=R)
 *
 * PMOD_SHIFT_BITS: right-shift applied before clamping to int16 range.
 * Adjust to match your MDF filter gain setting.
 */
#define PMOD_SHIFT_BITS		10

/* 
 * Device tree alias
 *
 * The overlay must define: aliases { sai-dac = &sai2_a; };
 */
#define PMOD_I2S_DT_ALIAS	sai_dac

/* Driver statistics */
struct pmod_i2s_stats {
	uint32_t block_count;		/* blocks successfully written */
	uint32_t recover_count;		/* TX underrun recoveries */
};

/*
 * pmod_i2s_init - Configure I2S and prefill TX queue with silence.
 *
 * Looks up the I2S device from device tree (PMOD_I2S_DT_ALIAS),
 * configures it with the format defined above, and queues
 * PMOD_PREFILL_BLOCKS of silence so TX can start immediately.
 *
 * Return: 0 on success, negative errno on failure.
 */
int pmod_i2s_init(void);

/*
 * pmod_i2s_start - Start I2S TX stream.
 *
 * Issues I2S_TRIGGER_START.  Must be called after pmod_i2s_init().
 *
 * Return: 0 on success, negative errno on failure.
 */
int pmod_i2s_start(void);

/*
 * pmod_i2s_write_mono32 - Write mono 32-bit samples as stereo 16-bit I2S.
 *
 * For each sample: right-shift by PMOD_SHIFT_BITS, clamp to int16 range,
 * duplicate to L+R channels, write to I2S TX queue.
 * Auto-recovers on TX underrun (DROP → refill → START).
 *
 * @samples: mono 32-bit PCM input buffer
 * @count:   number of samples (should be PMOD_BLOCK_SAMPLES)
 *
 * Return: 0 on success, negative errno on failure.
 */
int pmod_i2s_write_mono32(const int32_t *samples, uint32_t count);

/* pmod_i2s_get_stats - Copy current driver statistics  */
void pmod_i2s_get_stats(struct pmod_i2s_stats *stats);

#endif /* PMOD_I2S_H */
