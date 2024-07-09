/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

/***** tmf882x_shim_linux_kernel.h *****/

#ifndef __TMF882X_SHIM_LINUX_KERNEL_H
#define __TMF882X_SHIM_LINUX_KERNEL_H

#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include "tmf882x_driver.h"

#define tof_err(p, fmt, ...) \
({ \
    struct tof_sensor_chip *__chip = (struct tof_sensor_chip*)p; \
    dev_err(tof_to_dev(__chip), fmt "\n", ##__VA_ARGS__); \
})

#define tof_info(p, fmt, ...) \
({ \
    struct tof_sensor_chip *__chip = (struct tof_sensor_chip*)p; \
    dev_info(tof_to_dev(__chip), fmt "\n", ##__VA_ARGS__); \
})

#define tof_dbg(p, fmt, ...) \
({ \
    struct tof_sensor_chip *__chip = (struct tof_sensor_chip*)p; \
    dev_dbg(tof_to_dev(__chip), fmt "\n", ##__VA_ARGS__); \
})

static inline int32_t tof_i2c_read(struct tof_sensor_chip *chip, uint8_t reg,
                                   uint8_t *buf, int32_t len)
{
    return tof_frwk_i2c_read(chip, reg, buf, len);
}

static inline int32_t tof_i2c_write(struct tof_sensor_chip *chip, uint8_t reg,
                                    const uint8_t *buf, int32_t len)
{
    return tof_frwk_i2c_write(chip, reg, buf, len);
}

static inline int32_t tof_set_register(struct tof_sensor_chip *chip, uint8_t reg,
                                       uint8_t val)
{
    return tof_frwk_i2c_write(chip, reg, &val, 1);
}

static inline int32_t tof_get_register(struct tof_sensor_chip *chip, uint8_t reg,
                                       uint8_t *val)
{
    return tof_frwk_i2c_read(chip, reg, val, 1);
}

static inline void tof_usleep(struct tof_sensor_chip *chip, uint32_t usec)
{
    usleep_range(usec, usec+10);
}

static inline int32_t tof_queue_msg(struct tof_sensor_chip *chip, struct tmf882x_msg *msg)
{
    return tof_frwk_queue_msg(chip, msg);
}

static inline void tof_get_timespec(struct timespec64 *ts)
{
    ktime_get_real_ts64(ts);
}

#endif
