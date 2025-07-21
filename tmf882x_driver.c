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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio/consumer.h>
#include <linux/kfifo.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/eventpoll.h>
#include <linux/version.h>
#ifdef CONFIG_TMF882X_QCOM_AP
#include <linux/sensors.h>
#endif

#include "tmf882x_driver.h"
#include "tmf882x_interface.h"

#define TMF882X_NAME                "tmf882x"
#define TOF_GPIO_INT_NAME           "irq"
#define TOF_GPIO_ENABLE_NAME        "enable"
#define TOF_PROP_NAME_POLLIO        "poll_period"
#define TMF882X_DEFAULT_INTERVAL_MS 10

#define AMS_MUTEX_LOCK(m) { \
    mutex_lock(m); \
}
#define AMS_MUTEX_TRYLOCK(m) ({ \
    mutex_trylock(m); \
})
#define AMS_MUTEX_UNLOCK(m) { \
    mutex_unlock(m); \
}

#define TMF_DEFAULT_I2C_ADDR 0x41

struct tmf882x_platform_data {
    const char *tof_name;
    struct gpio_desc *gpiod_interrupt;
    struct gpio_desc *gpiod_enable;
    const char *fac_calib_data_fname;
    const char *config_calib_data_fname;
    const char *ram_patch_fname[];
};

struct tof_sensor_chip {

    bool driver_remove;
    bool fwdl_needed;
    int poll_period;
    int open_refcnt;
    int driver_debug;

    /* Linux kernel structure(s) */
    DECLARE_KFIFO(fifo_out, u8, 4*PAGE_SIZE);
    struct mutex lock;
    struct miscdevice tof_mdev;
    struct input_dev *tof_idev;
    struct completion ram_patch_in_progress;
    struct firmware *tof_fw;
    struct tmf882x_platform_data *pdata;
    struct i2c_client *client;
    struct task_struct *poll_irq;
    wait_queue_head_t fifo_wait;

#ifdef CONFIG_TMF882X_QCOM_AP
    // Qualcomm linux kernel AP structures
    struct sensors_classdev cdev;
#endif

    /* ToF structure(s) */
    struct tmf882x_tof  tof;
    struct tmf882x_mode_app_config  tof_cfg;
    struct tmf882x_mode_app_spad_config  tof_spad_cfg;
    struct tmf882x_mode_app_calib tof_calib;
    bool tof_spad_uncommitted;
    bool resume_measurements;
};

static struct tmf882x_platform_data tof_pdata = {
    .tof_name = TMF882X_NAME,
    .fac_calib_data_fname = "tmf882x_fac_calib.bin",
    .config_calib_data_fname = "tmf882x_config_calib.bin",
    .ram_patch_fname = {
        "tmf882x_firmware.bin",
        NULL,
    },
};

#ifdef CONFIG_TMF882X_QCOM_AP
static struct sensors_classdev sensors_cdev = {
    .name = TMF882X_NAME,
    .vendor = "ams",
    .version = 1,
    .handle = SENSORS_PROXIMITY_HANDLE,
    .type = SENSOR_TYPE_PROXIMITY,
    .max_range = "5",
    .resolution = "0.001",
    .sensor_power = "40",
    .min_delay = 0,
    .max_delay = USHRT_MAX,
    .fifo_reserved_event_count = 0,
    .fifo_max_event_count = 0,
    .enabled = 0,
    .delay_msec = TMF882X_DEFAULT_INTERVAL_MS,
    .sensors_enable = NULL,
    .sensors_poll_delay = NULL,
};
#endif

inline struct device * tof_to_dev(struct tof_sensor_chip *chip)
{
    return &chip->client->dev;
}
/*
 *
 * Function Declarations
 *
 */
static void tof_ram_patch_callback(const struct firmware *cfg, void *ctx);
static irqreturn_t tof_irq_handler(int irq, void *dev_id);
static int tof_hard_reset(struct tof_sensor_chip *chip);
static int tof_frwk_i2c_write_mask(struct tof_sensor_chip *chip, char reg,
                                   const char *val, char mask);
static int tof_poweroff_device(struct tof_sensor_chip *chip);
static int tof_poweron_device(struct tof_sensor_chip *chip);
static int tof_open_mode(struct tof_sensor_chip *chip, uint32_t req_mode);

static size_t tof_fifo_next_msg_size(struct tof_sensor_chip *chip)
{
    struct tmf882x_msg_header hdr;
    int ret;
    if (kfifo_is_empty(&chip->fifo_out))
        return 0;
    ret = kfifo_out_peek(&chip->fifo_out, (char *)&hdr, sizeof(hdr));
    if (ret != sizeof(hdr))
        return 0;
    return hdr.msg_len;
}

static void tof_publish_input_events(struct tof_sensor_chip *chip,
                                     struct tmf882x_msg *msg)
{
    int val = 0;
    uint32_t code = 0;
    uint32_t i = 0;
    struct tmf882x_msg_meas_results *res;
    switch(msg->hdr.msg_id) {
        case ID_MEAS_RESULTS:
            /* Input Event encoding for measure results :
             *     code       => channel
             *     val(31:24) => sub capture index
             *     val(23:16) => confidence
             *     val(15:0)  => distance in mm
             */
            res = &msg->meas_result_msg;
            for (i = 0; i < res->num_results; i++) {
                val = 0;
                code = res->results[i].channel;
                val |= (res->results[i].sub_capture & 0xFF) << 24;
                val |= (res->results[i].confidence & 0xFF) << 16;
                val |= res->results[i].distance_mm & 0xFFFF;
                input_report_abs(chip->tof_idev, code, val);
            }
            break;
        default:
            /* Don't publish any input events if not explicitly handled */
            return;
    }
    input_sync(chip->tof_idev);
}

static int tof_firmware_download(struct tof_sensor_chip *chip)
{
    /*** ASSUME MUTEX IS ALREADY HELD ***/
    int error;
    int file_idx = 0;
    const struct firmware *fw = NULL;

    /* Iterate through all Firmware(s) to find one that works
     */
    for (file_idx=0;
         chip->pdata->ram_patch_fname[file_idx] != NULL;
         file_idx++) {

        /*** reset completion event that FWDL is starting ***/
        reinit_completion(&chip->ram_patch_in_progress);

        dev_info(&chip->client->dev, "Trying firmware: \'%s\'...\n",
                chip->pdata->ram_patch_fname[file_idx]);

        /***** Check for available firmware to load *****/
        error = request_firmware_direct(&fw,
                                        chip->pdata->ram_patch_fname[file_idx],
                                        &chip->client->dev);
        if (error) {
            dev_warn(&chip->client->dev,
                     "Firmware not available \'%s\': %d\n",
                     chip->pdata->ram_patch_fname[file_idx], error);
            continue;
        } else {
            tof_ram_patch_callback(fw, chip);
        }

        if (!wait_for_completion_interruptible_timeout(&chip->ram_patch_in_progress,
                                                       msecs_to_jiffies(TOF_FWDL_TIMEOUT_MSEC))) {
            dev_err(&chip->client->dev,
                    "Timeout waiting for Ram Patch \'%s\' Complete",
                    chip->pdata->ram_patch_fname[file_idx]);
            error = -EIO;
            continue;
        }

        // assume everything was successful
        error = 0;
    }
    return error;
}

static int tof_poweron_device(struct tof_sensor_chip *chip)
{
    int error = 0;
    if (chip->pdata->gpiod_enable &&
        !gpiod_get_value_cansleep(chip->pdata->gpiod_enable)) {
        error = gpiod_direction_output(chip->pdata->gpiod_enable, 1);
        if (error) {
            dev_err(&chip->client->dev,
                    "Error powering chip: %d\n", error);
            return error;
        }
    }

    return 0;
}

static int tof_open_mode(struct tof_sensor_chip *chip, uint32_t mode)
{
    tmf882x_mode_t req_mode = (tmf882x_mode_t) mode;

    if (tof_poweron_device(chip))
        return -1;

    dev_info(&chip->client->dev, "%s: %#x\n", __func__, req_mode);

    // open core driver
    if (tmf882x_open(&chip->tof)) {
        dev_err(&chip->client->dev, "failed to open TMF882X core driver\n");
        return -1;
    }

    if (req_mode == TMF882X_MODE_BOOTLOADER) {

        // mode switch to the bootloader (no-op if already in bootloader)
        if (tmf882x_mode_switch(&chip->tof, TMF882X_MODE_BOOTLOADER)) {
            dev_info(&chip->client->dev, "%s mode switch failed\n", __func__);
            tmf882x_dump_i2c_regs(tmf882x_mode_hndl(&chip->tof));
            return -1;
        }
        // we lose the FW if switching to the bootloader
        chip->fwdl_needed = true;

    } else if (req_mode == TMF882X_MODE_APP) {

        // Try FWDL - this will perform no action if poweroff has not occurred
        //  result state is that of the FW if successful, or the bootloader if not
        if (chip->fwdl_needed) {
            if (0 == tof_firmware_download(chip))
                // FWDL is no longer necessary unless device loses power
                chip->fwdl_needed = false;
        }

        // NO-OP If already in APP, else mode switch to the APP by loading from
        //  ROM/FLASH/etc
        if (tmf882x_mode_switch(&chip->tof, req_mode)) {
            tmf882x_dump_i2c_regs(tmf882x_mode_hndl(&chip->tof));
            return -1;
        }

    }

    // if we have gotten here then one of FWDL/ROM/FLASH load was successful
    return !(tmf882x_get_mode(&chip->tof) == req_mode);
}

static int tof_set_default_config(struct tof_sensor_chip *chip)
{
    int error;

    // use current debug setting
    tmf882x_set_debug(&chip->tof, !!(chip->driver_debug));

    // retrieve current config
    error = tmf882x_ioctl(&chip->tof, IOCAPP_GET_CFG, NULL, &chip->tof_cfg);
    if (error) {
        dev_err(&chip->client->dev, "Error, app get config failed.\n");
        return error;
    }

    /////////////////////////////////////
    //
    //  Set default config (if any)
    //
    ////////////////////////////////////

#if (CONFIG_TMF882X_SPAD_CFG())
    // retrieve current spad config
    memset(&chip->tof_spad_cfg, 0, sizeof(chip->tof_spad_cfg));
    error = tmf882x_ioctl(&chip->tof, IOCAPP_GET_SPADCFG, NULL, &chip->tof_spad_cfg);
    if (error) {
        dev_err(&chip->client->dev, "Error, app get spad config failed.\n");
        return error;
    }
#endif

    return 0;
}

static ssize_t mode_show(struct device * dev,
                         struct device_attribute * attr,
                         char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    dev_info(dev, "%s\n", __func__);
    return scnprintf(buf, PAGE_SIZE, "0x%hhx\n",
                     tmf882x_mode(tmf882x_mode_hndl(&chip->tof)));
}

static ssize_t mode_store(struct device * dev,
                          struct device_attribute * attr,
                          const char * buf,
                          size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int req_mode;
    int error;
    dev_info(dev, "%s\n", __func__);
    sscanf(buf, "%i", &req_mode);
    AMS_MUTEX_LOCK(&chip->lock);
    error = tof_open_mode(chip, req_mode);
    if (error) {
        dev_err(&chip->client->dev, "Error opening requested mode: %#hhx", req_mode);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    if (req_mode == TMF882X_MODE_APP) {
        // if APP mode re-sync our default config
        error = tof_set_default_config(chip);
        if (error) {
            dev_err(&chip->client->dev, "Error, set default config failed.\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t chip_enable_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int state;
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (!chip->pdata->gpiod_enable) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    state = !gpiod_get_value_cansleep(chip->pdata->gpiod_enable);
    dev_info(dev, "%s: %u\n", __func__, state);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%d\n", !!state);
}

static ssize_t chip_enable_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buf,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int req_state;
    int error;
    dev_info(dev, "%s\n", __func__);
    error = sscanf(buf, "%i", &req_state);
    if (error != 1)
        return -1;
    AMS_MUTEX_LOCK(&chip->lock);
    if (!chip->pdata->gpiod_enable) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    if (req_state == 0) {
        tof_poweroff_device(chip);
    } else {
        error = tof_open_mode(chip, TMF882X_MODE_APP);
        if (error) {
            dev_err(&chip->client->dev, "Error powering-on device");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t driver_debug_show(struct device * dev,
                                 struct device_attribute * attr,
                                 char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    dev_info(dev, "%s\n", __func__);
    return scnprintf(buf, PAGE_SIZE, "%d\n", chip->driver_debug);
}

static ssize_t driver_debug_store(struct device * dev,
                                  struct device_attribute * attr,
                                  const char * buf,
                                  size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int debug;
    dev_info(dev, "%s\n", __func__);
    sscanf(buf, "%i", &debug);
    if (debug == 0) {
        chip->driver_debug = 0;
    } else {
        chip->driver_debug = debug;
    }
    tmf882x_set_debug(&chip->tof, !!(chip->driver_debug));
    return count;
}

static ssize_t firmware_version_show(struct device * dev,
                                     struct device_attribute * attr,
                                     char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int len = 0;
    char str[17] = {0};
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    len = tmf882x_get_firmware_ver(&chip->tof, str, sizeof(str));
    if (len < 0) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    len = scnprintf(buf, PAGE_SIZE, "%u.%u.%u.%u\n", str[0], str[1], str[2], str[3]);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t device_uid_show(struct device * dev,
                               struct device_attribute * attr,
                               char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int len = 0;
    struct tmf882x_mode_app_dev_UID uid;
    int error;
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);

    //UID is only available through the application so we must open it
    error = tof_open_mode(chip, TMF882X_MODE_APP);
    if (error) {
        dev_err(&chip->client->dev, "Error powering-on device");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    memset(&uid, 0, sizeof(uid));
    error = tmf882x_ioctl(&chip->tof, IOCAPP_DEV_UID, NULL, &uid);
    if (error || (uid.len == 0)) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    if (!chip->open_refcnt) {
        tmf882x_close(&chip->tof);
    }
    len = scnprintf(buf, PAGE_SIZE, "%u.%u.%u.%u\n",
                    uid.uid[0], uid.uid[1], uid.uid[2], uid.uid[3]);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t device_revision_show(struct device * dev,
                                    struct device_attribute * attr,
                                    char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int len = 0;
    char revision[17] = {0};
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    if (tof_poweron_device(chip)) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    len = tmf882x_get_device_revision(&chip->tof, revision, sizeof(revision));
    if (len < 0) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    len = scnprintf(buf, PAGE_SIZE, "%u.%u\n", revision[0], revision[1]);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t registers_show(struct device * dev,
                              struct device_attribute * attr,
                              char * buf)
{
    int per_line = 4;
    int len = 0;
    int idx, per_line_idx;
    int bufsize = PAGE_SIZE;
    int error;
    char regs[MAX_REGS] = {0};
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);

    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    error = tof_frwk_i2c_read(chip, 0x00, regs, MAX_REGS);
    if (error < 0) {
        dev_err(&chip->client->dev, "Read all registers failed: %d\n", error);
        return error;
    }
    if (error) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return error;
    }

    for (idx = 0; idx < MAX_REGS; idx += per_line) {
        len += scnprintf(buf + len, bufsize - len, "0x%02x:", idx);
        for (per_line_idx = 0; per_line_idx < per_line; per_line_idx++) {
            len += scnprintf(buf + len, bufsize - len, " ");
            len += scnprintf(buf + len, bufsize - len, "%02x", regs[idx+per_line_idx]);
        }
        len += scnprintf(buf + len, bufsize - len, "\n");
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return len;
}

static ssize_t register_write_store(struct device * dev,
                                    struct device_attribute * attr,
                                    const char * buf,
                                    size_t count)
{
    char preg;
    char pval;
    char pmask = -1;
    int numparams;
    int rc;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    dev_info(dev, "%s\n", __func__);

    numparams = sscanf(buf, "%hhi:%hhi:%hhi", &preg, &pval, &pmask);
    if ((numparams < 2) || (numparams > 3))
        return -EINVAL;
    if ((numparams >= 1) && (preg < 0))
        return -EINVAL;
    if ((numparams >= 2) && (preg < 0 || preg > 0xff))
        return -EINVAL;
    if ((numparams >= 3) && (pmask < 0 || pmask > 0xff))
        return -EINVAL;

    if (pmask == -1) {
        rc = tof_frwk_i2c_write(chip, preg, &pval, 1);
    } else {
        rc = tof_frwk_i2c_write_mask(chip, preg, &pval, pmask);
    }

    return rc ? rc : count;
}

static ssize_t request_ram_patch_store(struct device * dev,
                                       struct device_attribute * attr,
                                       const char * buf,
                                       size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int error = 0;
    dev_info(dev, "%s\n", __func__);
    AMS_MUTEX_LOCK(&chip->lock);
    /***** Try to re-open the app (perform fwdl if available) *****/
    error = tof_hard_reset(chip);
    if (error) {
        dev_err(dev, "Error re-patching device\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    // We always end up in APP mode after fwdl, so re-sync our config
    error = tof_set_default_config(chip);
    if (error) {
        dev_err(dev, "Error, set default config failed.\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t capture_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool meas = false;

    dev_info(dev, "%s\n", __func__);

    (void) tmf882x_ioctl(&chip->tof, IOCAPP_IS_MEAS, NULL, &meas);
    return scnprintf(buf, PAGE_SIZE, "%u\n", meas);
}

static ssize_t capture_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int capture;

    sscanf(buf, "%i", &capture);
    AMS_MUTEX_LOCK(&chip->lock);
    if (capture) {
        dev_info(dev, "%s: start capture\n", __func__);
        if (tof_open_mode(chip, TMF882X_MODE_APP)) {
            dev_err(dev, "Chip power-on failed\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        // start measurements
        if (tmf882x_start(&chip->tof)) {
            dev_info(dev, "Error starting measurements\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
    } else {
        dev_info(dev, "%s: stop capture\n", __func__);
        // stop measurements
        if (tmf882x_stop(&chip->tof)) {
            dev_info(dev, "Error stopping measurements\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        // stopping measurements, lets flush the ring buffer
        kfifo_reset(&chip->fifo_out);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t short_range_mode_show(struct device * dev,
                                     struct device_attribute * attr,
                                     char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool shortrange = false;
    int rc;

    dev_info(dev, "%s\n", __func__);

    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_IS_SHORTRANGE, NULL, &shortrange);
    if (rc) {
        dev_info(dev, "Error reading short_range mode\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%u\n", shortrange);
}

static ssize_t short_range_mode_store(struct device * dev,
                                      struct device_attribute * attr,
                                      const char * buf,
                                      size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool is_shortrange = false;
    uint32_t read_shortrange = 0;
    int rc;

    sscanf(buf, "%i", &read_shortrange);
    is_shortrange = (bool)read_shortrange;
    dev_info(dev, "%s: %u\n", __func__, is_shortrange);
    AMS_MUTEX_LOCK(&chip->lock);

    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_SHORTRANGE, &is_shortrange, NULL);
    if (rc) {
        dev_info(dev, "Error writing shortrange mode\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }

    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t report_period_ms_show(struct device * dev,
                                     struct device_attribute * attr,
                                     char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n", chip->tof_cfg.report_period_ms);
}

static ssize_t report_period_ms_store(struct device * dev,
                                      struct device_attribute * attr,
                                      const char * buf,
                                      size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t period_ms;
    int rc;

    sscanf(buf, "%i", &period_ms);
    dev_info(dev, "%s: %u ms\n", __func__, period_ms);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.report_period_ms = period_ms;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring reporting period\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t iterations_show(struct device * dev,
                               struct device_attribute * attr,
                               char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n",
                     chip->tof_cfg.kilo_iterations << 10);
}

static ssize_t iterations_store(struct device * dev,
                                struct device_attribute * attr,
                                const char * buf,
                                size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t iterations;
    int rc;

    sscanf(buf, "%i", &iterations);
    dev_info(dev, "%s: %u\n", __func__, iterations);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.kilo_iterations = iterations >> 10;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring iterations\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t alg_setting_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#x\n",
                     chip->tof_cfg.alg_setting);
}

static ssize_t alg_setting_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buf,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t alg_mask;
    int rc;

    sscanf(buf, "%i", &alg_mask);
    dev_info(dev, "%s: %#x\n", __func__, alg_mask);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.alg_setting = alg_mask;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring alg setting\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t power_cfg_show(struct device * dev,
                              struct device_attribute * attr,
                              char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#x\n",
                     chip->tof_cfg.power_cfg);
}

static ssize_t power_cfg_store(struct device * dev,
                               struct device_attribute * attr,
                               const char * buf,
                               size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t power_cfg;
    int rc;

    sscanf(buf, "%i", &power_cfg);
    dev_info(dev, "%s: %#x\n", __func__, power_cfg);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.power_cfg = power_cfg;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring power config\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t gpio_0_show(struct device * dev,
                           struct device_attribute * attr,
                           char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#hhx\n",
                     chip->tof_cfg.gpio_0);
}

static ssize_t gpio_0_store(struct device * dev,
                            struct device_attribute * attr,
                            const char * buf,
                            size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t gpio_mask;
    int rc;

    sscanf(buf, "%hhi", &gpio_mask);
    dev_info(dev, "%s: %#x\n", __func__, gpio_mask);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.gpio_0 = gpio_mask;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring gpio_0\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t gpio_1_show(struct device * dev,
                           struct device_attribute * attr,
                           char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#hhx\n",
                     chip->tof_cfg.gpio_1);
}

static ssize_t gpio_1_store(struct device * dev,
                            struct device_attribute * attr,
                            const char * buf,
                            size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t gpio_mask;
    int rc;

    sscanf(buf, "%hhi", &gpio_mask);
    dev_info(dev, "%s: %#x\n", __func__, gpio_mask);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.gpio_1 = gpio_mask;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring gpio_1\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t histogram_dump_show(struct device * dev,
                                   struct device_attribute * attr,
                                   char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#hhx\n",
                     chip->tof_cfg.histogram_dump);
}

static ssize_t histogram_dump_store(struct device * dev,
                                    struct device_attribute * attr,
                                    const char * buf,
                                    size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t hist_mask;
    int rc;

    sscanf(buf, "%hhi", &hist_mask);
    dev_info(dev, "%s: %#x\n", __func__, hist_mask);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.histogram_dump = hist_mask;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring histogram dump mask\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t spad_map_id_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#hhx\n",
                     chip->tof_cfg.spad_map_id);
}

static ssize_t spad_map_id_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buf,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t map_id;
    int rc;

    sscanf(buf, "%hhi", &map_id);
    dev_info(dev, "%s: %#x\n", __func__, map_id);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.spad_map_id = map_id;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring spad_map_id\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    // read out fresh spad configuration from device, overwrite local copy
    chip->tof_spad_uncommitted = false;
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t zone_mask_show(struct device * dev,
                              struct device_attribute * attr,
                              char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%#x\n",
                     chip->tof_cfg.zone_mask);
}

static ssize_t zone_mask_store(struct device * dev,
                               struct device_attribute * attr,
                               const char * buf,
                               size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t mask;
    int rc;

    sscanf(buf, "%i", &mask);
    dev_info(dev, "%s: %#x\n", __func__, mask);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.zone_mask = mask;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring zone_mask\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t conf_threshold_show(struct device * dev,
                                  struct device_attribute * attr,
                                  char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%hhu\n",
                     chip->tof_cfg.confidence_threshold);
}

static ssize_t conf_threshold_store(struct device * dev,
                                   struct device_attribute * attr,
                                   const char * buf,
                                   size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t th;
    int rc;

    sscanf(buf, "%hhi", &th);
    dev_info(dev, "%s: %hhu\n", __func__, th);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.confidence_threshold = th;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring conf threshold\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t low_threshold_show(struct device * dev,
                                  struct device_attribute * attr,
                                  char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%hu\n",
                     chip->tof_cfg.low_threshold);
}

static ssize_t low_threshold_store(struct device * dev,
                                   struct device_attribute * attr,
                                   const char * buf,
                                   size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint16_t th;
    int rc;

    sscanf(buf, "%hi", &th);
    dev_info(dev, "%s: %hu\n", __func__, th);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.low_threshold = th;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring low threshold\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t high_threshold_show(struct device * dev,
                                   struct device_attribute * attr,
                                   char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%hu\n",
                     chip->tof_cfg.high_threshold);
}

static ssize_t high_threshold_store(struct device * dev,
                                    struct device_attribute * attr,
                                    const char * buf,
                                    size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint16_t th;
    int rc;

    sscanf(buf, "%hi", &th);
    dev_info(dev, "%s: %hu\n", __func__, th);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.high_threshold = th;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring high threshold\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t persistence_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%hhu\n",
                     chip->tof_cfg.persistence);
}

static ssize_t persistence_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buf,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t per;
    int rc;

    sscanf(buf, "%hhi", &per);
    dev_info(dev, "%s: %hhu\n", __func__, per);
    AMS_MUTEX_LOCK(&chip->lock);

    chip->tof_cfg.persistence = per;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_info(dev, "Error configuring persistence\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t mode_8x8_show(struct device * dev,
                             struct device_attribute * attr,
                             char * buf)
{
    bool is_8x8 = false;
    int rc;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_IS_8X8MODE, NULL, &is_8x8);
    if (rc) {
        dev_info(dev, "Error reading 8x8 mode\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%u\n", is_8x8);
}

static ssize_t mode_8x8_store(struct device * dev,
                              struct device_attribute * attr,
                              const char * buf,
                              size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool is_8x8 = false;
    uint32_t read_8x8 = 0;
    int rc;

    sscanf(buf, "%i", &read_8x8);
    is_8x8 = (bool)read_8x8;
    dev_info(dev, "%s: %u\n", __func__, is_8x8);
    AMS_MUTEX_LOCK(&chip->lock);

    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_8X8MODE, &is_8x8, NULL);
    if (rc) {
        dev_info(dev, "Error writing 8x8 mode\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }

    if (is_8x8) {
        // SPAD Map ID is set to custom time-multiplexed mode for 8x8 mode
        chip->tof_cfg.spad_map_id = TMF8X2X_COM_SPAD_MAP_ID__spad_map_id__user_defined_2;
    }
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t xoff_q1_0_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     chip->tof_spad_cfg.spad_configs[0].xoff_q1);
}

static ssize_t xoff_q1_0_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int8_t xoff;

    sscanf(buf, "%hhi", &xoff);
    dev_info(dev, "%s: %d\n", __func__, xoff);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[0].xoff_q1 = xoff;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t xoff_q1_1_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     chip->tof_spad_cfg.spad_configs[1].xoff_q1);
}

static ssize_t xoff_q1_1_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int8_t xoff;

    sscanf(buf, "%hhi", &xoff);
    dev_info(dev, "%s: %d\n", __func__, xoff);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[1].xoff_q1 = xoff;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t yoff_q1_0_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     chip->tof_spad_cfg.spad_configs[0].yoff_q1);
}

static ssize_t yoff_q1_0_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int8_t yoff;

    sscanf(buf, "%hhi", &yoff);
    dev_info(dev, "%s: %d\n", __func__, yoff);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[0].yoff_q1 = yoff;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t yoff_q1_1_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%d\n",
                     chip->tof_spad_cfg.spad_configs[1].yoff_q1);
}

static ssize_t yoff_q1_1_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int8_t yoff;

    sscanf(buf, "%hhi", &yoff);
    dev_info(dev, "%s: %d\n", __func__, yoff);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[1].yoff_q1 = yoff;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t xsize_0_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n",
                     chip->tof_spad_cfg.spad_configs[0].xsize);
}

static ssize_t xsize_0_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t xsize;

    sscanf(buf, "%hhi", &xsize);
    dev_info(dev, "%s: %u\n", __func__, xsize);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[0].xsize = xsize;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t xsize_1_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n",
                     chip->tof_spad_cfg.spad_configs[1].xsize);
}

static ssize_t xsize_1_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t xsize;

    sscanf(buf, "%hhi", &xsize);
    dev_info(dev, "%s: %u\n", __func__, xsize);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[1].xsize = xsize;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t ysize_0_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n",
                     chip->tof_spad_cfg.spad_configs[0].ysize);
}

static ssize_t ysize_0_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t ysize;

    sscanf(buf, "%hhi", &ysize);
    dev_info(dev, "%s: %u\n", __func__, ysize);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[0].ysize = ysize;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t ysize_1_show(struct device * dev,
                            struct device_attribute * attr,
                            char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "%u\n",
                     chip->tof_spad_cfg.spad_configs[1].ysize);
}

static ssize_t ysize_1_store(struct device * dev,
                             struct device_attribute * attr,
                             const char * buf,
                             size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t ysize;

    sscanf(buf, "%hhi", &ysize);
    dev_info(dev, "%s: %u\n", __func__, ysize);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->tof_spad_cfg.spad_configs[1].ysize = ysize;
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t spad_mask_0_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t i, j, idx;
    uint32_t len = 0;
    uint32_t xlen, ylen;
    xlen = chip->tof_spad_cfg.spad_configs[0].xsize;
    ylen = chip->tof_spad_cfg.spad_configs[0].ysize;
    for (i = 0; i < ylen; ++i) {
        for (j = 0; j < xlen; ++j) {
            idx = i*xlen + j;
            len += scnprintf(buf + len, PAGE_SIZE - len,
                             "%u ", chip->tof_spad_cfg.spad_configs[0].spad_mask[idx]);
        }
        len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
    }
    return len;
}

static ssize_t spad_mask_0_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buffer,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t mask[TMF8X2X_COM_MAX_SPAD_SIZE] = {0};
    uint32_t i;
    int rc;
    char *buf = kstrndup(buffer, count, GFP_KERNEL);
    char *save_str = buf;
    char *tok = strsep((char **)&buf, " \n\r\t");

    for (i = 0; i < ARRAY_SIZE(mask) && tok; ++i) {
        while (tok && !(rc = sscanf(tok, "%hhi", &mask[i])))
            tok = strsep(&buf, " \n\r\t");
        if (rc < 0) {
            kfree(save_str);
            return -EIO;
        }
        tok = strsep(&buf, " \n\r\t");
    }
    AMS_MUTEX_LOCK(&chip->lock);
    memset(chip->tof_spad_cfg.spad_configs[0].spad_mask, 0,
           sizeof(chip->tof_spad_cfg.spad_configs[0].spad_mask));
    memcpy(chip->tof_spad_cfg.spad_configs[0].spad_mask, mask,
           sizeof(chip->tof_spad_cfg.spad_configs[0].spad_mask));
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    kfree(save_str);
    return count;
}

static ssize_t spad_mask_1_show(struct device * dev,
                                struct device_attribute * attr,
                                char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t i, j, idx;
    uint32_t len = 0;
    uint32_t xlen, ylen;
    xlen = chip->tof_spad_cfg.spad_configs[1].xsize;
    ylen = chip->tof_spad_cfg.spad_configs[1].ysize;
    for (i = 0; i < ylen; ++i) {
        for (j = 0; j < xlen; ++j) {
            idx = i*xlen + j;
            len += scnprintf(buf + len, PAGE_SIZE - len,
                             "%u ", chip->tof_spad_cfg.spad_configs[1].spad_mask[idx]);
        }
        len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
    }
    return len;
}

static ssize_t spad_mask_1_store(struct device * dev,
                                 struct device_attribute * attr,
                                 const char * buffer,
                                 size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t mask[TMF8X2X_COM_MAX_SPAD_SIZE] = {0};
    uint32_t i;
    int rc;
    char *buf = kstrndup(buffer, count, GFP_KERNEL);
    char *save_str = buf;
    char *tok = strsep((char **)&buf, " \n\r\t");

    for (i = 0; i < ARRAY_SIZE(mask) && tok; ++i) {
        while (tok && !(rc = sscanf(tok, "%hhi", &mask[i])))
            tok = strsep(&buf, " \n\r\t");
        if (rc < 0) {
            kfree(save_str);
            return -EIO;
        }
        tok = strsep(&buf, " \n\r\t");
    }
    AMS_MUTEX_LOCK(&chip->lock);
    memset(chip->tof_spad_cfg.spad_configs[1].spad_mask, 0,
           sizeof(chip->tof_spad_cfg.spad_configs[1].spad_mask));
    memcpy(chip->tof_spad_cfg.spad_configs[1].spad_mask, mask,
           sizeof(chip->tof_spad_cfg.spad_configs[1].spad_mask));
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    kfree(save_str);
    return count;
}

static ssize_t spad_map_0_show(struct device * dev,
                               struct device_attribute * attr,
                               char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t i, j, idx;
    uint32_t len = 0;
    uint32_t xlen, ylen;
    xlen = chip->tof_spad_cfg.spad_configs[0].xsize;
    ylen = chip->tof_spad_cfg.spad_configs[0].ysize;
    for (i = 0; i < ylen; ++i) {
        for (j = 0; j < xlen; ++j) {
            idx = i*xlen + j;
            len += scnprintf(buf + len, PAGE_SIZE - len,
                             "%u ", chip->tof_spad_cfg.spad_configs[0].spad_map[idx]);
        }
        len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
    }
    return len;
}

static ssize_t spad_map_0_store(struct device * dev,
                                struct device_attribute * attr,
                                const char * buffer,
                                size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t map[TMF8X2X_COM_MAX_SPAD_SIZE] = {0};
    uint32_t i;
    int rc;
    char *buf = kstrndup(buffer, count, GFP_KERNEL);
    char *save_str = buf;
    char *tok = strsep((char **)&buf, " \n\r\t");

    for (i = 0; i < ARRAY_SIZE(map) && tok; ++i) {
        while (tok && !(rc = sscanf(tok, "%hhi", &map[i])))
            tok = strsep(&buf, " \n\r\t");
        if (rc < 0) {
            kfree(save_str);
            return -EIO;
        }
        tok = strsep(&buf, " \n\r\t");
    }
    AMS_MUTEX_LOCK(&chip->lock);
    memset(chip->tof_spad_cfg.spad_configs[0].spad_map, 0,
           sizeof(chip->tof_spad_cfg.spad_configs[0].spad_map));
    memcpy(chip->tof_spad_cfg.spad_configs[0].spad_map, map,
           sizeof(chip->tof_spad_cfg.spad_configs[0].spad_map));
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    kfree(save_str);
    return count;
}

static ssize_t spad_map_1_show(struct device * dev,
                               struct device_attribute * attr,
                               char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t i, j, idx;
    uint32_t len = 0;
    uint32_t xlen, ylen;
    xlen = chip->tof_spad_cfg.spad_configs[1].xsize;
    ylen = chip->tof_spad_cfg.spad_configs[1].ysize;
    for (i = 0; i < ylen; ++i) {
        for (j = 0; j < xlen; ++j) {
            idx = i*xlen + j;
            len += scnprintf(buf + len, PAGE_SIZE - len,
                             "%u ", chip->tof_spad_cfg.spad_configs[1].spad_map[idx]);
        }
        len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
    }
    return len;
}

static ssize_t spad_map_1_store(struct device * dev,
                                struct device_attribute * attr,
                                const char * buffer,
                                size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint8_t map[TMF8X2X_COM_MAX_SPAD_SIZE] = {0};
    uint32_t i;
    int rc;
    char *buf = kstrndup(buffer, count, GFP_KERNEL);
    char *save_str = buf;
    char *tok = strsep((char **)&buf, " \n\r\t");

    for (i = 0; i < ARRAY_SIZE(map) && tok; ++i) {
        while (tok && !(rc = sscanf(tok, "%hhi", &map[i])))
            tok = strsep(&buf, " \n\r\t");
        if (rc < 0) {
            kfree(save_str);
            return -EIO;
        }
        tok = strsep(&buf, " \n\r\t");
    }
    AMS_MUTEX_LOCK(&chip->lock);
    memset(chip->tof_spad_cfg.spad_configs[1].spad_map, 0,
           sizeof(chip->tof_spad_cfg.spad_configs[1].spad_map));
    memcpy(chip->tof_spad_cfg.spad_configs[1].spad_map, map,
           sizeof(chip->tof_spad_cfg.spad_configs[1].spad_map));
    chip->tof_spad_uncommitted = true;
    AMS_MUTEX_UNLOCK(&chip->lock);
    kfree(save_str);
    return count;
}

static ssize_t commit_spad_cfg_show(struct device * dev,
                                    struct device_attribute * attr,
                                    char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    // return true if spad configuration is committed
    return scnprintf(buf, PAGE_SIZE, "%u\n", !chip->tof_spad_uncommitted);
}

static ssize_t commit_spad_cfg_store(struct device * dev,
                                     struct device_attribute * attr,
                                     const char * buf,
                                     size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    int val;
    sscanf(buf, "%i", &val);
    if (val == 1) {
        AMS_MUTEX_LOCK(&chip->lock);
        rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_SPADCFG, &chip->tof_spad_cfg, NULL);
        if (rc) {
            dev_err(&chip->client->dev, "Error, committing spad config failed.\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        chip->tof_spad_uncommitted = false;
        kfifo_reset(&chip->fifo_out);
        AMS_MUTEX_UNLOCK(&chip->lock);
    }
    return count;
}

static ssize_t reset_spad_cfg_store(struct device * dev,
                                    struct device_attribute * attr,
                                    const char * buf,
                                    size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    int val;
    sscanf(buf, "%i", &val);
    if (val == 1) {
        AMS_MUTEX_LOCK(&chip->lock);
        // read out spad config since spad map id has changed
        memset(&chip->tof_spad_cfg, 0, sizeof(chip->tof_spad_cfg));
        rc = tmf882x_ioctl(&chip->tof, IOCAPP_GET_SPADCFG, NULL, &chip->tof_spad_cfg);
        if (rc) {
            dev_err(&chip->client->dev, "Error, reading spad config failed.\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        // read out fresh spad configuration from device, overwrite local copy
        chip->tof_spad_uncommitted = false;
        AMS_MUTEX_UNLOCK(&chip->lock);
    }
    return count;
}

static ssize_t clock_compensation_show(struct device * dev,
                                       struct device_attribute * attr,
                                       char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool clk_skew_corr_enabled;
    int rc;
    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_IS_CLKADJ, NULL, &clk_skew_corr_enabled);
    if (rc) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        dev_err(&chip->client->dev,
                "Error, reading clock compensation state\n");
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%u\n", clk_skew_corr_enabled);
}

static ssize_t clock_compensation_store(struct device * dev,
                                        struct device_attribute * attr,
                                        const char * buf,
                                        size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    int val = 0;
    rc = sscanf(buf, "%i", &val);
    if (!rc) {
        dev_err(&chip->client->dev, "Error, invalid input\n");
        return -EINVAL;
    }
    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CLKADJ, &val, NULL);
    if (rc) {
        dev_err(&chip->client->dev,
                "Error, setting clock compensation state %u\n", !!val);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t osc_trim_show(struct device * dev,
                             struct device_attribute * attr,
                             char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    bool osc_trim_enabled;
    AMS_MUTEX_LOCK(&chip->lock);
    osc_trim_enabled = !!(chip->tof_cfg.power_cfg &
                          TMF8X2X_COM_POWER_CFG__allow_osc_retrim);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%u\n", osc_trim_enabled);
}

static ssize_t osc_trim_store(struct device * dev,
                              struct device_attribute * attr,
                              const char * buf,
                              size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    int val = 0;
    rc = sscanf(buf, "%i", &val);
    if (!rc) {
        dev_err(&chip->client->dev, "Error, invalid input\n");
        return -EINVAL;
    }
    val = !!val; // convert to 0/1
    AMS_MUTEX_LOCK(&chip->lock);

    if (val)
        chip->tof_cfg.power_cfg |= TMF8X2X_COM_POWER_CFG__allow_osc_retrim;
    else
        chip->tof_cfg.power_cfg &= ~TMF8X2X_COM_POWER_CFG__allow_osc_retrim;

    // write new config
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CFG, &chip->tof_cfg, NULL);
    if (rc) {
        dev_err(&chip->client->dev, "Error, app set config failed.\n");
        return rc;
    }

    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t osc_trim_freq_show(struct device * dev,
                                  struct device_attribute * attr,
                                  char * buf)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    uint32_t osc_trim_freq;
    int rc;
    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_GET_OSC_FREQ, NULL, &osc_trim_freq);
    if (rc) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        dev_err(&chip->client->dev,
                "Error, reading osc trim frequency\n");
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return scnprintf(buf, PAGE_SIZE, "%u\n", osc_trim_freq);
}

static ssize_t osc_trim_freq_store(struct device * dev,
                                   struct device_attribute * attr,
                                   const char * buf,
                                   size_t count)
{
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    uint32_t val = 0;
    rc = sscanf(buf, "%i", &val);
    if (!rc) {
        dev_err(&chip->client->dev, "Error, invalid input\n");
        return -EINVAL;
    }
    AMS_MUTEX_LOCK(&chip->lock);
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_OSC_FREQ, &val, NULL);
    if (rc) {
        dev_err(&chip->client->dev,
                "Error, setting osc frequency %u\n", val);
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t calibration_data_write(struct file * f, struct kobject * kobj,
                                      struct bin_attribute * attr, char *buf,
                                      loff_t off, size_t size)
{
    struct device *dev = kobj_to_dev(kobj);
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    int rc;
    AMS_MUTEX_LOCK(&chip->lock);
    memcpy(chip->tof_calib.data, buf, size);
    chip->tof_calib.calib_len = size;
    rc = tmf882x_ioctl(&chip->tof, IOCAPP_SET_CALIB, &chip->tof_calib, NULL);
    if (rc < 0) {
        dev_err(&chip->client->dev, "Error, writing calibration data\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return size;
}

static ssize_t calibration_data_read(struct file * f, struct kobject * kobj,
                                     struct bin_attribute * attr, char *buf,
                                     loff_t off, size_t size)
{
    struct device *dev = kobj_to_dev(kobj);
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    ssize_t count = 0;

    AMS_MUTEX_LOCK(&chip->lock);
    if (off == 0) {
        count = tmf882x_ioctl(&chip->tof, IOCAPP_GET_CALIB,
                              NULL, &chip->tof_calib);
        if (count) {
            dev_err(&chip->client->dev, "Error, reading calibration data\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        count = chip->tof_calib.calib_len;
    }

    if (off >= chip->tof_calib.calib_len || off >= sizeof(chip->tof_calib.data)) {
        // no more data to give
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }

    count = size < count ? size : count;

    if (size + off > chip->tof_calib.calib_len) {
        count = chip->tof_calib.calib_len - off;
    }

    memcpy(buf, &chip->tof_calib.data[off], count);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static ssize_t factory_calibration_read(struct file * f, struct kobject * kobj,
                                        struct bin_attribute * attr, char *buf,
                                        loff_t off, size_t size)
{
    struct device *dev = kobj_to_dev(kobj);
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    ssize_t count = 0;
    AMS_MUTEX_LOCK(&chip->lock);
    if (off == 0) {
        count = tmf882x_ioctl(&chip->tof, IOCAPP_DO_FACCAL,
                              NULL, &chip->tof_calib);
        if (count) {
            dev_err(&chip->client->dev, "Error, performing factory calibration\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        count = chip->tof_calib.calib_len;
    }

    if (off >= chip->tof_calib.calib_len || off >= sizeof(chip->tof_calib.data)) {
        // no more data to give
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }

    // cap return amount to user space
    count = size < count ? size : count;

    if (size + off > chip->tof_calib.calib_len) {
        count = chip->tof_calib.calib_len - off;
    }

    memcpy(buf, &chip->tof_calib.data[off], count);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

/****************************************************************************
 * Common Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RW(chip_enable);
static DEVICE_ATTR_RW(driver_debug);
/******* READ-ONLY attributes ******/
static DEVICE_ATTR_RO(firmware_version);
static DEVICE_ATTR_RO(registers);
static DEVICE_ATTR_RO(device_uid);
static DEVICE_ATTR_RO(device_revision);
/******* WRITE-ONLY attributes ******/
static DEVICE_ATTR_WO(register_write);
static DEVICE_ATTR_WO(request_ram_patch);

/****************************************************************************
 * Bootloader Sysfs Attributes
 * **************************************************************************/
/******* READ-WRITE attributes ******/
/******* READ-ONLY attributes ******/
/******* WRITE-ONLY attributes ******/

/****************************************************************************
 * app Sysfs Attributes
 * *************************************************************************/
/******* READ-WRITE attributes ******/
static DEVICE_ATTR_RW(capture);
static DEVICE_ATTR_RW(short_range_mode);
static DEVICE_ATTR_RW(report_period_ms);
static DEVICE_ATTR_RW(iterations);
static DEVICE_ATTR_RW(alg_setting);
static DEVICE_ATTR_RW(power_cfg);
static DEVICE_ATTR_RW(gpio_0);
static DEVICE_ATTR_RW(gpio_1);
static DEVICE_ATTR_RW(histogram_dump);
static DEVICE_ATTR_RW(spad_map_id);
static DEVICE_ATTR_RW(zone_mask);
static DEVICE_ATTR_RW(conf_threshold);
static DEVICE_ATTR_RW(low_threshold);
static DEVICE_ATTR_RW(high_threshold);
static DEVICE_ATTR_RW(persistence);
static DEVICE_ATTR_RW(mode_8x8);
static DEVICE_ATTR_RW(xoff_q1_0);
static DEVICE_ATTR_RW(xoff_q1_1);
static DEVICE_ATTR_RW(yoff_q1_0);
static DEVICE_ATTR_RW(yoff_q1_1);
static DEVICE_ATTR_RW(xsize_0);
static DEVICE_ATTR_RW(xsize_1);
static DEVICE_ATTR_RW(ysize_0);
static DEVICE_ATTR_RW(ysize_1);
static DEVICE_ATTR_RW(spad_mask_0);
static DEVICE_ATTR_RW(spad_mask_1);
static DEVICE_ATTR_RW(spad_map_0);
static DEVICE_ATTR_RW(spad_map_1);
static DEVICE_ATTR_RW(commit_spad_cfg);
static DEVICE_ATTR_RW(clock_compensation);
static DEVICE_ATTR_RW(osc_trim);
static DEVICE_ATTR_RW(osc_trim_freq);
/******* WRITE-ONLY attributes ******/
static DEVICE_ATTR_WO(reset_spad_cfg);

/******* READ-WRITE BINARY attributes ******/
static BIN_ATTR_RW(calibration_data, 0);
/******* WRITE-ONLY BINARY attributes ******/
/******* READ-ONLY BINARY attributes ******/
static BIN_ATTR_RO(factory_calibration, 0);

static struct attribute *tof_common_attrs[] = {
    &dev_attr_mode.attr,
    &dev_attr_chip_enable.attr,
    &dev_attr_driver_debug.attr,
    &dev_attr_firmware_version.attr,
    &dev_attr_registers.attr,
    &dev_attr_register_write.attr,
    &dev_attr_request_ram_patch.attr,
    &dev_attr_device_uid.attr,
    &dev_attr_device_revision.attr,
    NULL,
};
static struct attribute *tof_bl_attrs[] = {
    NULL,
};
static struct attribute *tof_app_attrs[] = {
    &dev_attr_capture.attr,
    &dev_attr_short_range_mode.attr,
    &dev_attr_report_period_ms.attr,
    &dev_attr_iterations.attr,
    &dev_attr_alg_setting.attr,
    &dev_attr_power_cfg.attr,
    &dev_attr_gpio_0.attr,
    &dev_attr_gpio_1.attr,
    &dev_attr_histogram_dump.attr,
    &dev_attr_spad_map_id.attr,
    &dev_attr_conf_threshold.attr,
    &dev_attr_low_threshold.attr,
    &dev_attr_high_threshold.attr,
    &dev_attr_persistence.attr,
    &dev_attr_zone_mask.attr,
    &dev_attr_mode_8x8.attr,
    &dev_attr_xoff_q1_0.attr,
    &dev_attr_xoff_q1_1.attr,
    &dev_attr_yoff_q1_0.attr,
    &dev_attr_yoff_q1_1.attr,
    &dev_attr_xsize_0.attr,
    &dev_attr_xsize_1.attr,
    &dev_attr_ysize_0.attr,
    &dev_attr_ysize_1.attr,
    &dev_attr_spad_mask_0.attr,
    &dev_attr_spad_mask_1.attr,
    &dev_attr_spad_map_0.attr,
    &dev_attr_spad_map_1.attr,
    &dev_attr_commit_spad_cfg.attr,
    &dev_attr_reset_spad_cfg.attr,
    &dev_attr_clock_compensation.attr,
    &dev_attr_osc_trim.attr,
    &dev_attr_osc_trim_freq.attr,
    NULL,
};
static struct bin_attribute *tof_app_bin_attrs[] = {
    &bin_attr_factory_calibration,
    &bin_attr_calibration_data,
    NULL,
};
static const struct attribute_group tof_common_group = {
    .attrs = tof_common_attrs,
};
static const struct attribute_group tof_bl_group = {
    .name = "bootloader",
    .attrs = tof_bl_attrs,
};
static const struct attribute_group tof_app_group = {
    .name = "app",
    .attrs = tof_app_attrs,
    .bin_attrs = tof_app_bin_attrs,
};
static const struct attribute_group *tof_groups[] = {
    &tof_common_group,
    &tof_bl_group,
    &tof_app_group,
    NULL,
};

/**
 * tof_frwk_i2c_read - Read number of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the received data
 * @len: number of bytes to read
 */
int tof_frwk_i2c_read(struct tof_sensor_chip *chip, char reg, char *buf, int len)
{
    struct i2c_client *client = chip->client;
    struct i2c_msg msgs[2];
    int ret;

    msgs[0].flags = 0;
    msgs[0].addr  = client->addr;
    msgs[0].len   = 1;
    msgs[0].buf   = &reg;

    msgs[1].flags = I2C_M_RD;
    msgs[1].addr  = client->addr;
    msgs[1].len   = len;
    msgs[1].buf   = buf;

    ret = i2c_transfer(client->adapter, msgs, 2);
    return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

/**
 * tof_frwk_i2c_write - Write nuber of bytes starting at a specific address over I2C
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @buf: pointer to a buffer that will contain the data to write
 * @len: number of bytes to write
 */
int tof_frwk_i2c_write(struct tof_sensor_chip *chip, char reg, const char *buf, int len)
{
    struct i2c_client *client = chip->client;
    u8 *addr_buf;
    struct i2c_msg msg;
    int idx = reg;
    int ret;
    char debug[120];
    u32 strsize = 0;

    addr_buf = kmalloc(len + 1, GFP_KERNEL);
    if (!addr_buf)
        return -ENOMEM;

    addr_buf[0] = reg;
    memcpy(&addr_buf[1], buf, len);
    msg.flags = 0;
    msg.addr = client->addr;
    msg.buf = addr_buf;
    msg.len = len + 1;

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret != 1) {
        dev_err(&client->dev, "i2c_transfer failed: %d msg_len: %u", ret, len);
    }
    if (chip->driver_debug > 2) {
        strsize = scnprintf(debug, sizeof(debug), "i2c_write: ");
        for(idx = 0; (ret == 1) && (idx < msg.len); idx++) {
            strsize += scnprintf(debug + strsize, sizeof(debug) - strsize, "%02x ", addr_buf[idx]);
        }
        dev_info(&client->dev, "%s", debug);
    }

    kfree(addr_buf);
    return ret < 0 ? ret : (ret != 1 ? -EIO : 0);
}

/**
 * tof_frwk_i2c_write_mask - Write a byte to the specified address with a given bitmask
 *
 * @client: the i2c client
 * @reg: the i2c register address
 * @val: byte to write
 * @mask: bitmask to apply to address before writing
 */
static int tof_frwk_i2c_write_mask(struct tof_sensor_chip *chip, char reg,
                                   const char *val, char mask)
{
    int ret;
    u8 temp;

    ret = tof_frwk_i2c_read(chip, reg, &temp, 1);
    temp &= ~mask;
    temp |= *val;
    ret = tof_frwk_i2c_write(chip, reg, &temp, 1);

    return ret;
}

/**
 * tof_hard_reset - use GPIO Chip Enable to reset the device
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_hard_reset(struct tof_sensor_chip *chip)
{
    int error = 0;

    (void) tof_poweroff_device(chip);
    error = tof_open_mode(chip, TMF882X_MODE_APP);
    if (error) {
        dev_err(&chip->client->dev, "Error powering up device: %d\n", error);
        return error;
    }

    return error;
}

/**
 * tof_get_gpio_config - Get GPIO config from DT
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_get_gpio_config(struct tof_sensor_chip *tof_chip)
{
    int error;
    struct device *dev;
    struct gpio_desc *gpiod;

    if (!tof_chip->client)
        return -EINVAL;
    dev = &tof_chip->client->dev;

    /* Get the enable line GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_ENABLE_NAME, GPIOD_OUT_HIGH);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        return error;
    }
    tof_chip->pdata->gpiod_enable = gpiod;

    // HW Chip reset
    if (gpiod) {
        (void) gpiod_direction_output(tof_chip->pdata->gpiod_enable, 0);
        usleep_range(1000, 1001);
        (void) gpiod_direction_output(tof_chip->pdata->gpiod_enable, 1);
    }

    /* Get the interrupt GPIO pin number */
    gpiod = devm_gpiod_get_optional(dev, TOF_GPIO_INT_NAME, GPIOD_IN);
    if (IS_ERR(gpiod)) {
        error = PTR_ERR(gpiod);
        return error;
    }
    tof_chip->pdata->gpiod_interrupt = gpiod;
    return 0;
}

/**
 * tof_ram_patch_callback - The firmware download callback
 *
 * @cfg: the firmware cfg structure
 * @ctx: private data pointer to struct tof_sensor_chip
 */
static void tof_ram_patch_callback(const struct firmware *cfg, void *ctx)
{
    struct tof_sensor_chip *chip = ctx;
    int result = 0;
    u64 fwdl_time = 0;
    struct timespec64 start_ts = {0}, end_ts = {0};
    if (!chip) {
        pr_err("AMS-TOF Error: Ram patch callback NULL context pointer.\n");
    }

    if (!cfg) {
        dev_warn(&chip->client->dev, "%s: Warning, firmware not available.\n", __func__);
        goto err_fwdl;
    }

    // mode switch to the bootloader for FWDL
    if (tmf882x_mode_switch(&chip->tof, TMF882X_MODE_BOOTLOADER)) {
        dev_info(&chip->client->dev, "%s mode switch for FWDL failed\n", __func__);
        tmf882x_dump_i2c_regs(tmf882x_mode_hndl(&chip->tof));
        goto err_fwdl;
    }

    dev_info(&chip->client->dev, "%s: Ram patch in progress...\n", __func__);
    //Start fwdl timer
    ktime_get_real_ts64(&start_ts);
    result = tmf882x_fwdl(&chip->tof, FWDL_TYPE_HEX, cfg->data, cfg->size);
    if (result)
        goto err_fwdl;
    //Stop fwdl timer
    ktime_get_real_ts64(&end_ts);
    //time in ms
    fwdl_time = timespec64_sub(end_ts, start_ts).tv_nsec / 1000000;
    dev_info(&chip->client->dev,
            "%s: Ram patch complete, dl time: %llu ms\n", __func__, fwdl_time);
err_fwdl:
    release_firmware(cfg);
    complete_all(&chip->ram_patch_in_progress);
}

static int tof_poweroff_device(struct tof_sensor_chip *chip)
{
    tmf882x_close(&chip->tof);
    if (!chip->pdata->gpiod_enable) {
        return 0;
    }
    chip->fwdl_needed = true;
    return gpiod_direction_output(chip->pdata->gpiod_enable, 0);
}

/**
 * tof_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t tof_irq_handler(int irq, void *dev_id)
{
    struct tof_sensor_chip *tof_chip = (struct tof_sensor_chip *)dev_id;
    AMS_MUTEX_LOCK(&tof_chip->lock);
    (void) tmf882x_process_irq(&tof_chip->tof);
    // wake up userspace even for errors
    wake_up_interruptible_sync(&tof_chip->fifo_wait);
    AMS_MUTEX_UNLOCK(&tof_chip->lock);
    return IRQ_HANDLED;
}

static int tmf882x_poll_irq_thread(void *tof_chip)
{
    struct tof_sensor_chip *chip = (struct tof_sensor_chip *)tof_chip;
    int us_sleep = 0;
    AMS_MUTEX_LOCK(&chip->lock);
    // Poll period is interpreted in units of 100 usec
    us_sleep = chip->poll_period * 100;
    dev_info(&chip->client->dev,
             "Starting ToF irq polling thread, period: %u us\n", us_sleep);
    AMS_MUTEX_UNLOCK(&chip->lock);
    while (!kthread_should_stop()) {
        (void) tof_irq_handler(0, tof_chip);
        AMS_MUTEX_LOCK(&chip->lock);
        // Poll period is interpreted in units of 100 usec
        us_sleep = chip->poll_period * 100;
        AMS_MUTEX_UNLOCK(&chip->lock);
        usleep_range(us_sleep, us_sleep + us_sleep/10);
    }
    return 0;
}

/**
 * tof_request_irq - request IRQ for given gpio
 *
 * @tof_chip: tof_sensor_chip pointer
 */
static int tof_request_irq(struct tof_sensor_chip *tof_chip)
{
    int irq = tof_chip->client->irq;
    unsigned long default_trigger =
        irqd_get_trigger_type(irq_get_irq_data(irq));
    dev_info(&tof_chip->client->dev,
             "irq: %d, trigger_type: %lu", irq, default_trigger);
    return devm_request_threaded_irq(&tof_chip->client->dev,
                                     tof_chip->client->irq,
                                     NULL, tof_irq_handler,
                                     default_trigger |
                                     IRQF_SHARED     |
                                     IRQF_ONESHOT,
                                     tof_chip->client->name,
                                     tof_chip);
}

int tof_frwk_queue_msg(struct tof_sensor_chip *chip, struct tmf882x_msg *msg)
{
    unsigned int fifo_len;
    int result = kfifo_in(&chip->fifo_out, msg->msg_buf, msg->hdr.msg_len);
    struct tmf882x_msg_error err;

    tof_publish_input_events(chip, msg); // publish any input events

    // handle FIFO overflow case
    if (result != msg->hdr.msg_len) {
        TOF_SET_ERR_MSG(&err, ERR_BUF_OVERFLOW);
        (void) kfifo_in(&chip->fifo_out, (char *)&err, err.hdr.msg_len);
        if (chip->driver_debug == 1)
            dev_err(&chip->client->dev,
                    "Error: Message buffer is full, clearing buffer.\n");
        kfifo_reset(&chip->fifo_out);
        result = kfifo_in(&chip->fifo_out, msg->msg_buf, msg->hdr.msg_len);
        if (result != msg->hdr.msg_len) {
            dev_err(&chip->client->dev,
                    "Error: queueing ToF output message.\n");
        }
    }
    if (chip->driver_debug == 2) {
        fifo_len = kfifo_len(&chip->fifo_out);
        dev_info(&chip->client->dev,
                "New fifo len: %u, fifo utilization: %u%%\n",
                fifo_len, (1000*fifo_len/kfifo_size(&chip->fifo_out))/10);
    }
    return (result == msg->hdr.msg_len) ? 0 : -1;
}

static void tof_idev_close(struct input_dev *dev)
{
    struct tof_sensor_chip *chip = input_get_drvdata(dev);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->open_refcnt--;
    if (!chip->open_refcnt) {
        dev_info(&dev->dev, "%s\n", __func__);
        // tof_poweroff_device(chip);
        // stop measurements
        if (tmf882x_stop(&chip->tof)) {
            dev_info(&dev->dev, "Error stopping measurements\n");
        }
        kfifo_reset(&chip->fifo_out);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return;
}

static int tof_idev_open(struct input_dev *dev)
{
    struct tof_sensor_chip *chip = input_get_drvdata(dev);
    int error = 0;
    AMS_MUTEX_LOCK(&chip->lock);
    if (chip->open_refcnt++) {
        error = tmf882x_start(&chip->tof);
        if (error) {
            dev_err(&dev->dev, "Error, start measurements failed.\n");
            chip->open_refcnt--;
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }

    dev_info(&dev->dev, "%s\n", __func__);
    error = tof_open_mode(chip, TMF882X_MODE_APP);
    if (error) {
        dev_err(&dev->dev, "Chip enable failed.\n");
        chip->open_refcnt--;
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    error = tof_set_default_config(chip);
    if (error) {
        dev_err(&dev->dev, "Error, set default config failed.\n");
        chip->open_refcnt--;
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    error = tmf882x_start(&chip->tof);
    if (error) {
        dev_err(&dev->dev, "Error, start measurements failed.\n");
        chip->open_refcnt--;
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return error;
}

static int tof_misc_release(struct inode *inode, struct file *f)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct tof_sensor_chip *chip =
        container_of(misc, struct tof_sensor_chip, tof_mdev);
    AMS_MUTEX_LOCK(&chip->lock);
    chip->open_refcnt--;
    if (!chip->open_refcnt) {
        dev_info(&chip->client->dev, "%s\n", __func__);
        // tof_poweroff_device(chip);
        kfifo_reset(&chip->fifo_out);
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return 0;
}

static int tof_misc_open(struct inode *inode, struct file *f)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct tof_sensor_chip *chip =
        container_of(misc, struct tof_sensor_chip, tof_mdev);
    int ret;

    if (O_WRONLY == (f->f_flags & O_ACCMODE))
        return -EACCES;

    if (f->f_flags & O_NONBLOCK) {
        ret = AMS_MUTEX_TRYLOCK(&chip->lock);
        if(!ret){
            dev_info(&chip->client->dev, "Error, open would block\n");
            return -EWOULDBLOCK;
        }
    } else {
        AMS_MUTEX_LOCK(&chip->lock);
    }
    if (chip->open_refcnt++) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return 0;
    }

    dev_info(&chip->client->dev, "%s\n", __func__);
    ret = tof_open_mode(chip, TMF882X_MODE_APP);
    if (ret) {
        dev_err(&chip->client->dev, "Chip init failed: %d\n", ret);
        chip->open_refcnt--;
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    ret = tof_set_default_config(chip);
    if (ret) {
        dev_err(&chip->client->dev, "Error, set default config failed.\n");
        chip->open_refcnt--;
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return 0;
}

static ssize_t tof_misc_read(struct file *f, char *buf,
                             size_t len, loff_t *off)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct tof_sensor_chip *chip =
        container_of(misc, struct tof_sensor_chip, tof_mdev);
    unsigned int copied = 0;
    int ret = 0;
    size_t msg_size;
    ssize_t count = 0;

    if (f->f_flags & O_NONBLOCK) {
        ret = AMS_MUTEX_TRYLOCK(&chip->lock);
        if(!ret){
            dev_info(&chip->client->dev, "Error, read would block\n");
            return -EWOULDBLOCK;
        }
    } else {
        AMS_MUTEX_LOCK(&chip->lock);
    }

    // sleep for more data
    while ( kfifo_is_empty(&chip->fifo_out) ) {
        if (f->f_flags & O_NONBLOCK) {
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -ENODATA;
        }
        AMS_MUTEX_UNLOCK(&chip->lock);
        ret = wait_event_interruptible(chip->fifo_wait,
                                       (!kfifo_is_empty(&chip->fifo_out) ||
                                        chip->driver_remove));
        if (ret) return ret;
        else if (chip->driver_remove) return 0;
        AMS_MUTEX_LOCK(&chip->lock);
    }

    count = 0;
    msg_size = tof_fifo_next_msg_size(chip);
    if (len < msg_size) {
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EINVAL;
    }

    do {
        ret = kfifo_to_user(&chip->fifo_out, &buf[count], msg_size, &copied);
        if (ret) {
            dev_err(&chip->client->dev, "Error (%d), reading from fifo\n", ret);
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        count += copied;
        msg_size = tof_fifo_next_msg_size(chip);
        if (!msg_size) break;
    } while (msg_size < (len - count));

    AMS_MUTEX_UNLOCK(&chip->lock);
    return count;
}

static unsigned int tof_misc_poll(struct file *f,
                                  struct poll_table_struct *wait)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct tof_sensor_chip *chip =
        container_of(misc, struct tof_sensor_chip, tof_mdev);

    poll_wait(f, &chip->fifo_wait, wait);
    if (!kfifo_is_empty(&chip->fifo_out))
        return POLLIN | POLLRDNORM;
    return 0;
}

static long tof_misc_ioctl(struct file *f, unsigned int cmd, unsigned long arg)
{
    struct miscdevice *misc = (struct miscdevice *)f->private_data;
    struct tof_sensor_chip *chip =
        container_of(misc, struct tof_sensor_chip, tof_mdev);
    int ret = 0;
    int nr = _IOC_NR(cmd);

    if (_IOC_TYPE(cmd) != TMF882X_IOC_MAG) return -ENOTTY;
    if ((nr < TMF882X_IOC_BASE) || (nr >= TMF882X_IOC_MAXNR)) return -ENOTTY;

    if (f->f_flags & O_NONBLOCK) {
        ret = AMS_MUTEX_TRYLOCK(&chip->lock);
        if(!ret){
            dev_info(&chip->client->dev, "Error, read would block\n");
            return -EWOULDBLOCK;
        }
    } else {
        AMS_MUTEX_LOCK(&chip->lock);
    }

    switch (cmd) {
        case TMF882X_IOCFIFOFLUSH:
            kfifo_reset(&chip->fifo_out);
            break;
        case TMF882X_IOCAPPRESET:
            ret = tof_hard_reset(chip);
            if (ret)
                ret = -EIO;
            break;
        default:
            dev_err(&chip->client->dev, "Error, Unhandled IOCTL cmd\n");
            ret = -ENOTTY;
    }

    AMS_MUTEX_UNLOCK(&chip->lock);
    return ret;
}

static const struct file_operations tof_miscdev_fops = {
    .owner          = THIS_MODULE,
    .read           = tof_misc_read,
    .poll           = tof_misc_poll,
    .unlocked_ioctl = tof_misc_ioctl,
    .open           = tof_misc_open,
    .release        = tof_misc_release,
    .llseek         = noop_llseek,
};

#ifdef CONFIG_TMF882X_QCOM_AP
static int sensors_classdev_enable(struct sensors_classdev *cdev,
                                   unsigned int enable)
{
    struct tof_sensor_chip *chip =
        container_of(cdev, struct tof_sensor_chip, cdev);

    if (enable) {
        chip->tof_idev->open(chip->tof_idev);
    } else {
        chip->tof_idev->close(chip->tof_idev);
    }
    return 0;
}

static int sensors_classdev_poll_delay(struct sensors_classdev *cdev,
                                       unsigned int delay_msec)
{
    struct tof_sensor_chip *chip =
        container_of(cdev, struct tof_sensor_chip, cdev);

    AMS_MUTEX_LOCK(&chip->lock);
    chip->poll_period = delay_msec * 100; // poll period in 100s of usec
    AMS_MUTEX_UNLOCK(&chip->lock);
    return 0;
}
#endif

static int tmf882x_suspend(struct device *dev)
{
    int error = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    AMS_MUTEX_LOCK(&chip->lock);
    dev_info(&chip->client->dev, "%s\n", __func__);
    // save capture state
    error = tmf882x_ioctl(&chip->tof, IOCAPP_IS_MEAS, NULL, &chip->resume_measurements);
    if (error) {
        dev_err(&chip->client->dev, "Error reading measure state.\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }

    tmf882x_close(&chip->tof);
    kfifo_reset(&chip->fifo_out);
    AMS_MUTEX_UNLOCK(&chip->lock);
    return 0;
}

static int tmf882x_resume(struct device *dev)
{
    int error = 0;
    struct tof_sensor_chip *chip = dev_get_drvdata(dev);
    AMS_MUTEX_LOCK(&chip->lock);
    dev_info(&chip->client->dev, "%s\n", __func__);
    error = tof_open_mode(chip, TMF882X_MODE_APP);
    if (error) {
        dev_err(&chip->client->dev, "Chip enable failed.\n");
        AMS_MUTEX_UNLOCK(&chip->lock);
        return -EIO;
    }

    // re-start measurements (if necessary)
    if (chip->resume_measurements) {
        error = tmf882x_start(&chip->tof);
        if (error) {
            dev_err(&chip->client->dev, "Error, start measurements failed.\n");
            AMS_MUTEX_UNLOCK(&chip->lock);
            return -EIO;
        }
        chip->resume_measurements = false;
    }
    AMS_MUTEX_UNLOCK(&chip->lock);
    return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,2,0)
static int tof_probe(struct i2c_client *client)
#else
static int tof_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
    struct tof_sensor_chip *tof_chip;
    int error = 0;
    void *poll_prop_ptr = NULL;
    int i;

    dev_info(&client->dev, "I2C Address: %#04x\n", client->addr);
    __u16 devaddr_buf = client->addr;
    client->addr = TMF_DEFAULT_I2C_ADDR;
    tof_chip = devm_kzalloc(&client->dev, sizeof(*tof_chip), GFP_KERNEL);
    if (!tof_chip)
        return -ENOMEM;

    /***** Setup data structures *****/
    mutex_init(&tof_chip->lock);
    char idevname [30];
    sprintf(idevname, "%s_%02X", TMF882X_NAME, devaddr_buf);
    tof_pdata.tof_name = idevname;
    client->dev.platform_data = (void *)&tof_pdata;
    tof_chip->client = client;
    tof_chip->pdata = &tof_pdata;
    i2c_set_clientdata(client, tof_chip);
    /***** Firmware sync structure initialization*****/
    init_completion(&tof_chip->ram_patch_in_progress);
    //initialize kfifo for frame output
    INIT_KFIFO(tof_chip->fifo_out);
    init_waitqueue_head(&tof_chip->fifo_wait);
    // init core ToF DCB
    tmf882x_init(&tof_chip->tof, tof_chip);

    AMS_MUTEX_LOCK(&tof_chip->lock);

    //Setup input device
    tof_chip->tof_idev = devm_input_allocate_device(&client->dev);
    if (tof_chip->tof_idev == NULL) {
        dev_err(&client->dev, "Error allocating input_dev.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto input_dev_alloc_err;
    }
    tof_chip->tof_idev->name = tof_chip->pdata->tof_name;
    tof_chip->tof_idev->id.bustype = BUS_I2C;
    input_set_drvdata(tof_chip->tof_idev, tof_chip);
    tof_chip->tof_idev->open = tof_idev_open;
    tof_chip->tof_idev->close = tof_idev_close;
    // add attributes to input device
    tof_chip->tof_idev->dev.groups = tof_groups;
    set_bit(EV_ABS, tof_chip->tof_idev->evbit);
    for (i = 0; i < TMF882X_HIST_NUM_TDC * 2; ++i) {
        // allow input event publishing for any channel (2 ch per TDC)
        input_set_abs_params(tof_chip->tof_idev, i, 0, 0xFFFFFFFF, 0, 0);
    }

    // setup misc char device
    tof_chip->tof_mdev.fops = &tof_miscdev_fops;
    char devname [10];
    sprintf(devname, "tof_%02X", devaddr_buf);
    tof_chip->tof_mdev.name = devname;
    tof_chip->tof_mdev.minor = MISC_DYNAMIC_MINOR;

    error = tof_get_gpio_config(tof_chip);
    if (error) {
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto gpio_err;
    }

    poll_prop_ptr = (void *)of_get_property(tof_chip->client->dev.of_node,
                                            TOF_PROP_NAME_POLLIO,
                                            NULL);
    tof_chip->poll_period = poll_prop_ptr ? be32_to_cpup(poll_prop_ptr) : 0;
    if (tof_chip->poll_period == 0) {
        /*** Use Interrupt I/O instead of polled ***/
        /***** Setup GPIO IRQ handler *****/
        if (tof_chip->pdata->gpiod_interrupt) {
            error = tof_request_irq(tof_chip);
            if (error) {
                dev_err(&client->dev, "Interrupt request Failed.\n");
                AMS_MUTEX_UNLOCK(&tof_chip->lock);
                goto gen_err;
            }
        }

    } else {
        /*** Use Polled I/O instead of interrupt ***/
        tof_chip->poll_irq = kthread_run(tmf882x_poll_irq_thread,
                                         (void *)tof_chip,
                                         "tof-irq_poll");
        if (IS_ERR(tof_chip->poll_irq)) {
            dev_err(&client->dev, "Error starting IRQ polling thread.\n");
            error = PTR_ERR(tof_chip->poll_irq);
            AMS_MUTEX_UNLOCK(&tof_chip->lock);
            goto kthread_start_err;
        }
    }

    error = tof_hard_reset(tof_chip);
    if (error) {
        dev_err(&client->dev, "Chip init failed.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto gen_err;
    }

    // Change I2C address only if it is different from default    
    if(devaddr_buf != TMF_DEFAULT_I2C_ADDR)
    {
        dev_info(&client->dev, "Changing I2C Address: %#04x -> %#04x\n", TMF_DEFAULT_I2C_ADDR, devaddr_buf);
        uint8_t buf[2];
    
        // set 0x3E --> I2C_ADDR_CHANGE register
        buf[0] = 0x00;  
        error = tof_i2c_write(tof_chip, 0x3E, buf, 1);
        if (error) {
            dev_err(&client->dev, "Error setting I2C_ADDR_CHANGE.\n");
            AMS_MUTEX_UNLOCK(&tof_chip->lock);
            goto gpio_err;
        }
        
        // set 0x3B with new address --> I2C_SLAVE_ADDRESS register
        buf[0] = devaddr_buf<<1;  
        error = tof_i2c_write(tof_chip, 0x3B, buf, 1);
        if (error) {
            dev_err(&client->dev, "Error setting I2C_SLAVE_ADDRESS.\n");
            AMS_MUTEX_UNLOCK(&tof_chip->lock);
            goto gpio_err;
        }
    
        // set 0x08 with command 0x15 --> CMD_STAT gets CMD_WRITE_CONFIG_PAGE
        buf[0] = 0x15;  
        tof_i2c_write(tof_chip, 0x08, buf, 1); 
        
        // set 0x08 with command 0x21 --> CMD_STAT gets CMD_I2C_SLAVE_ADDRESS
        buf[0] = 0x21;  
        error = tof_i2c_write(tof_chip, 0x08, buf, 1);
        if (error) {
            dev_err(&client->dev, "Error setting CMD_I2C_SLAVE_ADDRESS.\n");
            AMS_MUTEX_UNLOCK(&tof_chip->lock);
            goto gpio_err;
        }
        mdelay(500);
    
        // Continue with new I2C address from device tree
        client->addr = devaddr_buf;

        // check state using new I2C address -> APPID must be 0x3
        error = tof_i2c_read(tof_chip, 0x00, &buf[0], 1);
        if ((error != 0) || (buf[0] != 0x03)) {
            dev_err(&client->dev, "ERROR: Check new I2C address, APPID -> %#04x.\n", buf[0]);
            AMS_MUTEX_UNLOCK(&tof_chip->lock);
            goto gpio_err;
        }
    }

    error = sysfs_create_groups(&client->dev.kobj, tof_groups);
    if (error) {
        dev_err(&client->dev, "Error creating sysfs attribute group.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto gen_err;
    }

    error = input_register_device(tof_chip->tof_idev);
    if (error) {
        dev_err(&client->dev, "Error registering input_dev.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto sysfs_err;
    }

    error = misc_register(&tof_chip->tof_mdev);
    if (error) {
        dev_err(&client->dev, "Error registering misc_dev.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto misc_reg_err;
    }

#ifdef CONFIG_TMF882X_QCOM_AP
    tof_chip->cdev = sensors_cdev;
    tof_chip->cdev.sensors_enable = sensors_classdev_enable;
    tof_chip->cdev.sensors_poll_delay = sensors_classdev_poll_delay;
    error = sensors_classdev_register(&tof_chip->tof_idev->dev,
                                      &tof_chip->cdev);
    if (error) {
        dev_err(&client->dev, "Error registering sensors_classdev.\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto classdev_reg_err;
    }
#endif

    // Turn off device until requested
    // tof_poweroff_device(tof_chip);

    // stop measurements
    if (tmf882x_stop(&tof_chip->tof)) {
        dev_info(&client->dev, "Error stopping measurements\n");
        AMS_MUTEX_UNLOCK(&tof_chip->lock);
        goto gen_err;
    }
    // stopping measurements, lets flush the ring buffer
    kfifo_reset(&tof_chip->fifo_out);

    AMS_MUTEX_UNLOCK(&tof_chip->lock);
    dev_info(&client->dev, "Probe ok.\n");
    return 0;

    /***** Failure case(s), unwind and return error *****/
#ifdef CONFIG_TMF882X_QCOM_AP
classdev_reg_err:
    misc_deregister(&tof_chip->tof_mdev);
#endif
misc_reg_err:
    input_unregister_device(tof_chip->tof_idev);
sysfs_err:
    sysfs_remove_groups(&client->dev.kobj, tof_groups);
gen_err:
    if (tof_chip->poll_period != 0) {
        (void)kthread_stop(tof_chip->poll_irq);
    }
gpio_err:
    if (tof_chip->pdata->gpiod_enable)
        (void) gpiod_direction_output(tof_chip->pdata->gpiod_enable, 0);
kthread_start_err:
input_dev_alloc_err:
    i2c_set_clientdata(client, NULL);
    dev_info(&client->dev, "Probe failed.\n");
    return error;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,18,0)
static void tof_remove(struct i2c_client *client)
#else
static int tof_remove(struct i2c_client *client)
#endif
{
    struct tof_sensor_chip *chip = i2c_get_clientdata(client);

    (void) tof_poweroff_device(chip);
    chip->driver_remove = true;
    wake_up_all(&chip->fifo_wait);

#ifdef CONFIG_TMF882X_QCOM_AP
	sensors_classdev_unregister(&chip->cdev);
#endif

    if (chip->pdata->gpiod_interrupt) {
        devm_gpiod_put(&client->dev, chip->pdata->gpiod_interrupt);
    }

    if (chip->pdata->gpiod_enable) {
        devm_gpiod_put(&client->dev, chip->pdata->gpiod_enable);
    }

    if (chip->poll_period != 0) {
        (void)kthread_stop(chip->poll_irq);
    } else {
        devm_free_irq(&client->dev, client->irq, chip);
    }

    misc_deregister(&chip->tof_mdev);
    input_unregister_device(chip->tof_idev);
    sysfs_remove_groups(&client->dev.kobj,
                        (const struct attribute_group **)&tof_groups);

    i2c_set_clientdata(client, NULL);
    dev_info(&client->dev, "%s\n", __func__);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,18,0)
    return;
#else
    return 0;
#endif
}

static struct i2c_device_id tof_idtable[] = {
    { TMF882X_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, tof_idtable);

static const struct dev_pm_ops tof_pm_ops = {
  .suspend = tmf882x_suspend,
  .resume  = tmf882x_resume,
};

static const struct of_device_id tof_of_match[] = {
    { .compatible = "ams,tmf882x" },
    { }
};
MODULE_DEVICE_TABLE(of, tof_of_match);

static struct i2c_driver tof_driver = {
    .driver = {
        .name = "ams-tof",
        .pm = &tof_pm_ops,
        .of_match_table = of_match_ptr(tof_of_match),
    },
    .id_table = tof_idtable,
    .probe = tof_probe,
    .remove = tof_remove,
};

module_i2c_driver(tof_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("AMS TMF882X ToF sensor driver");
MODULE_VERSION(TMF882X_MODULE_VER);
