#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/mfd/rv3032.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


#define DT_DRV_COMPAT microcrystal_rv3032_counter

LOG_MODULE_REGISTER(counter_rv3032, CONFIG_COUNTER_LOG_LEVEL);

struct counter_rv3032_config {
    // This must be the first field, since API functions like 
    // counter_get_max_top_value cast dev->config into a
    // struct counter_config_info
    struct counter_config_info counter_info;
    uint8_t freq_config;
    const struct device* mfd;
};

struct counter_rv3032_data {
    bool counter_is_running;
    uint16_t top_value;

    struct k_sem lock;
	const struct device* dev;
	struct k_work work;
    
    bool alarm_is_set;
    void *callback;
    void *user_data;
};

void counter_rv3032_lock_sem(const struct device* dev) 
{
    struct counter_rv3032_data* data = dev->data;
    (void)k_sem_take(&data->lock, K_FOREVER);
}

void counter_rv3032_unlock_sem(const struct device* dev) 
{
    struct counter_rv3032_data* data = dev->data;
    k_sem_give(&data->lock);
}

int counter_rv3032_stop(const struct device* dev);

/* Called by parent MFD driver to handle interrupts if enabled.
 */
static void counter_rv3032_work_cb(struct k_work *work) 
{
	struct counter_rv3032_data *data = CONTAINER_OF(work, struct counter_rv3032_data, work);
	const struct device *dev = data->dev;
    const struct counter_rv3032_config *config = dev->config;

    bool is_alarm_callback;
	void *callback = NULL;
    void *user_data = NULL;

	int err;
	uint8_t status;
    uint8_t buf[2] = {0};

	counter_rv3032_lock_sem(dev);

	err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

	if (status & RV3032_MASK_STATUS_TF) {
        is_alarm_callback = data->alarm_is_set;
        callback = data->callback;
        user_data = data->user_data;

        if (is_alarm_callback) {
            data->alarm_is_set = false;
            data->callback = NULL;
            data->user_data = NULL;
            
            err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
            if (err) {
                LOG_ERR("Failed to clear timer top value while clearing alarm (err=%d)", err);
                goto unlock;
            }
        }

		status &= ~(RV3032_MASK_STATUS_AF);
        err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
        if (err) {
            goto unlock;
        }
	}

	/* Don't need to check if interrupt occurred between STATUS read/write 
     * since the interrupt pin is auto cleared even if the status flag isn't. 
     */

unlock:
	counter_rv3032_unlock_sem(dev);

	if (callback == NULL) {
        return;
    }

    if (is_alarm_callback) {
		((counter_alarm_callback_t) callback)(dev, 0, 0, user_data);
	} else {
        ((counter_top_callback_t) callback)(dev, user_data);
    }
}

/* Starts the periodic countdown timer.
 * Note that the duration of the first period can vary slightly.
 * See 4.8.3.FIRST PERIOD DURATION in the application manual.
 */
int counter_rv3032_start(const struct device* dev) 
{
	const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, 
                                             RV3032_MASK_CONTROL1_TE, RV3032_MASK_CONTROL1_TE);
    if (err) {
        return err;
    }
    data->counter_is_running = true;
    return 0;
}

int counter_rv3032_stop(const struct device* dev) 
{
	const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err =  mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, 
                                              RV3032_MASK_CONTROL1_TE, 0);
    if (err) {
        return err;
    }
    data->counter_is_running = false;
    return 0;
}

/* The RV-3032 doesn't support retrieving the current "ticks" value of the 
 * periodic countdown timer.
 */
int counter_rv3032_get_value(const struct device* dev, uint32_t* ticks) 
{
    return -ENOTSUP;
}

int counter_rv3032_reset(const struct device* dev) 
{
    const struct counter_rv3032_config *config = dev->config;
    struct counter_rv3032_data *data = dev->data;
    // int err = counter_rv3032_stop(dev);
    // if (err) {
    //     LOG_ERR("Failed to stop counter for reset (err = %d)", err);
    //     return err;
    // }
    // err = counter_rv3032_start(dev);
    // if (err) {
    //     LOG_ERR("Failed to stop counter for reset (err = %d)", err);
    //     return err;
    // }

    uint8_t timer_value_0 = data->top_value & 0xff;
    int err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, &timer_value_0, sizeof(timer_value_0));
    return err;
}

int counter_rv3032_set_alarm(const struct device* dev, uint8_t chan_id, 
                             const struct counter_alarm_cfg* alarm_cfg) 
{
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err;
    bool counter_was_running = false;

    /* Don't need to check channel ID, this is handled by intermediate API */

    if (alarm_cfg->ticks > config->counter_info.max_top_value) {
        LOG_ERR("alarm_cfg->ticks must be %d or lower (actual: %d)", 
                config->counter_info.max_top_value, alarm_cfg->ticks);
        return -EINVAL;
    }
    
	if (alarm_cfg->flags & (COUNTER_ALARM_CFG_ABSOLUTE | COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE)) {
        LOG_ERR("Unsupported alarm_cfg->flags: 0x%X" 
                " (absolute alarms / expire when late are not supported, use relative alarms)", alarm_cfg->flags);
        return -ENOTSUP;
    }

    counter_rv3032_lock_sem(dev);

    if (data->alarm_is_set) {
        err = -EBUSY;
        goto unlock;
    }

    if (data->counter_is_running) {
        err = counter_rv3032_stop(dev);
        if (err) {
            LOG_ERR("Failed to pause counter (err = %d)", err);
            goto unlock;
        }
        counter_was_running = true;
    }

    uint8_t buf[2] = {alarm_cfg->ticks & 0xff, (alarm_cfg->ticks >> 8) & 0xf};
    err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to set top value: %d", alarm_cfg->ticks);
        goto unlock;
    }

    data->alarm_is_set = true;
    data->callback = alarm_cfg->callback;
    data->user_data = alarm_cfg->user_data;

    if (counter_was_running) {
        err = counter_rv3032_start(dev);
        if (err) {
            LOG_ERR("Failed to restart counter (err = %d)", err);
            goto unlock;
        }
    }

unlock:
    counter_rv3032_unlock_sem(dev);

    return err;
}

int counter_rv3032_cancel_alarm(const struct device* dev, uint8_t chan_id) 
{
    const struct counter_rv3032_config *config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err = 0;
    uint8_t buf[2] = {0};

    /* Don't need to check channel ID, this is handled by intermediate API */

    counter_rv3032_lock_sem(dev);

    if (!data->counter_is_running) {
        err = -ENOTSUP;
        goto unlock;
    }

    err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to clear timer top value while clearing alarm (err=%d)", err);
        goto unlock;
    }

    data->alarm_is_set = false;
    data->callback = NULL;
    data->user_data = NULL;

unlock:
    counter_rv3032_unlock_sem(dev);

    return err;
}

int counter_rv3032_set_top_value(const struct device* dev, const struct counter_top_cfg* top_cfg) 
{
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    bool counter_was_running = false;
    int err = 0;

    /* Don't have to check top_cfg->ticks, this is handled by an intermediate 
     * API function.
    */

    counter_rv3032_lock_sem(dev);

    if (data->alarm_is_set) {
        err = -EBUSY;
        goto unlock;
    }

    if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
        LOG_ERR("Unsupported top_cfg->flags: 0x%X (COUNTER_TOP_CFG_RESET_WHEN_LATE=1 is not supported)", top_cfg->flags);
        err = -ENOTSUP;
        goto unlock;
    }

    /* If counter should be reset, we can stop and start it again to restart it */
    if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) && data->counter_is_running) {
        err = counter_rv3032_stop(dev);
        if (err) {
            LOG_ERR("Failed to pause counter (err = %d)", err);
            goto unlock;
        }
        counter_was_running = true;
    }

    uint8_t buf[2] = {top_cfg->ticks & 0xff, (top_cfg->ticks >> 8) & 0xf};
    err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to set top value: %d", top_cfg->ticks);
        goto unlock;
    }

    data->callback = top_cfg->callback;
    data->user_data = top_cfg->user_data;
    data->top_value = top_cfg->ticks;

    if (counter_was_running) {
        err = counter_rv3032_start(dev);
        if (err) {
            LOG_ERR("Failed to restart counter (err = %d)", err);
            goto unlock;
        }
    }

unlock: 
    counter_rv3032_unlock_sem(dev);

    return err;
}

uint32_t counter_rv3032_get_pending_int(const struct device* dev) 
{
    const struct counter_rv3032_config* config = dev->config;
    uint8_t status;

    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to read status register for pending interrupt");
        return UINT32_MAX;
    }

    return (status & RV3032_MASK_STATUS_TF) > 0;
}

uint32_t counter_rv3032_get_top_value(const struct device* dev) 
{
    const struct counter_rv3032_config* config = dev->config;
    uint8_t buf[2];

    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        return UINT32_MAX;
    }
    return ((buf[1] & 0xf) << 8) | (buf[0]);
}

static DEVICE_API(counter, driver_api) = {
    .start = counter_rv3032_start,
    .stop = counter_rv3032_stop,
    .get_value = counter_rv3032_get_value,
    .reset = counter_rv3032_reset,
    .set_alarm = counter_rv3032_set_alarm,
    .cancel_alarm = counter_rv3032_cancel_alarm,
    .set_top_value = counter_rv3032_set_top_value,
    .get_pending_int = counter_rv3032_get_pending_int,
    .get_top_value = counter_rv3032_get_top_value,
};

static int counter_rv3032_init(const struct device* dev) 
{
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    struct mfd_rv3032_data* mfd_data = config->mfd->data;
    int err;
    uint8_t buf[2] = {0};

    if (!device_is_ready(config->mfd)) {
        return -ENODEV;
    }

    // From data sheet: initialize bits TE, TIE and TF to 0. In that order, to
    // prevent inadvertent interrupts on INT pin.
    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, 
                                         RV3032_MASK_CONTROL1_TE, 0);
    if (err) {
        return -ENODEV;
    }
    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL2, 
                                         RV3032_MASK_CONTROL2_TIE, 0);
    if (err) {
        return -ENODEV;
    }
    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_STATUS, 
                                         RV3032_MASK_STATUS_TF, 0);
    if (err) {
        return -ENODEV;
    }

    // Set periodic countdown timer frequency
    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, 
                                         RV3032_MASK_CONTROL1_TD, config->freq_config);
    if (err) {
        return -ENODEV;
    }

    // Enable interrupts from the periodic countdown timer (which begin to be 
    // triggered once it's enabled by the user calling counter_start).
    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL2, 
                                         RV3032_MASK_CONTROL2_TIE, RV3032_MASK_CONTROL2_TIE);
    if (err) {
        return -ENODEV;
    }

    // Initialize to 0 ticks.
    err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        return -ENODEV;
    }

    data->counter_is_running = false;
    
    k_sem_init(&data->lock, 1, 1);
    data->dev = dev;
    data->work.handler = counter_rv3032_work_cb;
    mfd_data->work_counter = &data->work;
    
    data->alarm_is_set = false;
    data->callback = NULL;
    data->user_data = NULL;

    return 0;
}

#define COUNTER_RV3032_FREQ_FROM_DT_INST(inst)                              \
	UTIL_CAT(COUNTER_RV3032_FREQ_, DT_INST_STRING_UPPER_TOKEN(inst, freq))

// When we set the .freq field in .counter_info below, we must set an integer
// frequency value. This macro maps the register configuration values (0b00 to 
// 0b11) to the actual frequencies, with the exception that both the frequencies 
// 1 Hz and 1/60 Hz are mapped to 1.
#define COUNTER_RV3032_FREQ_CONFIG_TO_HZ(freq_config)    \
    freq_config == COUNTER_RV3032_FREQ_4096 ? 4096 :     \
    freq_config == COUNTER_RV3032_FREQ_64   ? 64 :       \
    freq_config == COUNTER_RV3032_FREQ_1    ? 1 :        \
                                              1

#define COUNTER_RV3032_DEFINE(inst)                                                             \
    static const struct counter_rv3032_config config_##inst = {                                 \
        .counter_info = {                                                                       \
            .max_top_value = 4095,                                                              \
            .freq = COUNTER_RV3032_FREQ_CONFIG_TO_HZ(COUNTER_RV3032_FREQ_FROM_DT_INST(inst)),   \
            .flags = 0,                                                                         \
            .channels = 1,                                                                      \
        },                                                                                      \
        .freq_config = COUNTER_RV3032_FREQ_FROM_DT_INST(inst),                                  \
        .mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                                             \
    };                                                                                          \
    static struct counter_rv3032_data data_##inst;                                              \
    DEVICE_DT_INST_DEFINE(                                                                      \
        inst,                                                                                   \
        counter_rv3032_init,                                                                    \
        NULL,                                                                                   \
        &data_##inst,                                                                           \
        &config_##inst,                                                                         \
        POST_KERNEL,                                                                            \
        CONFIG_COUNTER_RV3032_INIT_PRIORITY,                                                    \
        &driver_api                                                                             \
    );

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RV3032_DEFINE)
    