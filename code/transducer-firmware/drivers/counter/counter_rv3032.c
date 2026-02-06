#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/mfd/rv3032.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>


#define DT_DRV_COMPAT microcrystal_rv3032_counter

LOG_MODULE_REGISTER(counter_rv3032, CONFIG_COUNTER_LOG_LEVEL);

struct counter_rv3032_config {
    struct counter_config_info counter_info;
    const struct device* mfd;
};

struct counter_rv3032_data {
    bool counter_is_running;

    struct k_sem lock;
	const struct device* dev;
	struct k_work work;
    
    counter_alarm_callback_t alarm_callback;
    void* alarm_user_data;

    counter_top_callback_t top_callback;
    void* top_user_data;
};

void counter_rv3032_lock_sem(const struct device* dev) {
    struct counter_rv3032_data* data = dev->data;
    (void)k_sem_take(&data->lock, K_FOREVER);
}

void counter_rv3032_unlock_sem(const struct device* dev) {
    struct counter_rv3032_data* data = dev->data;
    k_sem_give(&data->lock);
}

/* Called by parent MFD driver to handle interrupts if enabled.
 */
static void counter_rv3032_work_cb(struct k_work* work) {
	struct counter_rv3032_data* data = CONTAINER_OF(work, struct counter_rv3032_data, work);
	const struct device* dev = data->dev;
    const struct counter_rv3032_config* config = dev->config;

	counter_alarm_callback_t alarm_callback = NULL;
    counter_top_callback_t top_callback = NULL;
	uint8_t status;
	int err;

	counter_rv3032_lock_sem(dev);

	err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

	if ((status & RV3032_MASK_STATUS_TF) && (data->alarm_callback != NULL || data->top_callback != NULL)) {
		status &= ~(RV3032_MASK_STATUS_AF);
		alarm_callback = data->alarm_callback;
        top_callback = data->top_callback;
	}

	err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

	/* Don't need to check if interrupt occurred between STATUS read/write 
       since the interrupt pin is auto cleared even if the status flag isn't. */

unlock:
	counter_rv3032_unlock_sem(dev);

	if (alarm_callback != NULL) {
		alarm_callback(dev, 0, 0, data->alarm_user_data);
		data->alarm_callback = NULL;
        data->alarm_user_data = NULL;
	} else if (top_callback != NULL) {
        top_callback(dev, data->top_user_data);
    }
}

/* Starts the periodic countdown timer.
 * Note that the duration of the first period can vary slightly.
 * See 4.8.3.FIRST PERIOD DURATION in the application manual.
 */
int counter_rv3032_start(const struct device* dev) {
	const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, RV3032_MASK_CONTROL1_TE, RV3032_MASK_CONTROL1_TE);
    if (err) {
        return err;
    }
    data->counter_is_running = true;
    return 0;
}

int counter_rv3032_stop(const struct device* dev) {
	const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    int err =  mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, RV3032_MASK_CONTROL1_TE, 0);
    if (err) {
        return err;
    }
    data->counter_is_running = false;
    return 0;
}

/* The RV-3032 doesn't support retrieving the current "ticks" value of the 
 * periodic countdown timer.
 */
int counter_rv3032_get_value(const struct device* dev, uint32_t* ticks) {
    return -ENOTSUP;
}

int counter_rv3032_get_value_64(const struct device* dev, uint64_t* ticks) {
	return -ENOTSUP;
}

int counter_rv3032_reset(const struct device* dev) {
    int err = counter_rv3032_stop(dev);
    if (err) {
        LOG_ERR("Failed to stop counter for reset (err = %d)", err);
        return err;
    }
    err = counter_rv3032_start(dev);
    if (err) {
        LOG_ERR("Failed to stop counter for reset (err = %d)", err);
        return err;
    }
    return 0;
}

int counter_rv3032_set_alarm(const struct device* dev, uint8_t chan_id, const struct counter_alarm_cfg* alarm_cfg) {
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;

    if (chan_id != 0) {
        LOG_ERR("Invalid chan_id: %d", chan_id);
        return -EINVAL;
    }

    if (alarm_cfg->ticks > config->counter_info.max_top_value) {
        LOG_ERR("alarm_cfg->ticks must be %d or lower (actual: %d)", config->counter_info.max_top_value, alarm_cfg->ticks);
        return -EINVAL;
    }
    
    if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
        LOG_ERR("Unsupported alarm_cfg->flags: 0x%X (absolute alarms are not supported, use relative alarms)", alarm_cfg->flags);
        return -ENOTSUP;
    }

    data->alarm_callback = alarm_cfg->callback;
    data->alarm_user_data = alarm_cfg->user_data;

    data->top_callback = NULL;
    data->top_user_data = NULL;

    return 0;
}

int counter_rv3032_cancel_alarm(const struct device* dev, uint8_t chan_id) {
    struct counter_rv3032_data* data = dev->data;

    if (chan_id != 0) {
        LOG_ERR("Invalid chan_id: %d", chan_id);
        return -EINVAL;
    }

    data->alarm_callback = NULL;
    data->alarm_user_data = NULL;

    return 0;
}

int counter_rv3032_set_top_value(const struct device* dev, const struct counter_top_cfg* top_cfg) {
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    bool counter_was_running = false;
    int err;
    
    if (top_cfg->ticks > config->counter_info.max_top_value) {
        LOG_ERR("top_cfg->ticks must be %d or lower (actual: %d)", config->counter_info.max_top_value, top_cfg->ticks);
        return -EINVAL;
    }

    if (top_cfg->flags & COUNTER_TOP_CFG_RESET_WHEN_LATE) {
        LOG_ERR("Unsupported top_cfg->flags: 0x%X (COUNTER_TOP_CFG_RESET_WHEN_LATE=1 is not supported)", top_cfg->flags);
        return -ENOTSUP;
    }

    // If counter should be reset, we can stop and start it again to reload the
    // top value.
    if (!(top_cfg->flags & COUNTER_TOP_CFG_DONT_RESET) && data->counter_is_running) {
        err = counter_rv3032_stop(dev);
        if (err) {
            LOG_ERR("Failed to pause counter (err = %d)", err);
            return err;
        }
        counter_was_running = true;
    }

    uint8_t buf[2] = {top_cfg->ticks & 0xff, (top_cfg->ticks >> 8) & 0xf};
    err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to set top value: %d", top_cfg->ticks);
        return err;
    }

    data->top_callback = top_cfg->callback;
    data->top_user_data = top_cfg->user_data;

    data->alarm_callback = NULL;
    data->alarm_user_data = NULL;

    if (counter_was_running) {
        err = counter_rv3032_start(dev);
        if (err) {
            LOG_ERR("Failed to restart counter (err = %d)", err);
            return err;
        }
    }

    return 0;
}

/* If the alarm flag is set in the status register, clear it and return 1
 * Returns 0 if no interrupts are pending or if an error occurred.
 */
uint32_t counter_rv3032_get_pending_int(const struct device* dev) {
    const struct counter_rv3032_config* config = dev->config;
    uint8_t status;

    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
    if (err) {
        LOG_ERR("Failed to read status register for pending interrupt");
        return 0;
    }

    if (!(status & RV3032_MASK_STATUS_TF)) {
        return 0;
    }

    err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_STATUS, RV3032_MASK_STATUS_TF, 0);
    if (err) {
        LOG_ERR("Failed to read status register for pending interrupt");
        return 0;
    }
    return 1;
}

uint32_t counter_rv3032_get_top_value(const struct device* dev) {
    const struct counter_rv3032_config* config = dev->config;
    uint8_t buf[2];

    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        return UINT32_MAX;
    }
    return ((buf[1] & 0b1111) << 8) | (buf[0]);
}

uint32_t counter_rv3032_get_guard_period(const struct device* dev, uint32_t flags) {
	return 0;
}

int counter_rv3032_set_guard_period(const struct device* dev, uint32_t ticks, uint32_t flags) {
	return -ENOTSUP;
}

uint32_t counter_rv3032_get_freq(const struct device *dev) {
    const struct counter_rv3032_config* config = dev->config;
    return config->counter_info.freq;
}

static DEVICE_API(counter, driver_api) = {
    .start = counter_rv3032_start,
    .stop = counter_rv3032_stop,
    .get_value = counter_rv3032_get_value,
    .get_value_64 = counter_rv3032_get_value_64,
    .reset = counter_rv3032_reset,
    .set_alarm = counter_rv3032_set_alarm,
    .cancel_alarm = counter_rv3032_cancel_alarm,
    .set_top_value = counter_rv3032_set_top_value,
    .get_pending_int = counter_rv3032_get_pending_int,
    .get_top_value = counter_rv3032_get_top_value,
    .get_guard_period = counter_rv3032_get_guard_period,
    .set_guard_period = counter_rv3032_set_guard_period,
    .get_freq = counter_rv3032_get_freq,
};

static int counter_rv3032_init(const struct device* dev) {
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    struct mfd_rv3032_data* mfd_data = config->mfd->data;

    if (!device_is_ready(config->mfd)) {
        return -ENODEV;
    }

    data->counter_is_running = false;

    k_sem_init(&data->lock, 1, 1);
    data->dev = dev;
    data->work.handler = counter_rv3032_work_cb;
    mfd_data->work_counter = &data->work;

    data->alarm_callback = NULL;
    data->alarm_user_data = NULL;
    data->top_callback = NULL;
    data->top_user_data = NULL;

    return 0;
}

#define COUNTER_RV3032_FREQ_FROM_DT_INST(inst)                              \
	UTIL_CAT(COUNTER_RV3032_FREQ_, DT_INST_STRING_UPPER_TOKEN(inst, freq))

#define COUNTER_RV3032_DEFINE(inst)                                 \
    static const struct counter_rv3032_config config_##inst = {     \
        .counter_info = {                                           \
            .max_top_value = 4095,                                  \
            .freq = COUNTER_RV3032_FREQ_FROM_DT_INST(inst),         \
            .flags = COUNTER_CONFIG_INFO_COUNT_UP,                  \
            .channels = 1,                                          \
        },                                                          \
        .mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                 \
    };                                                              \
    static struct counter_rv3032_data data_##inst;                  \
    DEVICE_DT_INST_DEFINE(                                          \
        inst,                                                       \
        counter_rv3032_init,                                        \
        NULL,                                                       \
        &data_##inst,                                               \
        &config_##inst,                                             \
        POST_KERNEL,                                                \
        CONFIG_COUNTER_RV3032_INIT_PRIORITY,                        \
        &driver_api                                                 \
    );

DT_INST_FOREACH_STATUS_OKAY(COUNTER_RV3032_DEFINE)
    