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
    uint32_t top; // TODO: Get rid of this, read it from device when requested?

    struct k_sem lock;
	const struct device* dev;
	struct k_work work;
    
};


/* Starts the periodic countdown timer.
 * Note that the duration of the first period can vary slightly.
 * See 4.8.3.FIRST PERIOD DURATION in the application manual.
 */
int counter_rv3032_start(const struct device* dev) {
	const struct counter_rv3032_config* config = dev->config;
    return mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, RV3032_MASK_CONTROL1_TE, RV3032_MASK_CONTROL1_TE);
}

int counter_rv3032_stop(const struct device* dev) {
	const struct counter_rv3032_config* config = dev->config;
    return mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL1, RV3032_MASK_CONTROL1_TE, 0);
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

/* Could be achieved by stopping and starting, but this would have the side 
 * effect mentioned above counter_rv3032_start where the first period's 
 * duration can vary. Better to have the user stop and start themselves.
 */
int counter_rv3032_reset(const struct device* dev) {
    return -ENOTSUP;
}

int counter_rv3032_set_alarm(const struct device* dev, uint8_t chan_id, const struct counter_alarm_cfg* alarm_cfg) {
    

    if (chan_id != 0) {
        LOG_ERR("Invalid chan_id: %d", chan_id);
        return -EINVAL;
    }

    if (alarm_cfg->ticks >= 4096) {
        LOG_ERR("Invalid alarm_cfg->ticks: %d (ticks must be < 4096)", alarm_cfg->ticks);
        return -EINVAL;
    }
    
    if (alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) {
        LOG_ERR("Unsupported alarm_cfg->flags: 0x%X (absolute alarms are not supported, use relative alarms)", alarm_cfg->flags);
        return -ENOTSUP;
    }



}

int counter_rv3032_cancel_alarm(const struct device *dev, uint8_t chan_id) {

}

int counter_rv3032_set_top_value(const struct device* dev, const struct counter_top_cfg* cfg) {
    const struct counter_rv3032_config* config = dev->config;
    struct counter_rv3032_data* data = dev->data;
    
    if (cfg->ticks > config->counter_info.max_top_value) {
        LOG_ERR("cfg->ticks must be %d or lower (actual: %d)", config->counter_info.max_top_value, cfg->ticks);
        return -EINVAL;
    }

    uint8_t buf[2] = {cfg->ticks & 0xff, (cfg->ticks >> 8) & 0xf};
    int err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIMER_VALUE0, buf, sizeof(buf));
    if (err) {
        LOG_ERR("Failed to set top value: %d", cfg->ticks);
        return err;
    }

    data->top = cfg->ticks;
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

    int err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_STATUS, RV3032_MASK_STATUS_TF, 0);
    if (err) {
        LOG_ERR("Failed to read status register for pending interrupt");
        return 0;
    }
    return 1;
}

uint32_t counter_rv3032_get_top_value(const struct device* dev) {
    const struct counter_rv3032_data* data = dev->data;
    return data->top;
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

    if (!device_is_ready(config->mfd)) {
        return -ENODEV;
    }

    // data->top = 0;

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
    