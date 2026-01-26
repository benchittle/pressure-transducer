#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/mfd/rv3032.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/timeutil.h>

#include "rtc_utils.h"


#define DT_DRV_COMPAT microcrystal_rv3032_rtc

LOG_MODULE_REGISTER(rtc_rv3032, CONFIG_RTC_LOG_LEVEL);

struct rtc_rv3032_config {
    const struct device* mfd;
};

struct rtc_rv3032_data {
#if RTC_RV3032_INTERRUPTS
    struct k_sem lock;
	const struct device* dev;
	struct k_work work;

#if defined(CONFIG_RTC_ALARM)
	rtc_alarm_callback alarm_callback;
	void* alarm_user_data;
#endif /* CONFIG_RTC_ALARM */
#if defined(CONFIG_RTC_UPDATE)
	rtc_update_callback update_callback;
	void *update_user_data;
#endif /* CONFIG_RTC_UPDATE */
#endif /* RTC_RV3032_INTERRUPTS */
};


void rtc_rv3032_lock_sem(const struct device* dev) {
    struct rtc_rv3032_data* data = dev->data;
    (void)k_sem_take(&data->lock, K_FOREVER);
}

void rtc_rv3032_unlock_sem(const struct device* dev) {
    struct rtc_rv3032_data* data = dev->data;
    k_sem_give(&data->lock);
}

#if RTC_RV3032_INTERRUPTS

/* Called by parent MFD driver to handle interrupts if enabled.
 */
static void rtc_rv3032_work_cb(struct k_work* work) {
	struct rtc_rv3032_data* data = CONTAINER_OF(work, struct rtc_rv3032_data, work);
	const struct device* dev = data->dev;
    const struct rtc_rv3032_config* config = dev->config;

	rtc_alarm_callback alarm_callback = NULL;
	void* alarm_user_data = NULL;
	rtc_update_callback update_callback = NULL;
	void* update_user_data = NULL;
	uint8_t status;
	int err;

	rtc_rv3032_lock_sem(dev);

	err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

#ifdef CONFIG_RTC_ALARM
	if ((status & RV3032_MASK_STATUS_AF) && data->alarm_callback != NULL) {
		status &= ~(RV3032_MASK_STATUS_AF);
		alarm_callback = data->alarm_callback;
		alarm_user_data = data->alarm_user_data;
	}
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	if ((status & RV3032_MASK_STATUS_UF) && data->update_callback != NULL) {
		status &= ~(RV3032_MASK_STATUS_UF);
		update_callback = data->update_callback;
		update_user_data = data->update_user_data;
	}
#endif /* CONFIG_RTC_UPDATE */

	err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

	/* Check if interrupt occurred between STATUS read/write */
    // Needed because a second interrupt (e.g. caused by alarm while still 
    // processing update) would NOT generate an interrupt (pin is still 1)
	err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		goto unlock;
	}

	if (((status & RV3032_MASK_STATUS_AF) && alarm_callback != NULL) ||
	    ((status & RV3032_MASK_STATUS_UF) && update_callback != NULL)) {
		/* Another interrupt occurred while servicing this one */
		k_work_submit(&data->work);
	}

unlock:
	rtc_rv3032_unlock_sem(dev);

	if (alarm_callback != NULL) {
		alarm_callback(dev, 0, alarm_user_data);
		alarm_callback = NULL;
	}

	if (update_callback != NULL) {
		update_callback(dev, update_user_data);
		update_callback = NULL;
	}
}

#endif // RTC_RV3032_INTERRUPTS

static int rtc_rv3032_set_time(const struct device* dev, const struct rtc_time* timeptr) {
    const struct rtc_rv3032_config* config = dev->config;
    uint8_t buf[8];

    if (timeptr == NULL ||
	        !rtc_utils_validate_rtc_time(timeptr, RV3032_RTC_TIME_MASK) ||
	        (timeptr->tm_year < RV3032_YEAR_OFFSET)) {
		LOG_ERR("invalid time");
		return -EINVAL;
	}

    buf[0] = bin2bcd(timeptr->tm_nsec / 10000000) & RV3032_MASK_TIME_100TH_SECONDS;
    buf[1] = bin2bcd(timeptr->tm_sec)  & RV3032_MASK_TIME_SECONDS;
    buf[2] = bin2bcd(timeptr->tm_min) & RV3032_MASK_TIME_MINUTES;
    buf[3] = bin2bcd(timeptr->tm_hour) & RV3032_MASK_TIME_HOURS;
    buf[4] = bin2bcd(timeptr->tm_wday) & RV3032_MASK_TIME_WEEKDAY;
    buf[5] = bin2bcd(timeptr->tm_mday) & RV3032_MASK_TIME_DATE;
    buf[6] = bin2bcd(timeptr->tm_mon + RV3032_MONTH_OFFSET) & RV3032_MASK_TIME_MONTH;
    buf[7] = bin2bcd(timeptr->tm_year - RV3032_YEAR_OFFSET) & RV3032_MASK_TIME_YEAR;

    return mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_TIME_100TH_SECONDS, buf, sizeof(buf));
}

static int rtc_rv3032_get_time(const struct device* dev, struct rtc_time* timeptr) {
    const struct rtc_rv3032_config* config = dev->config;
	uint8_t buf[8];
	int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_TIME_100TH_SECONDS, buf, sizeof(buf));

	if (err != 0) {
		return err;
	}

    timeptr->tm_nsec    = bcd2bin(buf[0] & RV3032_MASK_TIME_100TH_SECONDS) * 10000000;
	timeptr->tm_sec     = bcd2bin(buf[1] & RV3032_MASK_TIME_SECONDS);
	timeptr->tm_min     = bcd2bin(buf[2] & RV3032_MASK_TIME_MINUTES);
	timeptr->tm_hour    = bcd2bin(buf[3] & RV3032_MASK_TIME_HOURS);
	timeptr->tm_wday    = bcd2bin(buf[4] & RV3032_MASK_TIME_WEEKDAY);
	timeptr->tm_mday    = bcd2bin(buf[5] & RV3032_MASK_TIME_DATE);
	timeptr->tm_mon     = bcd2bin(buf[6] & RV3032_MASK_TIME_MONTH) - RV3032_MONTH_OFFSET; // rtc range is 1-12, tm_mon is 0-11
	timeptr->tm_year    = bcd2bin(buf[7] & RV3032_MASK_TIME_YEAR) + RV3032_YEAR_OFFSET; // rtc stores years since 2000, tm_year is years since 1900
    timeptr->tm_isdst   = -1;
	timeptr->tm_yday    = -1;

	return 0;
}

#ifdef CONFIG_RTC_ALARM

static int rtc_rv3032_alarm_get_supported_fields(const struct device* dev, uint16_t id, 
                                                 uint16_t* mask) {
    ARG_UNUSED(dev);
    if (id != 0) {
        return -EINVAL;
    }
    *mask = RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY;
    return 0;
}

static int rtc_rv3032_alarm_set_time(const struct device* dev, uint16_t id, uint16_t mask,
				                     const struct rtc_time* timeptr) {
    const struct rtc_rv3032_config* config = dev->config;
    uint8_t buf[3];

    if (id != 0) {
		LOG_ERR("invalid alarm ID: %d", id);
		return -EINVAL;
	}

	if (mask & ~(RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR | RTC_ALARM_TIME_MASK_MONTHDAY)) {
		LOG_ERR("unsupported alarm field mask: 0x%04x", mask);
		return -EINVAL;
	}

	if (!rtc_utils_validate_rtc_time(timeptr, mask)) {
		LOG_ERR("invalid alarm time");
		return -EINVAL;
	}

    if (mask & RTC_ALARM_TIME_MASK_MINUTE) {
        buf[0] = (bin2bcd(timeptr->tm_min) & RV3032_MASK_ALARM_TIME_MINUTES);
    } else {
        buf[0] = RV3032_MASK_ALARM_TIME_MINUTES_AE_M;
    }
    if (mask & RTC_ALARM_TIME_MASK_HOUR) {
        buf[1] = (bin2bcd(timeptr->tm_hour)  & RV3032_MASK_ALARM_TIME_HOURS);
    } else {
        buf[1] = RV3032_MASK_ALARM_TIME_HOURS_AE_H;
    }
    if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
        buf[2] = (bin2bcd(timeptr->tm_mday) & RV3032_MASK_ALARM_TIME_DATE);
    } else {
        buf[2] = RV3032_MASK_ALARM_TIME_DATE_AE_D;
    }

    LOG_DBG("set alarm: mday = %d, hour = %d, min = %d, mask = 0x%04x",
		    timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min, mask);

    return mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_ALARM_TIME_MINUTES, buf, sizeof(buf));
}

static int rtc_rv3032_alarm_get_time(const struct device* dev, uint16_t id, uint16_t* mask,
				                       struct rtc_time* timeptr) {
    const struct rtc_rv3032_config* config = dev->config;
    uint8_t buf[3];

    if (id != 0) {
		LOG_ERR("invalid alarm ID: %d", id);
		return -EINVAL;
	}

    memset(timeptr, 0, sizeof(*timeptr));
    *mask = 0;

    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_ALARM_TIME_MINUTES, buf, sizeof(buf));
    if (err != 0) {
        return err;
    }

    if ((buf[0] & RV3032_MASK_ALARM_TIME_MINUTES_AE_M) == 0) {
        timeptr->tm_min = bcd2bin(buf[0] & RV3032_MASK_ALARM_TIME_MINUTES);
        *mask |= RTC_ALARM_TIME_MASK_MINUTE;
    }
    if ((buf[1] & RV3032_MASK_ALARM_TIME_HOURS_AE_H) == 0) {
        timeptr->tm_min = bcd2bin(buf[1] & RV3032_MASK_ALARM_TIME_HOURS);
        *mask |= RTC_ALARM_TIME_MASK_HOUR;
    }
    if ((buf[2] & RV3032_MASK_ALARM_TIME_DATE_AE_D) == 0) {
        timeptr->tm_min = bcd2bin(buf[2] & RV3032_MASK_ALARM_TIME_DATE);
        *mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
    }

    LOG_DBG("get alarm: mday = %d, hour = %d, min = %d, mask = 0x%04x",
		    timeptr->tm_mday, timeptr->tm_hour, timeptr->tm_min, *mask);

    return 0;
}

static int rtc_rv3032_alarm_is_pending(const struct device* dev, uint16_t id) {
    const struct rtc_rv3032_config* config = dev->config;
    if (id != 0) {
        return -EINVAL;
    }
    uint8_t status;
    int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
    if (err) {
        return err;
    } 

    if (status & RV3032_MASK_STATUS_AF) {
        // Clear flag
        status = ~ (uint8_t) RV3032_MASK_STATUS_AF;
        err = mfd_rv3032_i2c_set_registers(config->mfd, RV3032_REG_STATUS, &status, sizeof(status));
        if (err) {
            return err;
        }
        return 1;
    } else {
        return 0;
    }
}

static int rtc_rv3032_alarm_set_callback(const struct device* dev, uint16_t id,
				                         rtc_alarm_callback callback, void* user_data) {
	const struct rtc_rv3032_config* config = dev->config;
    const struct mfd_rv3032_config* mfd_config = config->mfd->config;
	struct rtc_rv3032_data* data = dev->data;
	int err;

	if (mfd_config->gpio_int.port == NULL) {
		return -ENOTSUP;
	}

	if (id != 0) {
		LOG_ERR("invalid alarm ID %d", id);
		return -EINVAL;
	}

	rtc_rv3032_lock_sem(dev);

	data->alarm_callback = callback;
	data->alarm_user_data = user_data;

	err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL2, 
                                         RV3032_MASK_CONTROL2_AIE,
				                         callback != NULL ? RV3032_MASK_CONTROL2_AIE : 0);
	if (err) {
        LOG_ERR("failed i2c");
		goto unlock;
	}
    LOG_INF("succeeded i2c");

unlock:
	rtc_rv3032_unlock_sem(dev);

	/* Alarm flag may already be set */
	k_work_submit(&data->work);

	return err;
}

#endif /* CONFIG_RTC_ALARM */

#if defined(CONFIG_RTC_UPDATE)

static int rtc_rv3032_update_set_callback(const struct device *dev, rtc_update_callback callback,
				                          void *user_data) {
	const struct rtc_rv3032_config* config = dev->config;
    const struct mfd_rv3032_config* mfd_config = config->mfd->config;
	struct rtc_rv3032_data* data = dev->data;
	int err;

	if (mfd_config->gpio_int.port == NULL) {
		return -ENOTSUP;
	}

	rtc_rv3032_lock_sem(dev);

	data->update_callback = callback;
	data->update_user_data = user_data;

	err = mfd_rv3032_i2c_update_register(config->mfd, RV3032_REG_CONTROL2, RV3032_MASK_CONTROL2_UIE,
				                         callback != NULL ? RV3032_MASK_CONTROL2_UIE : 0);
	if (err) {
		goto unlock;
	}

unlock:
	rtc_rv3032_unlock_sem(dev);

	/* Seconds flag may already be set */
	k_work_submit(&data->work);

	return err;
}

#endif /* defined(CONFIG_RTC_UPDATE) */

#if defined(CONFIG_RTC_CALIBRATION)

int rtc_rv3032_set_calibration(const struct device *dev, int32_t calibration) {
    return -ENOTSUP;
}

int rtc_rv3032_get_calibration(const struct device *dev, int32_t* calibration) {
    return -ENOTSUP;
}

#endif

static DEVICE_API(rtc, driver_api) = {
	.set_time = rtc_rv3032_set_time,
	.get_time = rtc_rv3032_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = rtc_rv3032_alarm_get_supported_fields,
	.alarm_set_time = rtc_rv3032_alarm_set_time,
	.alarm_get_time = rtc_rv3032_alarm_get_time,
	.alarm_is_pending = rtc_rv3032_alarm_is_pending,
	.alarm_set_callback = rtc_rv3032_alarm_set_callback,
#endif /* CONFIG_RTC_ALARM */

#ifdef CONFIG_RTC_UPDATE
	.update_set_callback = rtc_rv3032_update_set_callback,
#endif /* CONFIG_RTC_UPDATE */

#ifdef CONFIG_RTC_CALIBRATION
    .set_calibration = rtc_rv3032_set_calibration,
    .get_calibration = rtc_rv3032_get_calibration,
#endif /* CONFIG_RTC_CALIBRATION */
};

static int rtc_rv3032_init(const struct device* dev) {
	const struct rtc_rv3032_config* config = dev->config;
    struct rtc_rv3032_data* data = dev->data;
    struct mfd_rv3032_data* mfd_data = config->mfd->data;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

#if RTC_RV3032_INTERRUPTS
    k_sem_init(&data->lock, 1, 1);
    data->dev = dev;
    data->work.handler = rtc_rv3032_work_cb;
    mfd_data->work_rtc = &data->work;
#endif /* RTC_RV3032_INTERRUPTS */

	return 0;
}

#define RTC_RV3032_DEFINE(inst)                                                 \
    static const struct rtc_rv3032_config config_##inst = {                     \
        .mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),                             \
    };                                                                          \
    static struct rtc_rv3032_data data_##inst;                                  \
    DEVICE_DT_INST_DEFINE(inst,                                                 \
        rtc_rv3032_init,                                                        \
        NULL,                                                                   \
        &data_##inst,                                                           \
        &config_##inst,                                                         \
        POST_KERNEL,                                                            \
        CONFIG_RTC_RV3032_INIT_PRIORITY,                                        \
        &driver_api                                                             \
    );

DT_INST_FOREACH_STATUS_OKAY(RTC_RV3032_DEFINE)