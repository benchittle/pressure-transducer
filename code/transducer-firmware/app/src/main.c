#include <zephyr/kernel.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/rtio.h>

#include <zephyr/drivers/rtc.h>

#define Q31_DIVISOR 2147483648.0f // 2^31

#define TEMP_CHANNEL (struct sensor_chan_spec){SENSOR_CHAN_AMBIENT_TEMP, 0}

LOG_MODULE_REGISTER(i2c_example, LOG_LEVEL_INF);

static const struct device* rv3032_mfd = DEVICE_DT_GET(DT_NODELABEL(rv3032));
static const struct device* rv3032_rtc = DEVICE_DT_GET(DT_NODELABEL(rv3032_rtc));
static const struct device* rv3032_sensor = DEVICE_DT_GET(DT_NODELABEL(rv3032_sensor));
// static const struct device* rv3032_sensor = DEVICE_DT_GET(DT_NODELABEL(rv3032_sensor));

SENSOR_DT_READ_IODEV(temp_iodev, DT_NODELABEL(rv3032_sensor), TEMP_CHANNEL);
RTIO_DEFINE(temp_ctx, 1, 1);

struct alarm_cb_data {
    char* string;
};

void alarm_callback(const struct device *dev, uint16_t id, void *user_data) {
    if (id != 0) {
        LOG_ERR("Invalid ID");
        // return -EINVAL;
    }

    struct alarm_cb_data* data = user_data;

    LOG_INF("String is: %s", data->string);
}

int main(void) {
    uint8_t buf[10];
    struct sensor_q31_data temperature_sensor_data = {0};
    uint16_t mask;
    struct rtc_time t;
    struct alarm_cb_data cb_data = {.string = "BINGUS"};
    int ret;

    struct sensor_decode_context temperature_decoder = SENSOR_DECODE_CONTEXT_INIT(
        SENSOR_DECODER_DT_GET(DT_NODELABEL(rv3032_sensor)),
        buf,
        SENSOR_CHAN_AMBIENT_TEMP,
        0
    );

    LOG_INF("Starting up");
    k_sleep(K_SECONDS(2));
    LOG_INF("Demo starting...");

    if (!device_is_ready(rv3032_mfd)) {
        LOG_ERR("MFD device not ready");
        return 1;
    }

    if (!device_is_ready(rv3032_rtc)) {
        LOG_ERR("RTC device not ready");
        return 1;
    } 

    if (!device_is_ready(rv3032_sensor)) {
        LOG_ERR("Sensor device not ready");
        return 1;
    }

    // TEMPERATURE //

    // const struct sensor_decoder_api *dec;
    // ret = sensor_get_decoder(rv3032_sensor, &dec);

    // To get some info about the data from this device...
    // size_t base_sz, frame_sz;
    // dec->get_size_info(TEMP_CHANNEL, &base_sz, &frame_sz);
    //
    // uint16_t avail_frames;
    // dec->get_frame_count(buf, TEMP_CHANNEL, &avail_frames);\

    // RTC //

    LOG_INF("Getting supported fields");
    if (rtc_alarm_get_supported_fields(rv3032_rtc, 0, &mask) != 0) {
        LOG_ERR("Failed to get supported alarm fields");
        return 1;
    }
    LOG_INF("Supported fields: %02x", mask);

    LOG_INF("Setting callback");
    if ((ret = rtc_alarm_set_callback(rv3032_rtc, 0, alarm_callback, &cb_data)) != 0) {
        LOG_ERR("Failed to set alarm callback: %d", ret);
        return 1;
    }
    
    LOG_INF("Getting time");
    if (rtc_get_time(rv3032_rtc, &t) != 0) {
        LOG_ERR("Failed to get RTC time");
        return 1;
    } 
    t.tm_min += 1;
    if (t.tm_min == 60) {
        t.tm_min = 0;
    }
    LOG_INF("Setting alarm time");
    if (rtc_alarm_set_time(rv3032_rtc, 0, RTC_ALARM_TIME_MASK_MINUTE, &t) != 0) {
        LOG_ERR("Failed to set RTC alarm");
        return 1;
    }
    if (rtc_alarm_get_time(rv3032_rtc, 0, &mask, &t) != 0) {
        LOG_ERR("RTC Alarm Mask: %x;\t\tminutes: %02d;\t\thours: %02d", mask, t.tm_min, t.tm_hour);
        return 1;
    }

    // struct rtc_time t = {
    //     .tm_nsec = 0,
    //     .tm_sec = 0,
    //     .tm_min = 5,
    //     .tm_hour = 22,
    //     .tm_wday = 5,
    //     .tm_mday = 19,
    //     .tm_mon = 8,
    //     .tm_year = 125,
    //     .tm_isdst = -1,
    //     .tm_yday = -1,
    // };
    // if (rtc_set_time(rv3032_mfd, &t) != 0) {
    //     LOG_ERR("Failed to set RTC time");
    // }


    while (true) {
        if (rtc_get_time(rv3032_rtc, &t) != 0) {
            LOG_ERR("Failed to get RTC time");
            continue;
        } 
        
        LOG_INF("%04d-%02d-%02d %02d:%02d:%02d (wday=%d)",
                t.tm_year + 1900, t.tm_mon+1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, t.tm_wday);
        
        // LOG_INF("bingus or whatever");

    ret = sensor_read(&temp_iodev, &temp_ctx, buf, sizeof(buf));
    if (ret != 0) {
        LOG_ERR("sensor_read() failed %d\n", ret);
        return 1;
    }
    ret = sensor_decode(&temperature_decoder, &temperature_sensor_data, 1);
    if (ret < 0) {
        LOG_ERR("sensor_decode() failed %d\n", ret);
        return 1;
    } else if (ret != 1) {
        LOG_ERR("sensor_decode() returned unexpected number of samples to read: %d\n", ret);
        return 1;
    }
    temperature_decoder.fit = 0;

    float temperature = temperature_sensor_data.readings[0].temperature / Q31_DIVISOR * (1 << temperature_sensor_data.shift);
    LOG_INF("Got temperature: %.4f (q31 = 0x%x)", (double) temperature, temperature_sensor_data.readings[0].temperature);

        k_sleep(K_SECONDS(1)); 
    }
    return 0;
}
