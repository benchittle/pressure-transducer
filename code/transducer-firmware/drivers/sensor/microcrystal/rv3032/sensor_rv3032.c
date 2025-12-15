#include <zephyr/drivers/mfd/rv3032.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/rtio/work.h>

#define DT_DRV_COMPAT microcrystal_rv3032_sensor


LOG_MODULE_REGISTER(sensor_rv3032, CONFIG_SENSOR_LOG_LEVEL);

struct sensor_rv3032_config {
    const struct device* mfd;
};

struct sensor_rv3032_data {
    const struct device* dev;
	uint16_t raw_temp;
};

struct sensor_rv3032_header {
	uint64_t timestamp;
} __attribute__((__packed__));

// Data for a single sensor read entry
struct sensor_rv3032_edata {
	struct sensor_rv3032_header header;
	uint16_t raw_temp;
};

int sensor_rv3032_read_temp(const struct device* dev, uint16_t* raw_temp) {
	const struct sensor_rv3032_config* config = dev->config;

	uint8_t buf[2];
	int err = mfd_rv3032_i2c_get_registers(config->mfd, RV3032_REG_TEMPERATURE_LSB, buf, 2);
	if (err != 0) {
        return err;
	}
    
    *raw_temp = (uint16_t)(buf[1] << RV3032_TEMPERATURE_FRAC_BITS) | ((buf[0] & RV3032_MASK_TEMPERATURE_LSB_TEMP) >> RV3032_TEMPERATURE_FRAC_BITS);

	return 0;
}

void sensor_rv3032_submit_sync(struct rtio_iodev_sqe* iodev_sqe) {
	uint32_t min_buf_len = sizeof(struct sensor_rv3032_edata);
	int rc;
	uint8_t* buf;
	uint32_t buf_len;

	const struct sensor_read_config* cfg = iodev_sqe->sqe.iodev->data;
	const struct device* dev = cfg->sensor;
	const struct sensor_chan_spec* const channels = cfg->channels;

    // Notes for my understanding:
    // The buffer "returned" here depends on how user of this driver is using RTIO:
    //      if they're using a mempool, it asks for allocation from that pool
    //      if they're passing in fixed space buffer (e.g. sensor_read), it gets ptr to that buffer
    // We specify max len to give mempool an upper limit on memory requirements
	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	struct sensor_rv3032_edata* edata;

	edata = (struct sensor_rv3032_edata*)buf;

    // Return if an invalid channel is provided
	if (cfg->count != 1 || channels[0].chan_type != SENSOR_CHAN_AMBIENT_TEMP || channels[0].chan_idx != 0) {
        // SHOULD WE rtio_iodev_sqe_err() HERE???
		return;
	}

	uint16_t raw_temp;

	rc = sensor_rv3032_read_temp(dev, &raw_temp);
	if (rc != 0) {
		LOG_ERR("Failed to fetch samples");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}
	edata->header.timestamp = k_ticks_to_ns_floor64(k_uptime_ticks());
	edata->raw_temp = raw_temp;

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

void sensor_rv3032_submit(const struct device* dev, struct rtio_iodev_sqe* iodev_sqe) {
    ARG_UNUSED(dev);

	struct rtio_work_req* req = rtio_work_req_alloc();

	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed."
			"Consider to increase CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	/* TODO: optimize with new bus shims
	 * to avoid swapping execution contexts
	 * for a small register read
	 */
	rtio_work_req_submit(req, iodev_sqe, sensor_rv3032_submit_sync);
}

static int sensor_rv3032_decoder_get_frame_count(const uint8_t* buffer,
						                         struct sensor_chan_spec chan_spec,
						                         uint16_t* frame_count) {
	if (chan_spec.chan_type != SENSOR_CHAN_AMBIENT_TEMP || chan_spec.chan_idx != 0) {
		return -ENOTSUP;
	}
    
    *frame_count = 1;
    return 0;
}

static int sensor_rv3032_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t* base_size,
					                           size_t* frame_size) {
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		*base_size = sizeof(struct sensor_q31_sample_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int sensor_rv3032_decoder_decode(const uint8_t* buffer, struct sensor_chan_spec chan_spec,
					                    uint32_t* fit, uint16_t max_count, void* data_out) {
	if (*fit != 0) {
		return 0;
	}

	struct sensor_q31_data* out = data_out;

	out->header.reading_count = 1;

	const struct sensor_rv3032_edata* edata = (const struct sensor_rv3032_edata*)buffer;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		out->header.base_timestamp_ns = edata->header.timestamp;
		const uint16_t raw_temp = edata->raw_temp;

		out->shift = RV3032_TEMPERATURE_INT_BITS;
		out->readings[0].temperature = (q31_t)raw_temp << (31 - (RV3032_TEMPERATURE_INT_BITS + RV3032_TEMPERATURE_FRAC_BITS));

		break;
	default:
		return -EINVAL;
	}

	*fit = 1;

	return 1;
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = sensor_rv3032_decoder_get_frame_count,
	.get_size_info = sensor_rv3032_decoder_get_size_info,
	.decode = sensor_rv3032_decoder_decode,
};

int sensor_rv3032_get_decoder(const struct device* dev, const struct sensor_decoder_api** decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();

	return 0;
}

static int sensor_rv3032_init(const struct device* dev)
{
	const struct sensor_rv3032_config* config = dev->config;

	if (!device_is_ready(config->mfd)) {
		return -ENODEV;
	}

	return 0;
}

static DEVICE_API(sensor, driver_api) = {
	.sample_fetch = NULL, //sensor_rv3032_sample_fetch,
	.channel_get = NULL, // sensor_rv3032_channel_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = sensor_rv3032_submit,
	.get_decoder = sensor_rv3032_get_decoder,
#endif
};

#define SENSOR_RV3032_DEFINE(inst)                              \
    static const struct sensor_rv3032_config config_##inst = {  \
        .mfd = DEVICE_DT_GET(DT_INST_PARENT(inst)),             \
    };                                                          \
    static struct sensor_rv3032_data data_##inst;               \
    SENSOR_DEVICE_DT_INST_DEFINE(inst,                          \
        sensor_rv3032_init,                                     \
        NULL,                                                   \
        &data_##inst,                                           \
        &config_##inst,                                         \
        POST_KERNEL,                                            \
        CONFIG_SENSOR_RV3032_INIT_PRIORITY,                     \
        &driver_api                                             \
    );

DT_INST_FOREACH_STATUS_OKAY(SENSOR_RV3032_DEFINE)