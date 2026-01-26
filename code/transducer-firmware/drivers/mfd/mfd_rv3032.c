#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/mfd/rv3032.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT microcrystal_rv3032_mfd

LOG_MODULE_REGISTER(mfd_rv3032, CONFIG_MFD_LOG_LEVEL);

int mfd_rv3032_i2c_get_registers(const struct device *dev, uint8_t addr, 
                                 uint8_t *buf, const size_t buf_size) {
	struct mfd_rv3032_data *data = dev->data;
	const struct mfd_rv3032_config *config = dev->config;

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_burst_read_dt(&config->i2c_bus, addr, buf, buf_size);
    if (err) {
		LOG_ERR("failed to read reg addr 0x%02x, len %d (err %d)", addr, buf_size, err);
		return err;
	}

	k_sem_give(&data->lock);

	return err;
}

int mfd_rv3032_i2c_set_registers(const struct device *dev, uint8_t addr, 
                                 const uint8_t *buf, const size_t buf_size) {
	struct mfd_rv3032_data *data = dev->data;
	const struct mfd_rv3032_config *config = dev->config;

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_burst_write_dt(&config->i2c_bus, addr, buf, buf_size);
    if (err) {
		LOG_ERR("failed to write reg addr 0x%02x, len %d (err %d)", addr, buf_size, err);
		return err;
	}

	k_sem_give(&data->lock);

	return err;
}

int mfd_rv3032_i2c_update_register(const struct device* dev, uint8_t addr,
                                   uint8_t mask, uint8_t val) {
    struct mfd_rv3032_data *data = dev->data;
	const struct mfd_rv3032_config *config = dev->config;

	(void)k_sem_take(&data->lock, K_FOREVER);
	int err = i2c_reg_update_byte_dt(&config->i2c_bus, addr, mask, val);
	if (err) {
		LOG_ERR("failed to update reg addr 0x%02x, mask 0x%02x, val 0x%02x (err %d)", 
                addr, mask, val, err);
		return err;
	}

    k_sem_give(&data->lock);

	return 0;
}

#if RV3032_INTERRUPTS

// Interrupt handler for sub drivers. E.g. if RTC alarms are enabled, it will
// add the RTC interrupt callback to the work queue. 
static void mfd_rv3032_int_handler(const struct device* port, struct gpio_callback* cb,
			                       gpio_port_pins_t pins) {
	struct mfd_rv3032_data* data = CONTAINER_OF(cb, struct mfd_rv3032_data, int_callback);

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

#ifdef COUNTER_RV3032_INTERRUPTS
    __ASSERT(data->work_counter != NULL, "data->work_counter is NULL");
    k_work_submit(data->work_counter);
#endif /* COUNTER_RV3032_INTERRUPTS */

#ifdef RTC_RV3032_INTERRUPTS 
    __ASSERT(data->work_rtc != NULL, "data->work_rtc is NULL");
	k_work_submit(data->work_rtc);
#endif /* RTC_RV3032_INTERRUPTS */

// ifdef CONFIG_RTC_UPDATE .....
}

#endif // RV3032_INTERRUPTS

static int mfd_rv3032_init(const struct device* dev) {
    int err;
    uint8_t status;
    uint8_t regs[3];

	const struct mfd_rv3032_config *config = dev->config;
	struct mfd_rv3032_data *data = dev->data;

	k_sem_init(&data->lock, 1, 1);
	if (!i2c_is_ready_dt(&(config->i2c_bus))) {
		LOG_ERR("I2C bus not ready.");
		return -ENODEV;
	}

#if RV3032_INTERRUPTS
	if (config->gpio_int.port != NULL) {
		if (!gpio_is_ready_dt(&config->gpio_int)) {
			LOG_ERR("GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&config->gpio_int, GPIO_INPUT);
		if (err) {
			LOG_ERR("failed to configure GPIO (err %d)", err);
			return -ENODEV;
		}

		err = gpio_pin_interrupt_configure_dt(&config->gpio_int, GPIO_INT_EDGE_TO_ACTIVE);
		if (err) {
			LOG_ERR("failed to enable GPIO interrupt (err %d)", err);
			return err;
		}

		gpio_init_callback(&data->int_callback, mfd_rv3032_int_handler,
				           BIT(config->gpio_int.pin));

		err = gpio_add_callback_dt(&config->gpio_int, &data->int_callback);
		if (err) {
			LOG_ERR("failed to add GPIO callback (err %d)", err);
			return -ENODEV;
		}

		data->dev = dev;
	}
#endif /* RV3032_INTERRUPTS */

    // If power was lost, clear the flag in the status register. Otherwise, put
    // device status and control registers into known state.
    // Alarm interrupts will be enabled by rtc_rv3032_alarm_set_callback if
    // called.
    err = mfd_rv3032_i2c_get_registers(dev, RV3032_REG_STATUS, &status, sizeof(status));
	if (err) {
		return -ENODEV;
	}
    if (status & RV3032_MASK_STATUS_PORF) {
        // Control registers already reset after power lost
        
        LOG_DBG("device lost power, clock time should be reset");

        err = mfd_rv3032_i2c_update_register(dev, RV3032_REG_STATUS, RV3032_MASK_STATUS_PORF, 0);
        if (err) {
            return -ENODEV;
        }
    } else {
        regs[0] = 0;
        regs[1] = 0;
        regs[2] = 0;
        err = mfd_rv3032_i2c_set_registers(dev, RV3032_REG_CONTROL1, regs, sizeof(regs));
        if (err) {
            return -ENODEV;
        }
        status = 0;
        err = mfd_rv3032_i2c_set_registers(dev, RV3032_REG_STATUS, &status, sizeof(status));
        if (err) {
            return -ENODEV;
        }
    }

    // Not worried about passwords
    // Not worried about EEPROM backup stuff
    // Not worried about EEPROM offset
    // Not worried about EEPROM configuring CLKOUT
    // Not worried about EEPROM temperature references (shouldn't touch anyways
    // probably)
    // Not worried about EEPROM passwords

    // Configure trickle charger and backup switchover mode. Also clear NCLKE.
    err = mfd_rv3032_i2c_update_register(dev, RV3032_REG_EEPROM_PMU, 
                                         GENMASK(7, 0), config->backup);
    if (err) {
		return -ENODEV;
	}

    // Disable alarms. If CONFIG_RTC_ALARM is enabled, alarms will be enabled
    // when one is set through the RTC API.
    err = mfd_rv3032_i2c_get_registers(dev, RV3032_REG_ALARM_TIME_MINUTES, regs, sizeof(regs));
	if (err) {
		return -ENODEV;
	}
	regs[0] |= RV3032_MASK_ALARM_TIME_MINUTES_AE_M;
	regs[1] |= RV3032_MASK_ALARM_TIME_HOURS_AE_H;
	regs[2] |= RV3032_MASK_ALARM_TIME_DATE_AE_D;
	err = mfd_rv3032_i2c_set_registers(dev, RV3032_REG_ALARM_TIME_MINUTES, regs, sizeof(regs));
	if (err) {
		return -ENODEV;
	}

    // TODO: Reset flags in TEMPERATURE LSB register 

	return 0;
}

#define MFD_RV3032_BSM_FROM_DT_INST(inst)                                           \
	UTIL_CAT(MFD_RV3032_PMU_BSM_, DT_INST_STRING_UPPER_TOKEN(inst, backup_switch_mode))

#define MFD_RV3032_TCM_FROM_DT_INST(inst)                                                       \
    UTIL_CAT(MFD_RV3032_PMU_TCM_, DT_INST_STRING_UPPER_TOKEN_OR(inst, trickle_charger_mode, OFF))

#define MFD_RV3032_BACKUP_FROM_DT_INST(inst)                                                        \
	((FIELD_PREP(RV3032_MASK_PMU_BSM, MFD_RV3032_BSM_FROM_DT_INST(inst))) |                         \
	 (FIELD_PREP(RV3032_MASK_PMU_TCR, DT_INST_ENUM_IDX_OR(inst, trickle_resistor_ohms, 0))) |       \
	 (FIELD_PREP(RV3032_MASK_PMU_TCM, MFD_RV3032_TCM_FROM_DT_INST(inst))))

#define MFD_RV3032_DEFINE(inst)                                                 \
	static const struct mfd_rv3032_config config##inst = {                      \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                  \
        .backup = MFD_RV3032_BACKUP_FROM_DT_INST(inst),                         \
        IF_ENABLED(RV3032_INTERRUPTS,                                           \
			   (.gpio_int = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0})))    \
    };                                                                          \
	static struct mfd_rv3032_data data##inst = {                                \
        IF_ENABLED(RTC_RV3032_INTERRUPTS,                                       \
            (.work_rtc = NULL,))                                                \
        IF_ENABLED(COUNTER_RV3032_INTERRUPTS,                                   \
            (.work_counter = NULL,))                                            \
    };                                                                          \
	DEVICE_DT_INST_DEFINE(inst,                                                 \
        &mfd_rv3032_init,                                                       \
        NULL,                                                                   \
        &data##inst,                                                            \
        &config##inst,                                                          \
        POST_KERNEL,                                                            \
        CONFIG_MFD_INIT_PRIORITY,                                               \
        NULL);                                                                                  

DT_INST_FOREACH_STATUS_OKAY(MFD_RV3032_DEFINE)