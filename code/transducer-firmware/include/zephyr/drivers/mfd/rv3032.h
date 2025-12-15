#ifndef ZEPHYR_INCLUDE_DRIVERS_MFD_RV3032_H_
#define ZEPHYR_INCLUDE_DRIVERS_MFD_RV3032_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/kernel.h>

/*
 * REGISTERS
 */

/* RAM */
#define RV3032_REG_TIME_100TH_SECONDS   0x00
#define RV3032_REG_TIME_SECONDS         0x01
#define RV3032_REG_TIME_MINUTES         0x02
#define RV3032_REG_TIME_HOURS           0x03
#define RV3032_REG_TIME_WEEKDAY         0x04
#define RV3032_REG_TIME_DATE            0x05
#define RV3032_REG_TIME_MONTH           0x06
#define RV3032_REG_TIME_YEAR            0x07

#define RV3032_REG_ALARM_TIME_MINUTES   0x08
#define RV3032_REG_ALARM_TIME_HOURS     0x09
#define RV3032_REG_ALARM_TIME_DATE      0x0A

#define RV3032_REG_TIMER_VALUE0         0x0B
#define RV3032_REG_TIMER_VALUE1         0x0C

#define RV3032_REG_STATUS               0x0D

#define RV3032_REG_TEMPERATURE_LSB      0x0E
#define RV3032_REG_TEMPERATURE_MSB      0x0F

#define RV3032_REG_CONTROL1             0x10
#define RV3032_REG_CONTROL2             0x11
#define RV3032_REG_CONTROL3             0x12

#define RV3032_REG_TIME_STAMP_CONTROL   0x13

#define RV3032_REG_CLOCK_INTERRUPT_MASK 0x14

#define RV3032_REG_EVI_CONTROL          0x15

#define RV3032_REG_TLOW_THRESHOLD       0x16
#define RV3032_REG_THIGH_THRESHOLD      0x17

#define RV3032_REG_TS_TLOW_COUNT        0x18
#define RV3032_REG_TS_TLOW_SECONDS      0x19
#define RV3032_REG_TS_TLOW_MINUTES      0x1A
#define RV3032_REG_TS_TLOW_HOURS        0x1B
#define RV3032_REG_TS_TLOW_DATE         0x1C
#define RV3032_REG_TS_TLOW_MONTH        0x1D
#define RV3032_REG_TS_TLOW_YEAR         0x1E

#define RV3032_REG_TS_THIGH_COUNT       0x1F
#define RV3032_REG_TS_THIGH_SECONDS     0x20
#define RV3032_REG_TS_THIGH_MINUTES     0x21
#define RV3032_REG_TS_THIGH_HOURS       0x22
#define RV3032_REG_TS_THIGH_DATE        0x23
#define RV3032_REG_TS_THIGH_MONTH       0x24
#define RV3032_REG_TS_THIGH_YEAR        0x25

#define RV3032_REG_TS_EVI_COUNT         0x26
#define RV3032_REG_TS_EVI_100TH_SECONDS 0x27
#define RV3032_REG_TS_EVI_SECONDS       0x28
#define RV3032_REG_TS_EVI_MINUTES       0x29
#define RV3032_REG_TS_EVI_HOURS         0x2A
#define RV3032_REG_TS_EVI_DATE          0x2B
#define RV3032_REG_TS_EVI_MONTH         0x2C
#define RV3032_REG_TS_EVI_YEAR          0x2D

#define RV3032_REG_PASSWORD0            0x39
#define RV3032_REG_PASSWORD1            0x3A
#define RV3032_REG_PASSWORD2            0x3B
#define RV3032_REG_PASSWORD3            0x3C

#define RV3032_REG_EE_ADDRESS           0x3D
#define RV3032_REG_EE_DATA              0x3E
#define RV3032_REG_EE_COMMAND           0x3F

#define RV3032_REG_USER_RAM_START       0x40
#define RV3032_USER_RAM_SIZE            16  // bytes

/* Configuration EEPROM */
#define RV3032_REG_EEPROM_PMU           0xC0
#define RV3032_REG_EEPROM_OFFSET        0xC1
#define RV3032_REG_EEPROM_CLKOUT1       0xC2
#define RV3032_REG_EEPROM_CLKOUT2       0xC3
#define RV3032_REG_EEPROM_TREFERENCE0   0xC4
#define RV3032_REG_EEPROM_TREFERENCE1   0xC5
#define RV3032_REG_EEPROM_PASSWORD0     0xC6
#define RV3032_REG_EEPROM_PASSWORD1     0xC7
#define RV3032_REG_EEPROM_PASSWORD2     0xC8
#define RV3032_REG_EEPROM_PASSWORD3     0xC9
#define RV3032_REG_EEPROM_PW_ENABLE     0xCA

/* User EEPROM */
#define RV3032_REG_USER_EEPROM_START    0xCB
#define RV3032_USER_EEPROM_SIZE         32  // bytes
 

/*
 * BITMASKS
 */

/* Time */
#define RV3032_MASK_TIME_100TH_SECONDS  GENMASK(7, 0)
#define RV3032_MASK_TIME_SECONDS        GENMASK(6, 0)
#define RV3032_MASK_TIME_MINUTES        GENMASK(6, 0)
#define RV3032_MASK_TIME_HOURS          GENMASK(5, 0)
#define RV3032_MASK_TIME_WEEKDAY        GENMASK(2, 0)
#define RV3032_MASK_TIME_DATE           GENMASK(5, 0)
#define RV3032_MASK_TIME_MONTH          GENMASK(4, 0)
#define RV3032_MASK_TIME_YEAR           GENMASK(7, 0)

#define RV3032_MASK_ALARM_TIME_MINUTES_AE_M BIT(7)
#define RV3032_MASK_ALARM_TIME_MINUTES      GENMASK(6, 0)
#define RV3032_MASK_ALARM_TIME_HOURS_AE_H   BIT(7)
#define RV3032_MASK_ALARM_TIME_HOURS        GENMASK(5, 0)
#define RV3032_MASK_ALARM_TIME_DATE_AE_D    BIT(7)
#define RV3032_MASK_ALARM_TIME_DATE         GENMASK(5, 0)

#define RV3032_MASK_PERIODIC_COUNTDOWN_TIMER_VALUE0 GENMASK(7, 0)
#define RV3032_MASK_PERIODIC_COUNTDOWN_TIMER_VALUE1 GENMASK(3, 0)

#define RV3032_MASK_STATUS_THF          BIT(7)
#define RV3032_MASK_STATUS_TLF          BIT(6)
#define RV3032_MASK_STATUS_UF           BIT(5)
#define RV3032_MASK_STATUS_TF           BIT(4)
#define RV3032_MASK_STATUS_AF           BIT(3)
#define RV3032_MASK_STATUS_EVF          BIT(2)    
#define RV3032_MASK_STATUS_PORF         BIT(1)    
#define RV3032_MASK_STATUS_VLF          BIT(0)  

#define RV3032_MASK_TEMPERATURE_LSB_TEMP    GENMASK(7, 4)

#define RV3032_MASK_CONTROL1_GP0        BIT(5)
#define RV3032_MASK_CONTROL1_USEL       BIT(4)
#define RV3032_MASK_CONTROL1_TE         BIT(3)
#define RV3032_MASK_CONTROL1_EERD       BIT(2)
#define RV3032_MASK_CONTROL1_TD         GENMASK(1, 0)

#define RV3032_MASK_CONTROL2_CLKIE      BIT(6)
#define RV3032_MASK_CONTROL2_UIE        BIT(5)
#define RV3032_MASK_CONTROL2_TIE        BIT(4)
#define RV3032_MASK_CONTROL2_AIE        BIT(3)
#define RV3032_MASK_CONTROL2_EIE        BIT(2)
#define RV3032_MASK_CONTROL2_GP1        BIT(1)
#define RV3032_MASK_CONTROL2_STOP       BIT(0)

#define RV3032_MASK_CONTROL3_BSIE       BIT(4)
#define RV3032_MASK_CONTROL3_THE        BIT(3)
#define RV3032_MASK_CONTROL3_TLE        BIT(2)
#define RV3032_MASK_CONTROL3_THIE       BIT(1)
#define RV3032_MASK_CONTROL3_TLIE       BIT(0)

#define RV3032_MASK_PMU_BSM             GENMASK(5, 4)
#define RV3032_MASK_PMU_TCR             GENMASK(3, 2)
#define RV3032_MASK_PMU_TCM             GENMASK(1, 0)


/* 
 * CONFIGURATIONS 
 */

#define RV3032_PMU_BSM_DISABLED         0b00
#define RV3032_PMU_BSM_DIRECT           0b01
#define RV3032_PMU_BSM_LEVEL            0b10

#define RV3032_PMU_TCM_OFF              0b00
#define RV3032_PMU_TCM_1_75V            0b01
#define RV3032_PMU_TCM_3_00V            0b10
#define RV3032_PMU_TCM_4_50V            0b11


#define RV3032_TEMPERATURE_INT_BITS     8
#define RV3032_TEMPERATURE_FRAC_BITS    4

#define RV3032_YEAR_OFFSET              100
#define RV3032_MONTH_OFFSET             1
#define RV3032_RTC_TIME_MASK                                                                   \
	(RTC_ALARM_TIME_MASK_SECOND | RTC_ALARM_TIME_MASK_MINUTE | RTC_ALARM_TIME_MASK_HOUR |      \
	 RTC_ALARM_TIME_MASK_MONTH | RTC_ALARM_TIME_MASK_MONTHDAY | RTC_ALARM_TIME_MASK_YEAR |     \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

/* Helper macro to guard int-gpios related code */
#if DT_ANY_COMPAT_HAS_PROP_STATUS_OKAY(microcrystal_rv3032_mfd, int_gpios) &&   \
	(defined(CONFIG_RTC_ALARM) || defined(CONFIG_RTC_UPDATE))
    //TODO: Add CONFIG_COUNTDOWN_TIMER or whatever it is
#define RV3032_INT_GPIOS_IN_USE 1
#endif

/**
 * @brief Get specified number of registers from an I2C device
 * starting at the given register address.
 *
 * @param dev rv3032 mfd device
 * @param addr The register address to start at.
 * @param buf The buffer array pointer to store results in.
 * @param buf_size The number of register values to read.
 * @retval 0 on success
 * @retval -errno in case of any bus error
 */
int mfd_rv3032_i2c_get_registers(const struct device *dev, uint8_t addr, 
                                 uint8_t *buf, const size_t buf_size);

/**
 * @brief Set specified number of registers on an I2C device 
 * starting at the given register address.
 *
 * @param dev rv3032 mfd device
 * @param addr The register address to start at.
 * @param buf The buffer array pointer to write from.
 * @param buf_size The number of registers to write to.
 * @retval 0 on success
 * @retval -errno in case of any bus error
 */
int mfd_rv3032_i2c_set_registers(const struct device *dev, uint8_t addr, 
                                 const uint8_t *buf, const size_t buf_size);

/**
 * @brief Update a register on an I2C device at the given register address.
 *
 * @param dev rv3032 mfd device
 * @param start_reg The register address to update.
 * @param mask Bitmask of bits to set/unset based on val.
 * @param val New value to write for masked bits.
 * @retval 0 on success
 * @retval -errno in case of any bus error
 */
int mfd_rv3032_i2c_update_register(const struct device* dev, uint8_t addr,
                                   uint8_t mask, uint8_t val);

struct mfd_rv3032_config {
    struct i2c_dt_spec i2c_bus;
    const uint8_t backup;
#if RV3032_INT_GPIOS_IN_USE
    struct gpio_dt_spec gpio_int;
#endif /* RV3032_INT_GPIOS_IN_USE */
};

struct mfd_rv3032_data {
	struct k_sem lock;
#if RV3032_INT_GPIOS_IN_USE
	const struct device *dev;
    struct gpio_callback int_callback;
#ifdef CONFIG_RTC_ALARM
    struct k_work* work_rtc; // To be set by child driver
#endif /* CONFIG_RTC_ALARM */
// #ifdef CONFIG_COUNTER???
#endif /* RV3032_INT_GPIOS_IN_USE */
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_MFD_RV3032_H_ */