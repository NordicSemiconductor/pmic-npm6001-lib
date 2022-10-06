/*
 * Copyright (c) 2022, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _LIB_NPM6001_H_
#define _LIB_NPM6001_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file
 * @defgroup lib_npm6001 nPM6001
 * @{
 * @brief nPM6001 library/driver.
 *
 * @details This library contains logic for control and event processing
 *          of the nPM6001 power management IC. Hardware specific functions, two-wire (TWI)
 *          communication and GPIO handling, are split into function pointers provided by
 *          higher layers.
 *
 *          To use this library on a new hardware platform, the platform-specific
 *          functions in @ref lib_npm6001_platform must be ported.
 *          Additionally, if interrupts are enabled, the @ref lib_npm6001_int_read function
 *          should be called when the nINT signal is asserted by the nPM6001.
 */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_LIB_NPM6001_WRITE_SWREADY
/* This define can be used to override SWREADY behavior. By default 'READY' value is written to
 * SWREADY register in @ref lib_npm6001_init, which enables usef of BUCK_MODE pins.
 */
#define CONFIG_LIB_NPM6001_WRITE_SWREADY true
#endif

#ifndef CONFIG_LIB_NPM6001_C3_WORKAROUND
/* This define can be used to override C3 workaround behavior.
 * Should be set to 'true' if nPM6001 pin C3 is connected to low instead of high level.
 */
#define CONFIG_LIB_NPM6001_C3_WORKAROUND false
#endif

/**@brief nPM6001 TWI address. */
#define LIB_NPM6001_TWI_ADDR 0x70

/**@brief BUCK0 minimum voltage [mV]. */
#define LIB_NPM6001_BUCK0_MINV 1800
/**@brief BUCK0 maximum voltage [mV]. */
#define LIB_NPM6001_BUCK0_MAXV 3300
/**@brief BUCK0 voltage selection resolution [mV]. */
#define LIB_NPM6001_BUCK0_RES 100

/**@brief BUCK1 minimum voltage [mV]. */
#define LIB_NPM6001_BUCK1_MINV 700
/**@brief BUCK1 maximum voltage [mV]. */
#define LIB_NPM6001_BUCK1_MAXV 1400
/**@brief BUCK1 voltage selection resolution [mV]. */
#define LIB_NPM6001_BUCK1_RES 50

/**@brief BUCK2 minimum voltage [mV]. */
#define LIB_NPM6001_BUCK2_MINV 1200
/**@brief BUCK2 maximum voltage [mV]. */
#define LIB_NPM6001_BUCK2_MAXV 1400
/**@brief BUCK2 voltage selection resolution [mV]. */
#define LIB_NPM6001_BUCK2_RES 50

/**@brief BUCK3 minimum voltage [mV]. */
#define LIB_NPM6001_BUCK3_MINV 500
/**@brief BUCK3 maximum voltage [mV]. */
#define LIB_NPM6001_BUCK3_MAXV 3300
/**@brief BUCK3 voltage selection resolution [mV]. */
#define LIB_NPM6001_BUCK3_RES 25

/**@brief LDO0 minimum voltage [mV]. */
#define LIB_NPM6001_LDO0_MINV 1800
/**@brief LDO0 maximum voltage [mV]. */
#define LIB_NPM6001_LDO0_MAXV 3300
/**@brief LDO0 voltage selection resolution [mV]. */
#define LIB_NPM6001_LDO0_RES 300

/**@brief LDO1 minimum voltage [mV]. */
#define LIB_NPM6001_LDO1_MINV 1800
/**@brief LDO1 maximum voltage [mV]. */
#define LIB_NPM6001_LDO1_MAXV 1800

/**
 * @brief List of register names (contents of nPM6001 NRF_DIGITAL_Type struct).
 */
#define LIB_NPM6001_REGISTER_NAME_LIST \
	X(SWREADY)\
	X(TASKS_START_BUCK3)\
	X(TASKS_START_LDO0)\
	X(TASKS_START_LDO1)\
	X(TASKS_START_THWARN)\
	X(TASKS_START_TH_SHUTDN)\
	X(TASKS_STOP_BUCK3)\
	X(TASKS_STOP_LDO0)\
	X(TASKS_STOP_LDO1)\
	X(TASKS_STOP_THWARN)\
	X(TASKS_STOP_THSHUTDN)\
	X(TASKS_UPDATE_VOUTPWM)\
	X(EVENTS_THWARN)\
	X(EVENTS_BUCK0OC)\
	X(EVENTS_BUCK1OC)\
	X(EVENTS_BUCK2OC)\
	X(EVENTS_BUCK3OC)\
	X(INTEN0)\
	X(INTENSET0)\
	X(INTENCLR0)\
	X(INTPEND0)\
	X(BUCK0VOUTULP)\
	X(BUCK0VOUTPWM)\
	X(BUCK1VOUTULP)\
	X(BUCK1VOUTPWM)\
	X(BUCK2VOUTULP)\
	X(BUCK2VOUTPWM)\
	X(BUCK3SELDAC)\
	X(BUCK3VOUT)\
	X(LDO0VOUT)\
	X(BUCK0CONFPWMMODE)\
	X(BUCK1CONFPWMMODE)\
	X(BUCK2CONFPWMMODE)\
	X(BUCK3CONFPWMMODE)\
	X(BUCKMODEPADCONF)\
	X(THDYNPOWERUP)\
	X(PADDRIVESTRENGTH)\
	X(WDARMEDVALUE)\
	X(WDARMEDSTROBE)\
	X(WDTRIGGERVALUE0)\
	X(WDTRIGGERVALUE1)\
	X(WDTRIGGERVALUE2)\
	X(WDDATASTROBE)\
	X(WDPWRUPVALUE)\
	X(WDPWRUPSTROBE)\
	X(WDKICK)\
	X(WDREQPOWERDOWN)\
	X(GPIOOUTSET)\
	X(GPIOOUTCLR)\
	X(GPIOIN)\
	X(GPIO0CONF)\
	X(GPIO1CONF)\
	X(GPIO2CONF)\
	X(LDO0CTRL)\
	X(LDO1CTRL)\
	X(OVERRIDEPWRUPBUCK)

/**
 * @brief nPM6001 regulators.
 */
enum lib_npm6001_vreg {
	LIB_NPM6001_BUCK0,
	LIB_NPM6001_BUCK1,
	LIB_NPM6001_BUCK2,
	LIB_NPM6001_BUCK3,
	LIB_NPM6001_LDO0,
	LIB_NPM6001_LDO1,
};

/**
 * @brief nPM6001 regulator mode.
 */
enum lib_npm6001_vreg_mode {
	LIB_NPM6001_MODE_ULP,
	LIB_NPM6001_MODE_PWM,
};

/**
 * @brief nPM6001 interrupt type.
 */
enum lib_npm6001_int {
	LIB_NPM6001_INT_THERMAL_WARNING,
	LIB_NPM6001_INT_BUCK0_OVERCURRENT,
	LIB_NPM6001_INT_BUCK1_OVERCURRENT,
	LIB_NPM6001_INT_BUCK2_OVERCURRENT,
	LIB_NPM6001_INT_BUCK3_OVERCURRENT,
};

/**
 * @brief nPM6001 regulator mode pins.
 */
enum lib_npm6001_mode_pin {
	LIB_NPM6001_BUCK_PIN_MODE0,
	LIB_NPM6001_BUCK_PIN_MODE1,
	LIB_NPM6001_BUCK_PIN_MODE2,
};

/**
 * @brief nPM6001 thermal sensor.
 */
enum lib_npm6001_thermal_sensor {
	LIB_NPM6001_THERMAL_SENSOR_WARNING,
	LIB_NPM6001_THERMAL_SENSOR_SHUTDOWN,
};

/**
 * @brief nPM6001 BUCK mode pin configuration.
 */
struct lib_npm6001_mode_pin_cfg {
	enum {
		LIB_NPM6001_MODE_PIN_CFG_PAD_TYPE_SCHMITT,
		LIB_NPM6001_MODE_PIN_CFG_PAD_TYPE_CMOS,
	} pad_type;

	enum {
		LIB_NPM6001_MODE_PIN_CFG_PULLDOWN_ENABLED,
		LIB_NPM6001_MODE_PIN_CFG_PULLDOWN_DISABLED,
	} pulldown;
};

/**
 * @brief nPM6001 general purpose I/O (GPIO).
 */
enum lib_npm6001_gpio {
	LIB_NPM6001_GPIO0,
	LIB_NPM6001_GPIO1,
	LIB_NPM6001_GPIO2,
};

/**
 * @brief nPM6001 general purpose I/O (GPIO) configuration.
 */
struct lib_npm6001_gpio_cfg {
	enum {
		LIB_NPM6001_GPIO_CFG_DIRECTION_INPUT,
		LIB_NPM6001_GPIO_CFG_DIRECTION_OUTPUT,
	} direction;

	enum {
		LIB_NPM6001_GPIO_CFG_INPUT_ENABLED,
		LIB_NPM6001_GPIO_CFG_INPUT_DISABLED,
	} input;

	enum {
		LIB_NPM6001_GPIO_CFG_PULLDOWN_ENABLED,
		LIB_NPM6001_GPIO_CFG_PULLDOWN_DISABLED,
	} pulldown;

	enum {
		LIB_NPM6001_GPIO_CFG_DRIVE_NORMAL,
		LIB_NPM6001_GPIO_CFG_DRIVE_HIGH,
	} drive;

	enum {
		LIB_NPM6001_GPIO_CFG_SENSE_LOW,
		LIB_NPM6001_GPIO_CFG_SENSE_HIGH,
	} sense;
};

/**
 * @brief Platform-specific functions for TWI and GPIO control.
 */
struct lib_npm6001_platform {
	/**
	 * @brief Initialize platform-specifics. Can be NULL if not needed.
	 *
	 * @return 0 if successful. Otherwise an error code.
	 */
	int (*lib_npm6001_platform_init)(void);

	/**
	 * @brief Read bytes from register.
	 *
	 * @param[out] buf Buffer to hold read values.
	 * @param[in] len Number of bytes to read.
	 * @param[in] reg_addr Register address.
	 *
	 * @return 0 if successful. Otherwise an error code.
	 */
	int (*lib_npm6001_twi_read)(uint8_t *buf, uint8_t len, uint8_t reg_addr);

	/**
	 * @brief Write bytes to register.
	 *
	 * @param[in] buf Values to write.
	 * @param[in] len Number of bytes to write.
	 * @param[in] reg_addr Register address.
	 *
	 * @return 0 if successful. Otherwise an error code.
	 */
	int (*lib_npm6001_twi_write)(const uint8_t *buf, uint8_t len, uint8_t reg_addr);
};

/**
 * @brief Initialize nPM6001 sample driver.
 *
 * @param[in] hw_funcs Platform-specific functions.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_init(const struct lib_npm6001_platform *hw_funcs);

/**
 * @brief Read nPM6001 interrupt source.
 *
 * @details Should be called while N_INT pin is low, as interrupt sources are cleared one at a time.
 *
 * @param[out] interrupt Interrupt source.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_int_read(enum lib_npm6001_int *interrupt);

/**
 * @brief Enable interrupt.
 *
 * @param[in] interrupt Interrupt to enable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_int_enable(enum lib_npm6001_int interrupt);

/**
 * @brief Disable interrupt.
 *
 * @param[in] interrupt Interrupt to disable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_int_disable(enum lib_npm6001_int interrupt);

/**
 * @brief Enable voltage regulator.
 *
 * @param[in] regulator Regulator to enable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_vreg_enable(enum lib_npm6001_vreg regulator);

/**
 * @brief Disable voltage regulator.
 *
 * @note Regulators BUCK0, BUCK1, and BUCK2 are always on.
 *
 * @param[in] regulator Regulator to disable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_vreg_disable(enum lib_npm6001_vreg regulator);

/**
 * @brief Set mode for BUCK regulator.
 *
 * @param[in] regulator Regulator to adjust.
 * @param[in] mode BUCK mode to set.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_vreg_buck_mode_set(enum lib_npm6001_vreg regulator,
				   enum lib_npm6001_vreg_mode mode);

/**
 * @brief Enable mode pin control for BUCK regulator.
 *
 * @details Mode pins can be used to force PWM mode for one or more of the BUCK regulators.
 *
 * @param[in] regulator Regulator to enable pin mode control for.
 * @param[in] pin Mode pin.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_vreg_mode_pin_enable(enum lib_npm6001_vreg regulator,
				     enum lib_npm6001_mode_pin pin);

/**
 * @brief Configure mode pin.
 *
 * @param[in] pin Mode pin.
 * @param[in] cfg Mode pin configuration.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_mode_pin_cfg(enum lib_npm6001_mode_pin pin,
			     const struct lib_npm6001_mode_pin_cfg *cfg);

/**
 * @brief Set regulator voltage.
 *
 * @details An error is returned if target voltage is outside of allowed range.
 *          Within the allowed range, the target value is rounded to the closest configurable value.
 *
 * @param[in] regulator Regulator to adjust.
 * @param[in] voltage Desired voltage in mV.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_vreg_voltage_set(enum lib_npm6001_vreg regulator, uint16_t voltage);

/**
 * @brief Enable thermal sensor.
 *
 * @param[in] sensor Sensor function to enable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_thermal_sensor_enable(enum lib_npm6001_thermal_sensor sensor);

/**
 * @brief Disable thermal sensor.
 *
 * @param[in] sensor Sensor function to disable.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_thermal_sensor_disable(enum lib_npm6001_thermal_sensor sensor);

/**
 * @brief Enable thermal sensor dynamic powerup.
 *
 * @details Power on thermal sensor dynamically function when designated BUCK regulator(s)
 *          enter PWM mode.
 *
 * @param[in] sensor Sensor function to enable dynamic powerup for.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_thermal_sensor_dyn_pwrup_enable(enum lib_npm6001_thermal_sensor sensor);

/**
 * @brief Set thermal sensor dynamic powerup trigger.
 *
 * @param[in] vreg BUCK regulator(s) that should trigger dynamic powerup.
 * @param[in] len Number of BUCK regulators.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_thermal_sensor_dyn_pwrup_trig(const enum lib_npm6001_vreg *vreg, uint8_t len);

/**
 * @brief Disable thermal sensor dynamic powerup.
 *
 * @param[in] sensor Sensor function to disable dynamic powerup for.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_thermal_sensor_dyn_pwrup_disable(enum lib_npm6001_thermal_sensor sensor);

/**
 * @brief Configure nPM6001 general purpose I/O.
 *
 * @param[in] pin GPIO pin to configure.
 * @param[in] cfg Configuration parameters.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_gpio_cfg(enum lib_npm6001_gpio pin, const struct lib_npm6001_gpio_cfg *cfg);

/**
 * @brief Set general purpose I/O (high level).
 *
 * @param[in] pin GPIO pin to set.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_gpio_set(enum lib_npm6001_gpio pin);

/**
 * @brief Clear general purpose I/O (low level).
 *
 * @param[in] pin GPIO pin to clear.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_gpio_clr(enum lib_npm6001_gpio pin);

/**
 * @brief Read general purpose I/O level.
 *
 * @param[in] pin GPIO pin to read.
 * @param[out] set True if level is high.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_gpio_read(enum lib_npm6001_gpio pin, bool *set);

/**
 * @brief Enable watchdog.
 *
 * @details The watchdog uses a 24-bit timer to generate a system reset when the desired
 *          timer value is reached. Once started, it must be kicked/reset before
 *          the timeout expires, otherwise a reset is triggered.
 *          The timer value is in 4 second ticks.
 *
 * @param[in] timer_val 24-bit watchdog timer value [4s units].
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_watchdog_enable(uint32_t timer_val);

/**
 * @brief Disable watchdog.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_watchdog_disable(void);

/**
 * @brief Kick (reset) the watchdog counter.
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_watchdog_kick(void);

/**
 * @brief Enter hibernation.
 *
 * @details In hibernation mode, all functionality except the wakeup timer is disabled.
 *          Once the timeout expires, the device wakes up again.
 *          The timer value is in 4 second ticks.
 *
 * @param[in] timer_val 24-bit hibernate timer value [4s units].
 *
 * @return 0 if successful. Otherwise a negative error code.
 */
int lib_npm6001_hibernate(uint32_t timer_val);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* _LIB_NPM6001_H_ */
