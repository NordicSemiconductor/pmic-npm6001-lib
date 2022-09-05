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

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#include <hal/nrf_rtc.h>

#include <ztest.h>

#include <lib_npm6001.h>

#define VREG_MEAS_TOLERANCE 50 /* [mV] */

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

#define REGULATOR_LIST \
	X(BUCK0) \
	X(BUCK1) \
	X(BUCK2) \
	X(BUCK3) \
	X(LDO0) \
	X(LDO1)

enum vreg_voltage_idx {
#define X(_vreg) VREG_ ## _vreg ## _VOLTAGE,
	REGULATOR_LIST
#undef X
};

const static char *vreg_names[] = {
#define X(_vreg) STRINGIFY(_vreg),
	REGULATOR_LIST
#undef X
};

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static const struct device *i2c_dev = DEVICE_DT_GET_ONE(nordic_nrf_twim);
static const struct gpio_dt_spec n_int = GPIO_DT_SPEC_GET(DT_ALIAS(nint), gpios);
static struct gpio_callback n_int_callback;

static int lib_npm6001_platform_init(void)
{
	if (i2c_dev == NULL) {
		return -ENODEV;
	}

	return 0;
}

static int lib_npm6001_twi_read(uint8_t *buf, uint8_t len, uint8_t reg_addr)
{
	int err;

	err = i2c_write_read(i2c_dev, LIB_NPM6001_TWI_ADDR,
		&reg_addr, sizeof(reg_addr),
		buf, len);

	if (err == 0) {
		/* Success: return number of bytes read */
		return len;
	} else {
		return -1;
	}
}

static int lib_npm6001_twi_write(const uint8_t *buf, uint8_t len, uint8_t reg_addr)
{
	struct i2c_msg msgs[] = {
		{.buf = &reg_addr, .len = sizeof(reg_addr), .flags = I2C_MSG_WRITE},
		{.buf = (uint8_t *)buf, .len = len, .flags = I2C_MSG_WRITE | I2C_MSG_STOP},
	};
	int err;

	err = i2c_transfer(i2c_dev, &msgs[0], ARRAY_SIZE(msgs), LIB_NPM6001_TWI_ADDR);
	if (err == 0) {
		/* Success: return number of bytes written */
		return len;
	} else {
		return -1;
	}
}

static void n_int_irq_handler(const struct device *port,
					struct gpio_callback *cb,
					gpio_port_pins_t pins)
{
	/* */
}

static void test_gpio_init(void)
{
	gpio_flags_t flags = n_int.dt_flags & GPIO_ACTIVE_LOW ? GPIO_PULL_UP : GPIO_PULL_DOWN;
	int err;

	err = gpio_pin_configure_dt(&n_int, GPIO_INPUT | flags);
	zassert_ok(err, "gpio_pin_configure_dt");

	gpio_init_callback(&n_int_callback, n_int_irq_handler, BIT(n_int.pin));

	err = gpio_add_callback(n_int.port, &n_int_callback);
	zassert_ok(err, "gpio_add_callback");

	err = gpio_pin_interrupt_configure_dt(&n_int, GPIO_INT_EDGE_TO_ACTIVE);
	zassert_ok(err, "gpio_pin_interrupt_configure_dt");
}

static void adc_read_all(int32_t *voltages_mv, int voltage_arr_len)
{
	int16_t buf;
	int err;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	zassert_equal(voltage_arr_len, ARRAY_SIZE(adc_channels), "Array size mismatch");

	for (int i = 0; i < ARRAY_SIZE(adc_channels); ++i) {
		err = adc_sequence_init_dt(&adc_channels[i], &sequence);
		zassert_ok(err, "adc_sequence_init_dt");

		err = adc_read(adc_channels[i].dev, &sequence);
		zassert_ok(err, "adc_read");

		voltages_mv[i] = buf;
		err = adc_raw_to_millivolts_dt(&adc_channels[i],
							&voltages_mv[i]);
		zassert_ok(err, "adc_read");
	}
}

static void adc_print_values(int32_t *voltages_mv, int voltage_arr_len)
{
	zassert_equal(voltage_arr_len, ARRAY_SIZE(vreg_names), "Array size mismatch");

	for (int i = 0; i < voltage_arr_len; ++i) {
		PRINT("%s=%d mV\n", vreg_names[i], voltages_mv[i]);
	}
}

static void test_adc_init(void)
{
	int err;

	for (int i = 0; i < ARRAY_SIZE(adc_channels); ++i) {
		zassert_true(device_is_ready(adc_channels[i].dev), "ADC not ready");

		err = adc_channel_setup_dt(&adc_channels[i]);
		zassert_ok(err, "adc_channel_setup_dt");
	}

	/* Dirty way to enable pull-down resistors on AIN.
	 * Needed to discharge the nPM6001 output capacitors in a timely manner.
	 */

	for (int i = 0; i < ARRAY_SIZE(NRF_SAADC_S->CH); ++i) {
		NRF_SAADC_S->CH[i].CONFIG &= ~SAADC_CH_CONFIG_RESP_Msk;
		NRF_SAADC_S->CH[i].CONFIG |= (SAADC_CH_CONFIG_RESP_Pulldown << SAADC_CH_CONFIG_RESP_Pos);
	}
}

static void test_lib_npm6001_init_reset(void)
{
	const struct lib_npm6001_platform ncs_hw_funcs = {
		.lib_npm6001_platform_init = lib_npm6001_platform_init,
		.lib_npm6001_twi_read = lib_npm6001_twi_read,
		.lib_npm6001_twi_write = lib_npm6001_twi_write,
	};
	int err;

	err = lib_npm6001_init(&ncs_hw_funcs);
	zassert_ok(err, "Failed to initialize lib");

	/* 4 second watchdog reset */
	err = lib_npm6001_watchdog_enable(1);
	zassert_ok(err, "lib_npm6001_watchdog_enable");

	/* Spin for 5 seconds */
	err = k_sleep(K_SECONDS(5));
	zassert_ok(err, "k_sleep");
}

static void test_lib_npm6001_init(void)
{
	const struct lib_npm6001_platform ncs_hw_funcs = {
		.lib_npm6001_platform_init = lib_npm6001_platform_init,
		.lib_npm6001_twi_read = lib_npm6001_twi_read,
		.lib_npm6001_twi_write = lib_npm6001_twi_write,
	};
	int err;

	err = lib_npm6001_init(&ncs_hw_funcs);
	zassert_ok(err, "Failed to initialize lib");
}

static void test_lib_npm6001_vreg_mode(void)
{
	int err;

	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_BUCK0, LIB_NPM6001_MODE_PWM);
	zassert_ok(err, "BUCK0 mode");
	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_BUCK1, LIB_NPM6001_MODE_PWM);
	zassert_ok(err, "BUCK1 mode");
	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_BUCK2, LIB_NPM6001_MODE_PWM);
	zassert_ok(err, "BUCK2 mode");
	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_BUCK3, LIB_NPM6001_MODE_PWM);
	zassert_ok(err, "BUCK3 mode");

	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_LDO0, LIB_NPM6001_MODE_PWM);
	zassert_equal(err, -EINVAL, "LDO0 mode");
	err = lib_npm6001_vreg_buck_mode_set(LIB_NPM6001_LDO1, LIB_NPM6001_MODE_PWM);
	zassert_equal(err, -EINVAL, "LDO1 mode");
}

static void test_lib_npm6001_vreg_on(void)
{
	int32_t voltages_mv[6];
	int err;

	err = lib_npm6001_vreg_enable(LIB_NPM6001_BUCK3);
	zassert_ok(err, "Failed to enable BUCK3");

	err = lib_npm6001_vreg_enable(LIB_NPM6001_LDO0);
	zassert_ok(err, "Failed to enable LDO0");

	err = lib_npm6001_vreg_enable(LIB_NPM6001_LDO1);
	zassert_ok(err, "Failed to enable LDO1");

	/* Delay for voltages to settle */
	k_msleep(250);
	adc_read_all(voltages_mv, ARRAY_SIZE(voltages_mv));
	adc_print_values(voltages_mv, ARRAY_SIZE(voltages_mv));

	/* Verify default voltages */
	zassert_within(voltages_mv[VREG_BUCK0_VOLTAGE], 1800, VREG_MEAS_TOLERANCE, "BUCK0 default voltage");
	zassert_within(voltages_mv[VREG_BUCK1_VOLTAGE], 800, VREG_MEAS_TOLERANCE, "BUCK1 default voltage");
	zassert_within(voltages_mv[VREG_BUCK2_VOLTAGE], 1200, VREG_MEAS_TOLERANCE, "BUCK2 default voltage");
	zassert_within(voltages_mv[VREG_BUCK3_VOLTAGE], 500, VREG_MEAS_TOLERANCE, "BUCK3 default voltage");
	zassert_within(voltages_mv[VREG_LDO0_VOLTAGE], 3000, VREG_MEAS_TOLERANCE, "LDO0 default voltage");
	zassert_within(voltages_mv[VREG_LDO1_VOLTAGE], 1800, VREG_MEAS_TOLERANCE, "LDO1 default voltage");
}

static void test_lib_npm6001_vreg_off(void)
{
	int32_t voltages[6];
	int32_t voltages_after_delay[6];
	int voltage_rate[6];
	int err;

	err = lib_npm6001_vreg_disable(LIB_NPM6001_BUCK3);
	zassert_ok(err, "Failed to disable BUCK3");

	err = lib_npm6001_vreg_disable(LIB_NPM6001_LDO0);
	zassert_ok(err, "Failed to disable LDO0");

	err = lib_npm6001_vreg_disable(LIB_NPM6001_LDO1);
	zassert_ok(err, "Failed to disable LDO1");

	err = lib_npm6001_vreg_disable(LIB_NPM6001_BUCK0);
	zassert_equal(err, -EINVAL, "BUCK0 cannot be turned off");

	err = lib_npm6001_vreg_disable(LIB_NPM6001_BUCK1);
	zassert_equal(err, -EINVAL, "BUCK1 cannot be turned off");

	err = lib_npm6001_vreg_disable(LIB_NPM6001_BUCK2);
	zassert_equal(err, -EINVAL, "BUCK2 cannot be turned off");

	/* It can take a while for voltages to discharge.
	 * Instead of waiting for several minutes,
	 * verify that the voltage is indeed dropping at a reasonable rate.
	 */

	adc_read_all(voltages, ARRAY_SIZE(voltages));

	/* Delay for voltages to drop */
	k_msleep(3000);

	adc_read_all(voltages_after_delay, ARRAY_SIZE(voltages_after_delay));

	for (int i = 0; i < ARRAY_SIZE(voltages); ++i) {
		voltage_rate[i] = (voltages_after_delay[i] - voltages[i]);
		PRINT("voltage_rate[%d]=%d\n", i, voltage_rate[i]);
	}
	
	zassert_true(voltage_rate[VREG_BUCK3_VOLTAGE] < -VREG_MEAS_TOLERANCE, "BUCK3 voltage drop rate too slow");
	zassert_true(voltage_rate[VREG_LDO0_VOLTAGE] < -VREG_MEAS_TOLERANCE, "LDO0 voltage drop rate too slow");
	zassert_true(voltage_rate[VREG_LDO1_VOLTAGE] < -VREG_MEAS_TOLERANCE, "LDO1 voltage drop rate too slow");
}

static void test_lib_npm6001_vreg_adjust(void)
{
	int32_t voltages_mv[6];
	int err;

	/* Set invalid voltages */
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK0, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK0, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK1, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK1, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK2, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK2, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK3, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK3, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_LDO0, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_LDO0, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_LDO1, 0);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_LDO1, 5000);
	zassert_equal(err, -EINVAL, "Invalid voltage selection");

	const uint16_t buck0_mv = 1900;
	const uint16_t buck1_mv = 700;
	const uint16_t buck2_mv = 1250;
	const uint16_t buck3_mv = 500;
	const uint16_t ldo0_mv = 2700;

	/* Set valid voltages */
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK0, buck0_mv);
	zassert_ok(err, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK1, buck1_mv);
	zassert_ok(err, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK2, buck2_mv);
	zassert_ok(err, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK3, buck3_mv);
	zassert_ok(err, "Invalid voltage selection");
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_LDO0, ldo0_mv);
	zassert_ok(err, "Invalid voltage selection");

	/* Delay for voltages to settle */
	k_msleep(1000);
	adc_read_all(voltages_mv, ARRAY_SIZE(voltages_mv));
	adc_print_values(voltages_mv, ARRAY_SIZE(voltages_mv));

	/* Verify voltages */
	zassert_within(voltages_mv[VREG_BUCK0_VOLTAGE], buck0_mv, VREG_MEAS_TOLERANCE, "BUCK0 voltage");
	zassert_within(voltages_mv[VREG_BUCK1_VOLTAGE], buck1_mv, VREG_MEAS_TOLERANCE, "BUCK1 voltage");
	zassert_within(voltages_mv[VREG_BUCK2_VOLTAGE], buck2_mv, VREG_MEAS_TOLERANCE, "BUCK2 voltage");
	zassert_within(voltages_mv[VREG_BUCK3_VOLTAGE], buck3_mv, VREG_MEAS_TOLERANCE, "BUCK3 voltage");
	zassert_within(voltages_mv[VREG_LDO0_VOLTAGE], ldo0_mv, VREG_MEAS_TOLERANCE, "LDO0 voltage");
}

static void test_lib_npm6001_watchdog_reset(void)
{
	const uint16_t buck0_mv = 2500;
	int32_t voltages_mv[6];
	int err;

	/* Set BUCK0 voltage to non-default value */
	err = lib_npm6001_vreg_voltage_set(LIB_NPM6001_BUCK0, buck0_mv);
	zassert_ok(err, "Invalid voltage selection");

	/* Delay for voltages to settle */
	k_msleep(500);
	adc_read_all(voltages_mv, ARRAY_SIZE(voltages_mv));

	/* Verify voltage */
	zassert_within(voltages_mv[LIB_NPM6001_BUCK0], buck0_mv, VREG_MEAS_TOLERANCE, "BUCK0 voltage");

	/* 4 second watchdog reset */
	err = lib_npm6001_watchdog_enable(1);
	zassert_ok(err, "lib_npm6001_watchdog_enable");

	/* Spin for 5 seconds */
	k_sleep(K_SECONDS(5));

	adc_read_all(voltages_mv, ARRAY_SIZE(voltages_mv));

	/* Verify BUCK0 voltage is back to default */
	zassert_within(voltages_mv[LIB_NPM6001_BUCK0], 1800, VREG_MEAS_TOLERANCE, "BUCK0 voltage");
}

void test_main(void)
{
	ztest_test_suite(test_lib_npm6001,
			ztest_unit_test(test_gpio_init),
			ztest_unit_test(test_adc_init),
			ztest_unit_test(test_lib_npm6001_init_reset),
			ztest_unit_test(test_lib_npm6001_init),
			ztest_unit_test(test_lib_npm6001_vreg_mode),
			ztest_unit_test(test_lib_npm6001_vreg_on),
			ztest_unit_test(test_lib_npm6001_vreg_adjust),
			ztest_unit_test(test_lib_npm6001_vreg_off),
			ztest_unit_test(test_lib_npm6001_watchdog_reset)
	);

	ztest_run_test_suite(test_lib_npm6001);
}
