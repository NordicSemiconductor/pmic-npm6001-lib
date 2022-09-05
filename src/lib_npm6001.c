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

#include "lib_npm6001.h"

#include "npm6001/npm6001.h"

#include <errno.h>

#ifndef ROUNDED_DIV
#define ROUNDED_DIV(a, b) (((a) + ((b) / 2)) / (b))
#endif /* ROUNDED_DIV */

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif /* ARRAY_SIZE */

#define VOLTAGE_VALID(_mv, _llim, _ulim) (_mv >= _llim && _mv <= _ulim)
#define VOLTAGE_ROUND(_mv, _res) (ROUNDED_DIV(_mv, _res) * _res)

#define NPM6001_REG_ADDR(_reg) (offsetof(NRF_DIGITAL_Type, _reg))

#define WDPWRUPVALUE_REG_VALUE                                                                     \
	((DIGITAL_WDPWRUPVALUE_OSC_ENABLE << DIGITAL_WDPWRUPVALUE_OSC_Pos) |                       \
	 (DIGITAL_WDPWRUPVALUE_COUNTER_ENABLE << DIGITAL_WDPWRUPVALUE_COUNTER_Pos) |               \
	 (DIGITAL_WDPWRUPVALUE_LS_ENABLE << DIGITAL_WDPWRUPVALUE_LS_Pos))

#define WDPWRUPSTROBE_REG_VALUE                                                                    \
	(DIGITAL_WDPWRUPSTROBE_STROBE_STROBE << DIGITAL_WDPWRUPSTROBE_STROBE_Pos)

#define WDDATASTROBE_REG_VALUE                                                                     \
	(DIGITAL_WDDATASTROBE_STROBE_STROBE << DIGITAL_WDDATASTROBE_STROBE_Pos)

#define WDARMEDVALUE_EN_REG_VALUE                                                                  \
	(DIGITAL_WDARMEDVALUE_VALUE_ENABLE << DIGITAL_WDARMEDVALUE_VALUE_Pos)

#define WDARMEDVALUE_DIS_REG_VALUE                                                                 \
	(DIGITAL_WDARMEDVALUE_VALUE_DISABLE << DIGITAL_WDARMEDVALUE_VALUE_Pos)

#define WDARMEDSTROBE_REG_VALUE                                                                    \
	(DIGITAL_WDARMEDSTROBE_STROBE_STROBE << DIGITAL_WDARMEDSTROBE_STROBE_Pos)

#define WDKICK_REG_VALUE (DIGITAL_WDKICK_KICK_KICK << DIGITAL_WDKICK_KICK_Pos)

#define WDREQPOWERDOWN_REG_VALUE                                                                   \
	(DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_REQUEST << DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Pos)

struct twi_reg_val {
	uint8_t addr;
	uint8_t val;
};

static int (*twi_read)(uint8_t *buf, uint8_t len, uint8_t reg_addr);
static int (*twi_write)(const uint8_t *buf, uint8_t len, uint8_t reg_addr);

static int twi_write_multiple(const struct twi_reg_val *buf, size_t count)
{
	int bytes_written;

	for (int i = 0; i < count; ++i) {
		bytes_written = twi_write(&buf[i].val, sizeof(buf[i].val), buf[i].addr);
		if (bytes_written != sizeof(buf[i].val)) {
			return -ENODEV;
		}
	}

	return 0;
}

static int buck12_set_state(enum lib_npm6001_vreg regulator, bool on)
{
	const uint8_t reg_addr = NPM6001_REG_ADDR(OVERRIDEPWRUPBUCK);
	uint8_t reg_val;

	int bytes_read;
	int bytes_written;

	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	switch (regulator) {
	case LIB_NPM6001_BUCK1:
		reg_val &= ~(DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Msk);
		reg_val &= ~(DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Msk);
		if (!on) {
			reg_val |= (DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_ENABLED <<
				DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Pos);
		}
		break;
	case LIB_NPM6001_BUCK2:
		reg_val &= ~(DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Msk);
		reg_val &= ~(DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Msk);
		if (!on) {
			reg_val |= (DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_ENABLED <<
				DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Pos);
		}
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written != sizeof(reg_val)) {
		return -ENODEV;
	}

	return 0;
}

static int buck012_vout_set(enum lib_npm6001_vreg regulator, uint16_t voltage)
{
	struct twi_reg_val reg_vals[5];
	uint8_t voltage_reg_val;
	int bytes_read;

	/* Regulator should be forced to PWM mode (if its not already) when updating voltage.
	 * The same voltage value should be applied to both ULP and PWM registers.
	 * This means voltage update is a TWI read, followed by 5 writes:
	 * conf register, 2 x voltage register, voltage task trigger, conf register
	 */

	reg_vals[3].addr = NPM6001_REG_ADDR(TASKS_UPDATE_VOUTPWM);

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
		reg_vals[0].addr = NPM6001_REG_ADDR(BUCK0CONFPWMMODE);
		reg_vals[1].addr = NPM6001_REG_ADDR(BUCK0VOUTPWM);
		reg_vals[2].addr = NPM6001_REG_ADDR(BUCK0VOUTULP);
		reg_vals[4].addr = NPM6001_REG_ADDR(BUCK0CONFPWMMODE);
		voltage_reg_val = ((voltage - LIB_NPM6001_BUCK0_MINV) / LIB_NPM6001_BUCK0_RES) +
				  DIGITAL_BUCK0VOUTPWM_VOLTAGE_Min;
		break;
	case LIB_NPM6001_BUCK1:
		reg_vals[0].addr = NPM6001_REG_ADDR(BUCK1CONFPWMMODE);
		reg_vals[1].addr = NPM6001_REG_ADDR(BUCK1VOUTPWM);
		reg_vals[2].addr = NPM6001_REG_ADDR(BUCK1VOUTULP);
		reg_vals[4].addr = NPM6001_REG_ADDR(BUCK1CONFPWMMODE);
		voltage_reg_val = ((voltage - LIB_NPM6001_BUCK1_MINV) / LIB_NPM6001_BUCK1_RES) +
				  DIGITAL_BUCK1VOUTPWM_VOLTAGE_Min;
		break;
	case LIB_NPM6001_BUCK2:
		reg_vals[0].addr = NPM6001_REG_ADDR(BUCK2CONFPWMMODE);
		reg_vals[1].addr = NPM6001_REG_ADDR(BUCK2VOUTPWM);
		reg_vals[2].addr = NPM6001_REG_ADDR(BUCK2VOUTULP);
		reg_vals[4].addr = NPM6001_REG_ADDR(BUCK2CONFPWMMODE);
		voltage_reg_val = ((voltage - LIB_NPM6001_BUCK2_MINV) / LIB_NPM6001_BUCK2_RES) +
				  DIGITAL_BUCK2VOUTPWM_VOLTAGE_Min;
		break;
	default:
		return -EINVAL;
	}

	bytes_read = twi_read(&reg_vals[4].val, sizeof(reg_vals[4].val), reg_vals[4].addr);
	if (bytes_read != sizeof(reg_vals[4].val)) {
		return -ENODEV;
	}

	reg_vals[0].val = (reg_vals[4].val & DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Msk) |
			  (DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_ON
			   << DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Pos);
	reg_vals[1].val = voltage_reg_val;
	reg_vals[2].val = voltage_reg_val;
	reg_vals[3].val = (DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Trigger
			   << DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Pos);

	return twi_write_multiple(reg_vals, ARRAY_SIZE(reg_vals));
}

static int buck3_vout_set(uint16_t voltage)
{
	uint8_t reg_addr;
	uint8_t reg_val;
	uint8_t reg_val_pwmmode;
	bool pwm_mode_enable;
	int bytes_written;
	int bytes_read;

	/* It is recommended to set the BUCK into PWM mode while changing the output voltage. */
	reg_addr = NPM6001_REG_ADDR(BUCK3CONFPWMMODE);
	bytes_read = twi_read(&reg_val_pwmmode, sizeof(reg_val_pwmmode), reg_addr);
	if (bytes_read != sizeof(reg_val_pwmmode)) {
		return -ENODEV;
	}

	if ((reg_val_pwmmode & DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Msk) ==
	    (DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_OFF
	     << DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Pos)) {
		pwm_mode_enable = true;
	} else {
		pwm_mode_enable = false;
	}

	if (pwm_mode_enable) {
		reg_val = reg_val_pwmmode | (DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_ON
					     << DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Pos);
		bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
		if (bytes_written != sizeof(reg_val)) {
			return -ENODEV;
		}
	}

	reg_addr = NPM6001_REG_ADDR(BUCK3VOUT);
	reg_val = ((voltage - LIB_NPM6001_BUCK3_MINV) / LIB_NPM6001_BUCK3_RES) +
		  DIGITAL_BUCK3VOUT_VOLTAGE_Min;

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written != sizeof(reg_val)) {
		return -ENODEV;
	}

	reg_addr = NPM6001_REG_ADDR(BUCK3SELDAC);
	reg_val = DIGITAL_BUCK3SELDAC_SELECT_ENABLE << DIGITAL_BUCK3SELDAC_SELECT_Pos;

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written != sizeof(reg_val)) {
		return -ENODEV;
	}

	/* Revert PWM mode if needed */
	if (pwm_mode_enable) {
		bytes_written = twi_write(&reg_val_pwmmode, sizeof(reg_val_pwmmode), reg_addr);
		if (bytes_written != sizeof(reg_val_pwmmode)) {
			return -ENODEV;
		}
	}

	return 0;
}

static int ldo0_vout_set(uint16_t voltage)
{
	uint8_t reg_addr;
	uint8_t reg_val;
	int bytes_written;

	switch (voltage) {
	case 1800:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET1V8 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	case 2100:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET2V1 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	case 2400:
	case 2410:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET2V41 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	case 2700:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET2V7 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	case 3000:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET3V0 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	case 3300:
		reg_val = DIGITAL_LDO0VOUT_VOLTAGE_SET3V3 << DIGITAL_LDO0VOUT_VOLTAGE_Pos;
		break;
	default:
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(LDO0VOUT);

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_init(const struct lib_npm6001_platform *hw_funcs)
{
	int err;

	if (hw_funcs == NULL || hw_funcs->lib_npm6001_twi_read == NULL ||
	    hw_funcs->lib_npm6001_twi_write == NULL) {
		return -EINVAL;
	}

	twi_read = hw_funcs->lib_npm6001_twi_read;
	twi_write = hw_funcs->lib_npm6001_twi_write;

	if (hw_funcs->lib_npm6001_platform_init != NULL) {
		err = hw_funcs->lib_npm6001_platform_init();
		if (err) {
			return err;
		}
	}

	if (CONFIG_LIB_NPM6001_C3_WORKAROUND) {
		/* Magic workaround values */
		const uint8_t reg_addr = 0xAD;
		const uint8_t reg_val = 0x11;
		int bytes_written;

		bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
		if (bytes_written != sizeof(reg_val)) {
			return -ENODEV;
		}
	}

	if (CONFIG_LIB_NPM6001_WRITE_SWREADY) {
		const uint8_t reg_addr = NPM6001_REG_ADDR(SWREADY);
		const uint8_t reg_val =
			(DIGITAL_SWREADY_SWREADY_READY << DIGITAL_SWREADY_SWREADY_Pos);
		int bytes_written;

		/* Write SWREADY register to enable use of BUCK mode pins */
		bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
		if (bytes_written != sizeof(reg_val)) {
			return -ENODEV;
		}
	}

	return 0;
}

int lib_npm6001_vreg_enable(enum lib_npm6001_vreg regulator)
{
	struct twi_reg_val reg_vals[2];
	size_t reg_count = 1;

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
		/* Always enabled */
		return 0;
	case LIB_NPM6001_BUCK1:
	case LIB_NPM6001_BUCK2:
		return buck12_set_state(regulator, true);
	case LIB_NPM6001_BUCK3:
		reg_vals[0] = (struct twi_reg_val){
			.addr = NPM6001_REG_ADDR(BUCK3SELDAC),
			.val = DIGITAL_BUCK3SELDAC_SELECT_ENABLE << DIGITAL_BUCK3SELDAC_SELECT_Pos,
		};
		reg_vals[1] = (struct twi_reg_val){
			.addr = NPM6001_REG_ADDR(TASKS_START_BUCK3),
			.val = DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Trigger
		};
		reg_count = 2;
		break;
	case LIB_NPM6001_LDO0:
		reg_vals[0] = (struct twi_reg_val){
			.addr = NPM6001_REG_ADDR(TASKS_START_LDO0),
			.val = DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Trigger
		};
		break;
	case LIB_NPM6001_LDO1:
		reg_vals[0] = (struct twi_reg_val){
			.addr = NPM6001_REG_ADDR(TASKS_START_LDO1),
			.val = DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Trigger
		};
		break;
	default:
		return -EINVAL;
	}

	return twi_write_multiple(reg_vals, reg_count);
}

int lib_npm6001_vreg_disable(enum lib_npm6001_vreg regulator)
{
	uint8_t reg_addr;
	uint8_t reg_val;
	int bytes_written;

	switch (regulator) {
	case LIB_NPM6001_BUCK1:
	case LIB_NPM6001_BUCK2:
		return buck12_set_state(regulator, false);
	case LIB_NPM6001_BUCK3:
		reg_addr = NPM6001_REG_ADDR(TASKS_STOP_BUCK3);
		reg_val = DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Trigger;
		break;
	case LIB_NPM6001_LDO0:
		reg_addr = NPM6001_REG_ADDR(TASKS_STOP_LDO0);
		reg_val = DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Trigger;
		break;
	case LIB_NPM6001_LDO1:
		reg_addr = NPM6001_REG_ADDR(TASKS_STOP_LDO1);
		reg_val = DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Trigger;
		break;
	/* Remaining regulators cannot be disabled */
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_vreg_buck_mode_set(enum lib_npm6001_vreg regulator, enum lib_npm6001_vreg_mode mode)
{
	uint8_t reg_addr;
	uint8_t reg_val;
	int bytes_written;
	int bytes_read;

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
		reg_addr = NPM6001_REG_ADDR(BUCK0CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK1:
		reg_addr = NPM6001_REG_ADDR(BUCK1CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK2:
		reg_addr = NPM6001_REG_ADDR(BUCK2CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK3:
		reg_addr = NPM6001_REG_ADDR(BUCK3CONFPWMMODE);
		break;
	default:
		return -EINVAL;
	}

	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	reg_val &= ~DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Msk;

	switch (mode) {
	case LIB_NPM6001_MODE_PWM:
		reg_val |= DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_ON
			   << DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Pos;
		break;
	case LIB_NPM6001_MODE_ULP:
		reg_val |= DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_OFF
			   << DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Pos;
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_vreg_mode_pin_enable(enum lib_npm6001_vreg regulator, enum lib_npm6001_mode_pin pin)
{
	uint8_t reg_addr;
	uint8_t reg_val;
	int bytes_written;

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
		reg_addr = NPM6001_REG_ADDR(BUCK0CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK1:
		reg_addr = NPM6001_REG_ADDR(BUCK1CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK2:
		reg_addr = NPM6001_REG_ADDR(BUCK2CONFPWMMODE);
		break;
	case LIB_NPM6001_BUCK3:
		reg_addr = NPM6001_REG_ADDR(BUCK3CONFPWMMODE);
		break;
	default:
		return -EINVAL;
	}

	switch (pin) {
	case LIB_NPM6001_BUCK_PIN_MODE0:
		reg_val = DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_PWM
			  << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Pos;
		break;
	case LIB_NPM6001_BUCK_PIN_MODE1:
		reg_val = DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_PWM
			  << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Pos;
		break;
	case LIB_NPM6001_BUCK_PIN_MODE2:
		reg_val = DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_PWM
			  << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Pos;
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_mode_pin_cfg(enum lib_npm6001_mode_pin pin,
			     const struct lib_npm6001_mode_pin_cfg *cfg)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	uint8_t padtype;
	uint8_t pulldown;
	int bytes_written;
	int bytes_read;

	if (cfg == NULL) {
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(BUCKMODEPADCONF);
	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	switch (cfg->pad_type) {
	case LIB_NPM6001_MODE_PIN_CFG_PAD_TYPE_SCHMITT:
		padtype = DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_SCHMITT;
		break;
	case LIB_NPM6001_MODE_PIN_CFG_PAD_TYPE_CMOS:
		padtype = DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_CMOS;
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->pulldown) {
	case LIB_NPM6001_MODE_PIN_CFG_PULLDOWN_ENABLED:
		pulldown = DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_ENABLED;
		break;
	case LIB_NPM6001_MODE_PIN_CFG_PULLDOWN_DISABLED:
		pulldown = DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_DISABLED;
		break;
	default:
		return -EINVAL;
	}

	switch (pin) {
	case LIB_NPM6001_BUCK_PIN_MODE0:
		reg_val &= ~DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Msk;
		reg_val |= (padtype << DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Pos) |
			   (pulldown << DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Pos);
		break;
	case LIB_NPM6001_BUCK_PIN_MODE1:
		reg_val &= ~DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Msk;
		reg_val |= (padtype << DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Pos) |
			   (pulldown << DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Pos);
		break;
	case LIB_NPM6001_BUCK_PIN_MODE2:
		reg_val &= ~DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Msk;
		reg_val |= (padtype << DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Pos) |
			   (pulldown << DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Pos);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_vreg_voltage_set(enum lib_npm6001_vreg regulator, uint16_t voltage)
{
	uint16_t voltage_rounded;
	uint16_t voltage_res;
	bool voltage_valid;
	int err;

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
		voltage_valid =
			VOLTAGE_VALID(voltage, LIB_NPM6001_BUCK0_MINV, LIB_NPM6001_BUCK0_MAXV);
		voltage_res = LIB_NPM6001_BUCK0_RES;
		break;
	case LIB_NPM6001_BUCK1:
		voltage_valid =
			VOLTAGE_VALID(voltage, LIB_NPM6001_BUCK1_MINV, LIB_NPM6001_BUCK1_MAXV);
		voltage_res = LIB_NPM6001_BUCK1_RES;
		break;
	case LIB_NPM6001_BUCK2:
		voltage_valid =
			VOLTAGE_VALID(voltage, LIB_NPM6001_BUCK2_MINV, LIB_NPM6001_BUCK2_MAXV);
		voltage_res = LIB_NPM6001_BUCK2_RES;
		break;
	case LIB_NPM6001_BUCK3:
		voltage_valid =
			VOLTAGE_VALID(voltage, LIB_NPM6001_BUCK3_MINV, LIB_NPM6001_BUCK3_MAXV);
		voltage_res = LIB_NPM6001_BUCK3_RES;
		break;
	case LIB_NPM6001_LDO0:
		voltage_valid =
			VOLTAGE_VALID(voltage, LIB_NPM6001_LDO0_MINV, LIB_NPM6001_LDO0_MAXV);
		voltage_res = LIB_NPM6001_LDO0_RES;
		break;
	case LIB_NPM6001_LDO1:
		/* LDO1 cannot be adjusted */
		return -EINVAL;
	default:
		return -EINVAL;
	}

	if (!voltage_valid) {
		return -EINVAL;
	}

	voltage_rounded = VOLTAGE_ROUND(voltage, voltage_res);

	switch (regulator) {
	case LIB_NPM6001_BUCK0:
	case LIB_NPM6001_BUCK1:
	case LIB_NPM6001_BUCK2:
		err = buck012_vout_set(regulator, voltage_rounded);
		break;
	case LIB_NPM6001_BUCK3:
		err = buck3_vout_set(voltage_rounded);
		break;
	case LIB_NPM6001_LDO0:
		err = ldo0_vout_set(voltage_rounded);
		break;
	default:
		return -EINVAL;
	}

	return err;
}

int lib_npm6001_thermal_sensor_enable(enum lib_npm6001_thermal_sensor sensor)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	switch (sensor) {
	case LIB_NPM6001_THERMAL_SENSOR_WARNING:
		reg_val = DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Trigger
			  << DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Pos;
		reg_addr = NPM6001_REG_ADDR(TASKS_START_THWARN);
		break;
	case LIB_NPM6001_THERMAL_SENSOR_SHUTDOWN:
		reg_val = DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Trigger
			  << DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Pos;
		reg_addr = NPM6001_REG_ADDR(TASKS_START_TH_SHUTDN);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_thermal_sensor_disable(enum lib_npm6001_thermal_sensor sensor)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	switch (sensor) {
	case LIB_NPM6001_THERMAL_SENSOR_WARNING:
		reg_val = DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Trigger
			  << DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Pos;
		reg_addr = NPM6001_REG_ADDR(TASKS_STOP_THWARN);
		break;
	case LIB_NPM6001_THERMAL_SENSOR_SHUTDOWN:
		reg_val = DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Trigger
			  << DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Pos;
		reg_addr = NPM6001_REG_ADDR(TASKS_STOP_THSHUTDN);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_thermal_sensor_dyn_pwrup_enable(enum lib_npm6001_thermal_sensor sensor)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;
	int bytes_read;

	reg_addr = NPM6001_REG_ADDR(THDYNPOWERUP);
	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	switch (sensor) {
	case LIB_NPM6001_THERMAL_SENSOR_WARNING:
		reg_val &= ~(DIGITAL_THDYNPOWERUP_WARNING_Msk);
		reg_val |=
			(DIGITAL_THDYNPOWERUP_WARNING_SELECTED << DIGITAL_THDYNPOWERUP_WARNING_Pos);
		break;
	case LIB_NPM6001_THERMAL_SENSOR_SHUTDOWN:
		reg_val &= ~(DIGITAL_THDYNPOWERUP_SHUTDWN_Msk);
		reg_val |=
			(DIGITAL_THDYNPOWERUP_SHUTDWN_SELECTED << DIGITAL_THDYNPOWERUP_SHUTDWN_Pos);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_thermal_sensor_dyn_pwrup_trig(const enum lib_npm6001_vreg *vreg, uint8_t len)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;
	int bytes_read;

	if (vreg == NULL || len == 0) {
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(THDYNPOWERUP);
	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	for (int i = 0; i < len; ++i) {
		switch (vreg[i]) {
		case LIB_NPM6001_BUCK0:
			reg_val &= ~(DIGITAL_THDYNPOWERUP_BUCK0PWM_Msk);
			reg_val |= (DIGITAL_THDYNPOWERUP_BUCK0PWM_PWRUP
				    << DIGITAL_THDYNPOWERUP_BUCK0PWM_Pos);
			break;
		case LIB_NPM6001_BUCK1:
			reg_val &= ~(DIGITAL_THDYNPOWERUP_BUCK1PWM_Msk);
			reg_val |= (DIGITAL_THDYNPOWERUP_BUCK1PWM_PWRUP
				    << DIGITAL_THDYNPOWERUP_BUCK1PWM_Pos);
			break;
		case LIB_NPM6001_BUCK2:
			reg_val &= ~(DIGITAL_THDYNPOWERUP_BUCK2PWM_Msk);
			reg_val |= (DIGITAL_THDYNPOWERUP_BUCK2PWM_PWRUP
				    << DIGITAL_THDYNPOWERUP_BUCK2PWM_Pos);
			break;
		case LIB_NPM6001_BUCK3:
			reg_val &= ~(DIGITAL_THDYNPOWERUP_BUCK3PWM_Msk);
			reg_val |= (DIGITAL_THDYNPOWERUP_BUCK3PWM_PWRUP
				    << DIGITAL_THDYNPOWERUP_BUCK3PWM_Pos);
			break;
		default:
			return -EINVAL;
		}
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_thermal_sensor_dyn_pwrup_disable(enum lib_npm6001_thermal_sensor sensor)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;
	int bytes_read;

	reg_addr = NPM6001_REG_ADDR(THDYNPOWERUP);
	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	switch (sensor) {
	case LIB_NPM6001_THERMAL_SENSOR_WARNING:
		reg_val &= ~(DIGITAL_THDYNPOWERUP_WARNING_Msk);
		reg_val |= (DIGITAL_THDYNPOWERUP_WARNING_NOTSELECTED
			    << DIGITAL_THDYNPOWERUP_WARNING_Pos);
		break;
	case LIB_NPM6001_THERMAL_SENSOR_SHUTDOWN:
		reg_val &= ~(DIGITAL_THDYNPOWERUP_SHUTDWN_Msk);
		reg_val |= (DIGITAL_THDYNPOWERUP_SHUTDWN_NOTSELECTED
			    << DIGITAL_THDYNPOWERUP_SHUTDWN_Pos);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_int_enable(enum lib_npm6001_int interrupt)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	switch (interrupt) {
	case LIB_NPM6001_INT_THERMAL_WARNING:
		reg_val = DIGITAL_INTENSET0_THWARN_SET << DIGITAL_INTENSET0_THWARN_Pos;
		break;
	case LIB_NPM6001_INT_BUCK0_OVERCURRENT:
		reg_val = DIGITAL_INTENSET0_BUCK0OC_SET << DIGITAL_INTENSET0_BUCK0OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK1_OVERCURRENT:
		reg_val = DIGITAL_INTENSET0_BUCK1OC_SET << DIGITAL_INTENSET0_BUCK1OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK2_OVERCURRENT:
		reg_val = DIGITAL_INTENSET0_BUCK2OC_SET << DIGITAL_INTENSET0_BUCK2OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK3_OVERCURRENT:
		reg_val = DIGITAL_INTENSET0_BUCK3OC_SET << DIGITAL_INTENSET0_BUCK3OC_Pos;
		break;
	default:
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(INTENSET0);

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_int_disable(enum lib_npm6001_int interrupt)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	switch (interrupt) {
	case LIB_NPM6001_INT_THERMAL_WARNING:
		reg_val = DIGITAL_INTENCLR0_THWARN_CLEAR << DIGITAL_INTENCLR0_THWARN_Pos;
		break;
	case LIB_NPM6001_INT_BUCK0_OVERCURRENT:
		reg_val = DIGITAL_INTENCLR0_BUCK0OC_CLEAR << DIGITAL_INTENCLR0_BUCK0OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK1_OVERCURRENT:
		reg_val = DIGITAL_INTENCLR0_BUCK1OC_CLEAR << DIGITAL_INTENCLR0_BUCK1OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK2_OVERCURRENT:
		reg_val = DIGITAL_INTENCLR0_BUCK2OC_CLEAR << DIGITAL_INTENCLR0_BUCK2OC_Pos;
		break;
	case LIB_NPM6001_INT_BUCK3_OVERCURRENT:
		reg_val = DIGITAL_INTENCLR0_BUCK3OC_CLEAR << DIGITAL_INTENCLR0_BUCK3OC_Pos;
		break;
	default:
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(INTENCLR0);

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_int_read(enum lib_npm6001_int *interrupt)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_read;
	int bytes_written;

	if (interrupt == NULL) {
		return -EINVAL;
	}

	reg_addr = NPM6001_REG_ADDR(INTPEND0);

	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	/* Check source of interrupt and clear, one at a time. */
	if ((reg_val & DIGITAL_INTPEND0_THWARN_Msk) ==
	    (DIGITAL_INTPEND0_THWARN_PENDING << DIGITAL_INTPEND0_THWARN_Pos)) {
		*interrupt = LIB_NPM6001_INT_THERMAL_WARNING;
		reg_addr = NPM6001_REG_ADDR(EVENTS_THWARN);
	} else if ((reg_val & DIGITAL_INTPEND0_BUCK0OC_Msk) ==
		   (DIGITAL_INTPEND0_BUCK0OC_PENDING << DIGITAL_INTPEND0_BUCK0OC_Pos)) {
		*interrupt = LIB_NPM6001_INT_BUCK0_OVERCURRENT;
		reg_addr = NPM6001_REG_ADDR(EVENTS_BUCK0OC);
	} else if ((reg_val & DIGITAL_INTPEND0_BUCK1OC_Msk) ==
		   (DIGITAL_INTPEND0_BUCK1OC_PENDING << DIGITAL_INTPEND0_BUCK1OC_Pos)) {
		*interrupt = LIB_NPM6001_INT_BUCK1_OVERCURRENT;
		reg_addr = NPM6001_REG_ADDR(EVENTS_BUCK1OC);
	} else if ((reg_val & DIGITAL_INTPEND0_BUCK2OC_Msk) ==
		   (DIGITAL_INTPEND0_BUCK2OC_PENDING << DIGITAL_INTPEND0_BUCK2OC_Pos)) {
		*interrupt = LIB_NPM6001_INT_BUCK2_OVERCURRENT;
		reg_addr = NPM6001_REG_ADDR(EVENTS_BUCK2OC);
	} else if ((reg_val & DIGITAL_INTPEND0_BUCK3OC_Msk) ==
		   (DIGITAL_INTPEND0_BUCK3OC_PENDING << DIGITAL_INTPEND0_BUCK3OC_Pos)) {
		*interrupt = LIB_NPM6001_INT_BUCK3_OVERCURRENT;
		reg_addr = NPM6001_REG_ADDR(EVENTS_BUCK3OC);
	} else {
		return -EIO;
	}

	reg_val = 0;

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_gpio_cfg(enum lib_npm6001_gpio pin, const struct lib_npm6001_gpio_cfg *cfg)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	if (cfg == NULL) {
		return -EINVAL;
	}

	switch (pin) {
	case LIB_NPM6001_GPIO0:
		reg_addr = NPM6001_REG_ADDR(GPIO0CONF);
		break;
	case LIB_NPM6001_GPIO1:
		reg_addr = NPM6001_REG_ADDR(GPIO1CONF);
		break;
	case LIB_NPM6001_GPIO2:
		reg_addr = NPM6001_REG_ADDR(GPIO2CONF);
		break;
	default:
		return -EINVAL;
	}

	reg_val = 0;

	switch (cfg->direction) {
	case LIB_NPM6001_GPIO_CFG_DIRECTION_INPUT:
		reg_val |= (DIGITAL_GPIO0CONF_DIRECTION_INPUT << DIGITAL_GPIO0CONF_DIRECTION_Pos);
		break;
	case LIB_NPM6001_GPIO_CFG_DIRECTION_OUTPUT:
		reg_val |=
			(DIGITAL_GPIO0CONF_DIRECTION_OUTPUT << DIGITAL_GPIO0CONF_DIRECTION_Pos);
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->input) {
	case LIB_NPM6001_GPIO_CFG_INPUT_ENABLED:
		reg_val |= (DIGITAL_GPIO0CONF_INPUT_ENABLED << DIGITAL_GPIO0CONF_INPUT_Pos);
		break;
	case LIB_NPM6001_GPIO_CFG_INPUT_DISABLED:
		reg_val |= (DIGITAL_GPIO0CONF_INPUT_DISABLED << DIGITAL_GPIO0CONF_INPUT_Pos);
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->pulldown) {
	case LIB_NPM6001_GPIO_CFG_PULLDOWN_ENABLED:
		reg_val |= (DIGITAL_GPIO0CONF_PULLDOWN_ENABLED << DIGITAL_GPIO0CONF_PULLDOWN_Pos);
		break;
	case LIB_NPM6001_GPIO_CFG_PULLDOWN_DISABLED:
		reg_val |=
			(DIGITAL_GPIO0CONF_PULLDOWN_DISABLED << DIGITAL_GPIO0CONF_PULLDOWN_Pos);
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->drive) {
	case LIB_NPM6001_GPIO_CFG_DRIVE_NORMAL:
		reg_val |= (DIGITAL_GPIO0CONF_DRIVE_NORMAL << DIGITAL_GPIO0CONF_DRIVE_Pos);
		break;
	case LIB_NPM6001_GPIO_CFG_DRIVE_HIGH:
		reg_val |= (DIGITAL_GPIO0CONF_DRIVE_HIGH << DIGITAL_GPIO0CONF_DRIVE_Pos);
		break;
	default:
		return -EINVAL;
	}

	switch (cfg->sense) {
	case LIB_NPM6001_GPIO_CFG_SENSE_LOW:
		reg_val |= (DIGITAL_GPIO0CONF_SENSE_SCHMITT << DIGITAL_GPIO0CONF_SENSE_Pos);
		break;
	case LIB_NPM6001_GPIO_CFG_SENSE_HIGH:
		reg_val |= (DIGITAL_GPIO0CONF_SENSE_CMOS << DIGITAL_GPIO0CONF_SENSE_Pos);
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_gpio_set(enum lib_npm6001_gpio pin)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	reg_addr = NPM6001_REG_ADDR(GPIOOUTSET);

	switch (pin) {
	case LIB_NPM6001_GPIO0:
		reg_val = DIGITAL_GPIOOUTSET_GPIO0OUTSET_SET
			  << DIGITAL_GPIOOUTSET_GPIO0OUTSET_Pos;
		break;
	case LIB_NPM6001_GPIO1:
		reg_val = DIGITAL_GPIOOUTSET_GPIO1OUTSET_SET
			  << DIGITAL_GPIOOUTSET_GPIO1OUTSET_Pos;
		break;
	case LIB_NPM6001_GPIO2:
		reg_val = DIGITAL_GPIOOUTSET_GPIO2OUTSET_SET
			  << DIGITAL_GPIOOUTSET_GPIO2OUTSET_Pos;
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_gpio_clr(enum lib_npm6001_gpio pin)
{
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_written;

	reg_addr = NPM6001_REG_ADDR(GPIOOUTCLR);

	switch (pin) {
	case LIB_NPM6001_GPIO0:
		reg_val = DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_CLR
			  << DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Pos;
		break;
	case LIB_NPM6001_GPIO1:
		reg_val = DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_CLR
			  << DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Pos;
		break;
	case LIB_NPM6001_GPIO2:
		reg_val = DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_CLR
			  << DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Pos;
		break;
	default:
		return -EINVAL;
	}

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_gpio_read(enum lib_npm6001_gpio pin, bool *set)
{
	const uint8_t reg_addr = NPM6001_REG_ADDR(GPIOIN);
	uint8_t reg_val;
	int bytes_read;

	if (set == NULL) {
		return -EINVAL;
	}

	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	switch (pin) {
	case LIB_NPM6001_GPIO0:
		*set = (reg_val & DIGITAL_GPIOIN_GPIO0IN_Msk) ==
		       (DIGITAL_GPIOIN_GPIO0IN_HIGH << DIGITAL_GPIOIN_GPIO0IN_Pos);
		break;
	case LIB_NPM6001_GPIO1:
		*set = (reg_val & DIGITAL_GPIOIN_GPIO1IN_Msk) ==
		       (DIGITAL_GPIOIN_GPIO1IN_HIGH << DIGITAL_GPIOIN_GPIO1IN_Pos);
		break;
	case LIB_NPM6001_GPIO2:
		*set = (reg_val & DIGITAL_GPIOIN_GPIO2IN_Msk) ==
		       (DIGITAL_GPIOIN_GPIO2IN_HIGH << DIGITAL_GPIOIN_GPIO2IN_Pos);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int lib_npm6001_watchdog_enable(uint32_t timer_val)
{
	const struct twi_reg_val reg_vals[] = {
		{ .addr = NPM6001_REG_ADDR(WDPWRUPVALUE), .val = WDPWRUPVALUE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDPWRUPSTROBE), .val = WDPWRUPSTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE0), .val = (timer_val + 1) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE1), .val = ((timer_val + 1) >> 8) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE2),
		  .val = ((timer_val + 1) >> 16) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDDATASTROBE), .val = WDDATASTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDVALUE), .val = WDARMEDVALUE_EN_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDSTROBE), .val = WDARMEDSTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDKICK), .val = WDKICK_REG_VALUE },
	};

	if (timer_val == 0 || timer_val > 0x00FFFFFF) {
		return -EINVAL;
	}

	return twi_write_multiple(reg_vals, ARRAY_SIZE(reg_vals));
}

int lib_npm6001_watchdog_disable(void)
{
	const struct twi_reg_val reg_vals[] = {
		{ .addr = NPM6001_REG_ADDR(WDARMEDVALUE), .val = WDARMEDVALUE_DIS_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDSTROBE), .val = WDARMEDSTROBE_REG_VALUE },
	};

	return twi_write_multiple(reg_vals, ARRAY_SIZE(reg_vals));
}

int lib_npm6001_watchdog_kick(void)
{
	const uint8_t reg_addr = NPM6001_REG_ADDR(WDKICK);
	const uint8_t reg_val = WDKICK_REG_VALUE;
	int bytes_written;

	bytes_written = twi_write(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_written == sizeof(reg_val)) {
		return 0;
	} else {
		return -ENODEV;
	}
}

int lib_npm6001_hibernate(uint32_t timer_val)
{
	const struct twi_reg_val reg_vals[] = {
		{ .addr = NPM6001_REG_ADDR(WDPWRUPVALUE), .val = WDPWRUPVALUE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDPWRUPSTROBE), .val = WDPWRUPSTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE0), .val = (timer_val + 1) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE1), .val = ((timer_val + 1) >> 8) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDTRIGGERVALUE2),
		  .val = ((timer_val + 1) >> 16) & 0xFF },
		{ .addr = NPM6001_REG_ADDR(WDDATASTROBE), .val = WDDATASTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDVALUE), .val = WDARMEDVALUE_EN_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDSTROBE), .val = WDARMEDSTROBE_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDREQPOWERDOWN), .val = WDREQPOWERDOWN_REG_VALUE },
	};
	const struct twi_reg_val disable_wd_reg_vals[] = {
		{ .addr = NPM6001_REG_ADDR(WDARMEDVALUE), .val = WDARMEDVALUE_DIS_REG_VALUE },
		{ .addr = NPM6001_REG_ADDR(WDARMEDSTROBE), .val = WDARMEDSTROBE_REG_VALUE },
	};
	uint8_t reg_val;
	uint8_t reg_addr;
	int bytes_read;

	if (timer_val == 0 || timer_val > 0x00FFFFFF) {
		return -EINVAL;
	}

	/* Check if watchdog is running.
	 * If yes, it must be disabled to ensure hibernation duration is set properly.
	 */
	reg_addr = NPM6001_REG_ADDR(WDARMEDVALUE);
	bytes_read = twi_read(&reg_val, sizeof(reg_val), reg_addr);
	if (bytes_read != sizeof(reg_val)) {
		return -ENODEV;
	}

	if ((reg_val & DIGITAL_WDARMEDVALUE_VALUE_Msk) ==
	    (DIGITAL_WDARMEDVALUE_VALUE_ENABLE << DIGITAL_WDARMEDVALUE_VALUE_Pos)) {
		int err;

		err = twi_write_multiple(disable_wd_reg_vals, ARRAY_SIZE(disable_wd_reg_vals));
		if (err) {
			return err;
		}
	}

	return twi_write_multiple(reg_vals, ARRAY_SIZE(reg_vals));
}
