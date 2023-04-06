/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file LSM6DS3.cpp
 * Driver for the ST LSM6DS3 MEMS accelerometer / gyronetometer connected via SPI.
 */

#include "LSM6DS3.hpp"

/*
  list of registers that will be checked in check_registers(). Note
  that ADDR_WHO_AM_I must be first in the list.
 */
static constexpr uint8_t _checked_registers[] = {
	ADDR_WHO_AM_I,
	CTRL1_XL,
	CTRL2_G,
	CTRL3_C,
	CTRL4_C,
};

LSM6DS3::LSM6DS3(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation),
	_accel_sample_perf(perf_alloc(PC_ELAPSED, "lsm6ds3: acc_read")),
	_gyro_sample_perf(perf_alloc(PC_ELAPSED, "lsm6ds3: gyro_read")),
	_bad_registers(perf_alloc(PC_COUNT, "lsm6ds3: bad_reg")),
	_bad_values(perf_alloc(PC_COUNT, "lsm6ds3: bad_val")),
	_accel_duplicates(perf_alloc(PC_COUNT, "lsm6ds3: acc_dupe"))
{
}

LSM6DS3::~LSM6DS3()
{
	// delete the perf counter
	perf_free(_accel_sample_perf);
	perf_free(_gyro_sample_perf);
	perf_free(_bad_registers);
	perf_free(_bad_values);
	perf_free(_accel_duplicates);
}

int
LSM6DS3::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed (%i)", ret);
		return ret;
	}

	reset();

	start();

	return ret;
}

void
LSM6DS3::reset()
{
	// Disable I2C
	write_checked_reg(CTRL4_C, CTRL4_C_I2C_disable);

	// Enable  Block Data Update and auto address increment
	write_checked_reg(CTRL3_C, CTRL3_C_BDU | CTRL3_C_IF_INC | CTRL3_C_BLE);

	accel_set_range(LSM6DS3_ACCEL_DEFAULT_RANGE_G);
	accel_set_samplerate(LSM6DS3_ACCEL_DEFAULT_ODR);

	gyro_set_range(LSM6DS3_GYRO_DEFAULT_RANGE_DPS);
	gyro_set_samplerate(LSM6DS3_GYRO_DEFAULT_ODR);
}

int
LSM6DS3::probe()
{
	// read dummy value to void to clear SPI statemachine on sensor
	read_reg(ADDR_WHO_AM_I);

	// verify that the device is attached and functioning
	if (read_reg(ADDR_WHO_AM_I) == WHO_I_AM) {
		_checked_values[0] = WHO_I_AM;
		return OK;
	}

	return -EIO;
}

uint8_t
LSM6DS3::read_reg(unsigned reg)
{
	uint8_t cmd[2] {};
	cmd[0] = reg | DIR_READ;

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

int
LSM6DS3::write_reg(unsigned reg, uint8_t value)
{
	uint8_t	cmd[2] {};

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	return transfer(cmd, nullptr, sizeof(cmd));
}

void
LSM6DS3::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < LSM6DS3_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
		}
	}
}

void
LSM6DS3::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

int
LSM6DS3::accel_set_range(unsigned max_g)
{
	uint8_t setbits = 0;
	uint8_t clearbits = CTRL1_XL_FS_XL_BITS;
	float new_scale_g_digit = 0.0f;

	if (max_g == 0) {
		max_g = LSM6DS3_ACCEL_DEFAULT_RANGE_G;
	}

	if (max_g <= 2) {
		// accel_range_m_s2 = 2.0f * CONSTANTS_ONE_G;
		setbits |= CTRL1_XL_FS_XL_2g;
		new_scale_g_digit = 0.061e-3f;

	} else if (max_g <= 4) {
		// accel_range_m_s2 = 4.0f * CONSTANTS_ONE_G;
		setbits |= CTRL1_XL_FS_XL_4g;
		new_scale_g_digit = 0.122e-3f;

	} else if (max_g <= 8) {
		// accel_range_m_s2 = 8.0f * CONSTANTS_ONE_G;
		setbits |= CTRL1_XL_FS_XL_8g;
		new_scale_g_digit = 0.244e-3f;

	} else if (max_g <= 16) {
		// accel_range_m_s2 = 16.0f * CONSTANTS_ONE_G;
		setbits |= CTRL1_XL_FS_XL_16g;
		new_scale_g_digit = 0.488e-3f;

	} else {
		return -EINVAL;
	}

	float accel_range_scale = new_scale_g_digit * CONSTANTS_ONE_G;

	_px4_accel.set_scale(accel_range_scale);

	modify_reg(CTRL1_XL, clearbits, setbits);

	return OK;
}

int
LSM6DS3::gyro_set_range(unsigned max_dps)
{
	uint8_t setbits = 0;
	uint8_t clearbits = CTRL2_G_FS_G_BITS;
	float new_scale_dps_digit = 0.0f;

	if (max_dps == 0) {
		max_dps = LSM6DS3_GYRO_DEFAULT_RANGE_DPS;
	}

	if (max_dps <= 125) {
		// gyro_range_dps = 125;
		setbits |= CTRL2_G_FS_G_125dps;
		new_scale_dps_digit = 4.375e-3f;

	} else if (max_dps <= 250) {
		// gyro_range_dps = 250;
		setbits |= CTRL2_G_FS_G_250dps;
		new_scale_dps_digit = 8.75e-3f;

	} else if (max_dps <= 500) {
		// gyro_range_dps = 500;
		setbits |= CTRL2_G_FS_G_500dps;
		new_scale_dps_digit = 17.50e-3f;

	} else if (max_dps <= 1000) {
		// gyro_range_dps = 1000;
		setbits |= CTRL2_G_FS_G_1000dps;
		new_scale_dps_digit = 35e-3f;

	} else if (max_dps <= 2000) {
		// gyro_range_dps = 2000;
		setbits |= CTRL2_G_FS_G_2000dps;
		new_scale_dps_digit = 70e-3f;

	} else {
		return -EINVAL;
	}

	_px4_gyro.set_scale(new_scale_dps_digit);

	modify_reg(CTRL2_G, clearbits, setbits);

	return OK;
}

int
LSM6DS3::accel_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = CTRL1_XL_ODR_XL_BITS;

	if (frequency == 0) {
		frequency = LSM6DS3_ACCEL_DEFAULT_ODR;
	}

	int accel_samplerate = 26;

	if (frequency <= 26) {
		setbits |= CTRL1_XL_ODR_XL_26Hz;
		accel_samplerate = 26;

	} else if (frequency <= 52) {
		setbits |= CTRL1_XL_ODR_XL_52Hz;
		accel_samplerate = 52;

	} else if (frequency <= 104) {
		setbits |= CTRL1_XL_ODR_XL_104Hz;
		accel_samplerate = 104;

	} else if (frequency <= 208) {
		setbits |= CTRL1_XL_ODR_XL_208Hz;
		accel_samplerate = 208;

	} else if (frequency <= 416) {
		setbits |= CTRL1_XL_ODR_XL_416Hz;
		accel_samplerate = 416;

	} else if (frequency <= 833) {
		setbits |= CTRL1_XL_ODR_XL_833Hz;
		accel_samplerate = 833;

	} else {
		return -EINVAL;
	}

	_call_accel_interval = 1000000 / accel_samplerate;

	modify_reg(CTRL1_XL, clearbits, setbits);

	return OK;
}

int
LSM6DS3::gyro_set_samplerate(unsigned frequency)
{
	uint8_t setbits = 0;
	uint8_t clearbits = CTRL2_G_ODR_G_BITS;

	if (frequency == 0) {
		frequency = LSM6DS3_GYRO_DEFAULT_ODR;
	}

	int gyro_samplerate = 26;

	if (frequency <= 26) {
		setbits |= CTRL2_G_ODR_G_26Hz;
		gyro_samplerate = 26;

	} else if (frequency <= 52) {
		setbits |= CTRL2_G_ODR_G_52Hz;
		gyro_samplerate = 52;

	} else if (frequency <= 104) {
		setbits |= CTRL2_G_ODR_G_104Hz;
		gyro_samplerate = 104;

	} else if (frequency <= 208) {
		setbits |= CTRL2_G_ODR_G_208Hz;
		gyro_samplerate = 208;

	} else if (frequency <= 416) {
		setbits |= CTRL2_G_ODR_G_416Hz;
		gyro_samplerate = 416;

	} else if (frequency <= 833) {
		setbits |= CTRL2_G_ODR_G_833Hz;
		gyro_samplerate = 833;

	} else {
		return -EINVAL;
	}

	_call_gyro_interval = 1000000 / gyro_samplerate;

	modify_reg(CTRL2_G, clearbits, setbits);

	return OK;
}

void
LSM6DS3::start()
{
	// start polling at the specified rate
	ScheduleOnInterval(_call_accel_interval - LSM6DS3_TIMER_REDUCTION);
}

void
LSM6DS3::RunImpl()
{
	// make measurement
	perf_begin(_accel_sample_perf);
	perf_begin(_gyro_sample_perf);

	// status register and data as read back from the device
#pragma pack(push, 1)
	struct {
		uint8_t		cmd;
		uint8_t		status;
		int16_t		temperature;
		int16_t		gx;
		int16_t		gy;
		int16_t		gz;
		int16_t		ax;
		int16_t		ay;
		int16_t		az;
	} data_report{};
#pragma pack(pop)

	check_registers();

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again.
		_register_wait--;
		perf_end(_accel_sample_perf);
		perf_end(_gyro_sample_perf);
		return;
	}

	/* fetch data from the sensor */
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	data_report.cmd = STATUS_REG | DIR_READ;
	transfer((uint8_t *)&data_report, (uint8_t *)&data_report, sizeof(data_report));

	if (!(data_report.status & (STATUS_REG_GDA | STATUS_REG_XLDA))) {
		perf_end(_accel_sample_perf);
		perf_end(_gyro_sample_perf);
		perf_count(_accel_duplicates);
		perf_count(_gyro_duplicates);
		return;
	}

	_last_temperature = 25.0f + (data_report.temperature / 256.0f);
	_px4_accel.set_temperature(_last_temperature);
	_px4_gyro.set_temperature(_last_temperature);

	// report the error count as the sum of the number of bad
	// register reads and bad values. This allows the higher level
	// code to decide if it should use this sensor based on
	// whether it has had failures
	_px4_accel.set_error_count(perf_event_count(_bad_registers) + perf_event_count(_bad_values));
	_px4_accel.update(timestamp_sample, data_report.ax, data_report.ay, data_report.az);
	_px4_gyro.update(timestamp_sample, data_report.gx, data_report.gy, data_report.gz);

	perf_end(_accel_sample_perf);
	perf_end(_gyro_sample_perf);
}

void
LSM6DS3::check_registers(void)
{
	uint8_t v = 0;

	if ((v = read_reg(_checked_registers[_checked_next])) != _checked_values[_checked_next]) {
		/*
		  if we get the wrong value then we know the SPI bus
		  or sensor is very sick. We set _register_wait to 20
		  and wait until we have seen 20 good values in a row
		  before we consider the sensor to be OK again.
		 */
		perf_count(_bad_registers);

		/*
		  try to fix the bad register value. We only try to
		  fix one per loop to prevent a bad sensor hogging the
		  bus. We skip zero as that is the WHO_AM_I, which
		  is not writeable
		 */
		if (_checked_next != 0) {
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % LSM6DS3_NUM_CHECKED_REGISTERS;
}

void
LSM6DS3::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_accel_sample_perf);
	perf_print_counter(_gyro_sample_perf);
	perf_print_counter(_bad_registers);
	perf_print_counter(_bad_values);
	perf_print_counter(_accel_duplicates);

	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < LSM6DS3_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i]);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}
	}
}
