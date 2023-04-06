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
 * @file LSM6DS3.hpp
 * Driver for the ST LSM6DS3 MEMS accelerometer / gyroscope connected via SPI.
 */

#pragma once

#include <drivers/device/spi.h>
#include <geo/geo.h>
#include <perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>

// Bit definitions
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)

/* register addresses: XL: accel, G: gyro, TEMP: temp */
#define ADDR_WHO_AM_I			0x0F
#define WHO_I_AM			0x6A

#define CTRL1_XL	0x10	// Linear acceleration sensor, Control Register 1.
#define CTRL2_G		0x11	// Angular rate sensor, Control Register 2.
#define CTRL3_C		0x12 // Control register 3.
#define CTRL4_C		0x13 // Control register 4.
#define CTRL5_C		0x14 // Control register 5.
#define CTRL6_C		0x15 // Control register 6.
#define CTRL7_G		0x16	// Angular rate sensor, Control Register 7.
#define CTRL8_XL	0x17 // Linear acceleration sensor, Control Register 8.
#define CTRL9_XL	0x18 // Linear acceleration sensor, Control Register 9.
#define CTRL10_C	0x19 // Control register 10.

#define OUT_TEMP_L      0x20
#define OUT_TEMP_H      0x21

#define STATUS_REG	0x1E // Only one STATUS_REG as compared to lsm9ds1

#define OUT_X_L_G       0x22
#define OUT_X_H_G       0x23
#define OUT_Y_L_G       0x24
#define OUT_Y_H_G       0x25
#define OUT_Z_L_G       0x26
#define OUT_Z_H_G       0x27

#define OUT_X_L_XL      0x28
#define OUT_X_H_XL      0x29
#define OUT_Y_L_XL      0x2A
#define OUT_Y_H_XL      0x2B
#define OUT_Z_L_XL      0x2C
#define OUT_Z_H_XL      0x2D

#define CTRL1_XL_ODR_XL_BITS	(Bit7 | Bit6 | Bit5 | Bit4)
#define CTRL1_XL_ODR_XL_26Hz	(Bit5)
#define CTRL1_XL_ODR_XL_52Hz	(Bit5 | Bit4)
#define CTRL1_XL_ODR_XL_104Hz	(Bit6)
#define CTRL1_XL_ODR_XL_208Hz	(Bit6 | Bit4)
#define CTRL1_XL_ODR_XL_416Hz	(Bit6 | Bit5)
#define CTRL1_XL_ODR_XL_833Hz	(Bit6 | Bit5 | Bit4)

#define CTRL1_XL_FS_XL_BITS	(Bit3 | Bit2)
#define CTRL1_XL_FS_XL_2g	(0)
#define CTRL1_XL_FS_XL_4g	(Bit3)
#define CTRL1_XL_FS_XL_8g	(Bit3 | Bit2)
#define CTRL1_XL_FS_XL_16g	(Bit2)

#define CTRL2_G_ODR_G_BITS	(Bit7 | Bit6 | Bit5 | Bit4)
#define CTRL2_G_ODR_G_26Hz	(Bit5)
#define CTRL2_G_ODR_G_52Hz	(Bit5 | Bit4)
#define CTRL2_G_ODR_G_104Hz	(Bit6)
#define CTRL2_G_ODR_G_208Hz	(Bit6 | Bit4)
#define CTRL2_G_ODR_G_416Hz	(Bit6 | Bit5)
#define CTRL2_G_ODR_G_833Hz	(Bit6 | Bit5 | Bit4)

#define CTRL2_G_FS_G_BITS	(Bit3 | Bit2)
#define CTRL2_G_FS_G_125dps	(Bit1)
#define CTRL2_G_FS_G_250dps	(0)
#define CTRL2_G_FS_G_500dps	(Bit2)
#define CTRL2_G_FS_G_1000dps	(Bit3)
#define CTRL2_G_FS_G_2000dps	(Bit3 | Bit2)

#define CTRL3_C_BDU		(Bit6)
#define CTRL3_C_IF_INC		(Bit2)
#define CTRL3_C_BLE		(Bit1)
#define CTRL3_C_SW_RESET	(Bit0)

#define CTRL4_C_I2C_disable	(Bit2)

#define STATUS_REG_TDA		(Bit2)
#define STATUS_REG_GDA		(Bit1)
#define STATUS_REG_XLDA		(Bit0)

/* default values for this device */
#define LSM6DS3_ACCEL_DEFAULT_RANGE_G			16
#define LSM6DS3_ACCEL_DEFAULT_ODR			26

#define LSM6DS3_GYRO_DEFAULT_RANGE_DPS			2000
#define LSM6DS3_GYRO_DEFAULT_ODR			26

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates using the data ready bit.
  This time reduction is enough to cope with worst case timing jitter
  due to other timers
 */
#define LSM6DS3_TIMER_REDUCTION				200

class LSM6DS3 : public device::SPI, public I2CSPIDriver<LSM6DS3>
{
public:
	LSM6DS3(const I2CSPIDriverConfig &config);
	~LSM6DS3() override;

	static void print_usage();

	void RunImpl();

	int		init() override;

	void print_status() override;

protected:
	int		probe() override;
private:

	void			start();
	void			reset();

	/**
	 * check key registers for correct values
	 */
	void			check_registers(void);

	/**
	 * Read a register from the LSM6DS3
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg) override;

	/**
	 * Write a register in the LSM6DS3
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 * @return		OK on success, negative errno otherwise.
	 */
	int			write_reg(unsigned reg, uint8_t value) override;

	/**
	 * Modify a register in the LSM6DS3
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the LSM6DS3, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * Set the LSM6DS3 accel measurement range.
	 *
	 * @param max_g	The measurement range of the accel is in g (9.81m/s^2)
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			accel_set_range(unsigned max_g);

	/**
	 * Set the LSM6DS3 gyro measurement range.
	 *
	 * @param max_ga	The measurement range of the gyro is in Ga
	 *			Zero selects the maximum supported range.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			gyro_set_range(unsigned max_g);

	/**
	 * Set the LSM6DS3 internal accel sampling frequency.
	 *
	 * @param frequency	The internal accel sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			accel_set_samplerate(unsigned frequency);

	/**
	 * Set the LSM6DS3 internal gyro sampling frequency.
	 *
	 * @param frequency	The internal gyro sampling frequency is set to not less than
	 *			this value.
	 *			Zero selects the maximum rate supported.
	 * @return		OK if the value can be supported.
	 */
	int			gyro_set_samplerate(unsigned frequency);


	PX4Accelerometer	_px4_accel;
	PX4Gyroscope		_px4_gyro;

	unsigned		_call_accel_interval{1000000 / LSM6DS3_ACCEL_DEFAULT_ODR};
	unsigned		_call_gyro_interval{1000000 / LSM6DS3_GYRO_DEFAULT_ODR};

	perf_counter_t		_accel_sample_perf;
	perf_counter_t		_gyro_sample_perf;
	perf_counter_t		_bad_registers;
	perf_counter_t		_bad_values;
	perf_counter_t		_accel_duplicates;
	perf_counter_t		_gyro_duplicates;

	uint8_t			_register_wait{0};

	float			_last_temperature{0.0f};

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int	LSM6DS3_NUM_CHECKED_REGISTERS{5};
	uint8_t			_checked_values[LSM6DS3_NUM_CHECKED_REGISTERS] {};
	uint8_t			_checked_next{0};

};
