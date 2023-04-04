/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file ST_LSM6DS3_registers.hpp
 *
 * ST LSM6DS3 registers.
 *
 */

#pragma once

#include <cstdint>

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

namespace ST_LSM6DS3
{
static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10 MHz SPI clock frequency

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t WHO_AM_I_ID = 0b01101010; // Who I am ID

static constexpr uint32_t FIFO_ODR = 833; // FIFO data rate
static constexpr uint32_t LA_ODR = 833; // Linear acceleration output data rate
static constexpr uint32_t G_ODR  = 833; // Angular rate output data rate

enum class Register : uint8_t {
	WHO_AM_I        = 0x0F,

	CTRL1_XL	= 0x10,	// Linear acceleration sensor, Control Register 1.
	CTRL2_G		= 0x11,	// Angular rate sensor, Control Register 2.
	CTRL3_C		= 0x12, // Control register 3.
	CTRL4_C		= 0x13, // Control register 4.
	CTRL5_C		= 0x14, // Control register 5.
	CTRL6_C		= 0x15, // Control register 6.
	CTRL7_G		= 0x16,	// Angular rate sensor, Control Register 7.
	CTRL8_XL	= 0x17, // Linear acceleration sensor, Control Register 8.
	CTRL9_XL	= 0x18, // Linear acceleration sensor, Control Register 9.
	CTRL10_C	= 0x19, // Control register 10.

	OUT_TEMP_L      = 0x20,
	OUT_TEMP_H      = 0x21,
	STATUS_REG_G    = 0x1E,	// Only one STATUS_REG as compared to lsm9ds1
	OUT_X_L_G       = 0x22,
	OUT_X_H_G       = 0x23,
	OUT_Y_L_G       = 0x24,
	OUT_Y_H_G       = 0x25,
	OUT_Z_L_G       = 0x26,
	OUT_Z_H_G       = 0x27,

	STATUS_REG_A    = 0x1E,	// Only one STATUS_REG as compared to lsm9ds1
	OUT_X_L_XL      = 0x28,
	OUT_X_H_XL      = 0x29,
	OUT_Y_L_XL      = 0x2A,
	OUT_Y_H_XL      = 0x2B,
	OUT_Z_L_XL      = 0x2C,
	OUT_Z_H_XL      = 0x2D,

	FIFO_CTRL3	= 0x08, // FIFO control register 3.
	FIFO_CTRL5	= 0x0A, // FIFO control register 5.
	FIFO_STATUS1	= 0x3A, // FIFO status control register 1.
	FIFO_STATUS2	= 0x3B, // FIFO status control register 2.
	FIFO_STATUS3	= 0x3C, // FIFO status control register 3. Used to check pattern.
	FIFO_STATUS4	= 0x3D, // FIFO status control register 4. Used to check pattern.
	FIFO_DATA_OUT_L	= 0x3E, // FIFO Data Out LSB
	FIFO_DATA_OUT_H	= 0x3F, // FIFO Data Out MSB
};

// CTRL1_XL
enum CTRL1_XL_BIT : uint8_t {
	// ODR_XL [3:0]
	ODR_XL_833HZ = Bit6 | Bit5 | Bit4, // 833 Hz ODR
	// FS_XL [1:0]
	FS_XL_16 = Bit2,
};

// CTRL2_G
enum CTRL2_G_BIT : uint8_t {
	// ODR_G [2:0]
	ODR_G_833HZ  = Bit6 | Bit5 | Bit4, // 833 Hz ODR
	// FS_G [1:0]
	FS_G_2000DPS = Bit3 | Bit2,
	// FS_G_125 [1:0]
	FS_G_125DPS   = Bit1, // FS_G 125dps
};

// CTRL3_C
enum CTRL3_C_BIT : uint8_t {
	BDU        = Bit6, // Block data update

	IF_ADD_INC = Bit2, // Register address automatically incremented

	SW_RESET   = Bit0, // Software reset
};

// CTRL4_C
enum CTRL4_C_BIT : uint8_t {
	I2C_DISABLE = Bit2,
};

// STATUS_REG (both STATUS_REG_A 0x17 and STATUS_REG_G 0x27)
enum STATUS_REG_BIT : uint8_t {
	TDA  = Bit2, // Temperature sensor new data available.
	GDA  = Bit1, // Gyroscope new data available.
	XLDA = Bit0, // Accelerometer new data available.
};

// FIFO_CTRL3
enum FIFO_CTRL3_BIT : uint8_t {
	// DEC_FIFO_GYRO [2:0]
	NO_DEC_FIFO_GYRO = Bit3,	// No decimation
	// DEC_FIFO_XL [2:0]
	NO_DEC_FIFO_XL = Bit0,		// No decimation
};

// FIFO_CTRL5
enum FIFO_CTRL5_BIT : uint8_t {
	// ODR_FIFO [3:0]
	ODR_FIFO_833HZ = Bit5 | Bit4 | Bit3, // 833 hz ODR
	// FMODE [2:0]
	FMODE_CONTINUOUS = Bit2 | Bit1, // Continuous mode. If the FIFO is full, the new sample over- writes the older sample.
};

// FIFO_STATUS2
enum FIFO_STATUS2_BIT : uint8_t {
	OVRN = Bit6, // FIFO overrun status.
};


namespace FIFO
{
static constexpr size_t SIZE = 128 * 12; // 32 samples max
}

} // namespace ST_LSM6DS3
