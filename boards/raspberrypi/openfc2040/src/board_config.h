/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * board internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* LEDs */
// Take a look at rpi_common micro_hal.h for gpio definitions
#define GPIO_LED1	PX4_MAKE_GPIO_OUTPUT_SET(13)
#define GPIO_LED2	PX4_MAKE_GPIO_OUTPUT_SET(14)
#define GPIO_LED3	PX4_MAKE_GPIO_OUTPUT_SET(15)

#define GPIO_LED_RED                 GPIO_LED1
#define GPIO_LED_GREEN               GPIO_LED2
#define GPIO_LED_BLUE                GPIO_LED3

#define BOARD_HAS_CONTROL_STATUS_LEDS 1
#define BOARD_OVERLOAD_LED     LED_RED
#define BOARD_ARMED_LED        LED_BLUE
#define BOARD_ARMED_STATE_LED  LED_GREEN

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that can be used by the Px4 Firmware in the adc driver
 */
#define ADC_CHANNELS (1 << 1) | (1 << 2)	// Change this later based on the adc channels actually used
#define ADC_BATTERY_VOLTAGE_CHANNEL  1		// Corresponding GPIO 27. Used in init.c for disabling GPIO_IE
#define ADC_BATTERY_CURRENT_CHANNEL  2		// Corresponding GPIO 28. Used in init.c for disabling GPIO_IE


/* High-resolution timer */
#define HRT_TIMER 1
#define HRT_TIMER_CHANNEL 1
#define HRT_PPM_CHANNEL 1	// Number really doesn't matter for this board
#define GPIO_PPM_IN		(24 | GPIO_FUN(RP2040_GPIO_FUNC_SIO))

/* This board provides a DMA pool and APIs */			// Needs to be figured out
#define BOARD_DMA_ALLOC_POOL_SIZE 2048

#define BOARD_ENABLE_CONSOLE_BUFFER
#define BOARD_CONSOLE_BUFFER_SIZE (1024*3)

/* USB
 *
 *  VBUS detection is on 29  ADC_DPM0 and PTE8
 */
#define GPIO_USB_VBUS_VALID     (29 | GPIO_FUN(RP2040_GPIO_FUNC_SIO))    // Used in usb.c

/* PWM
 *
 * Alternatively CH3/CH4 could be assigned to UART6_TX/RX
 */
#define DIRECT_PWM_OUTPUT_CHANNELS      4

// Has pwm outputs
#define BOARD_HAS_PWM    DIRECT_PWM_OUTPUT_CHANNELS

__BEGIN_DECLS

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Name: rp2040_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ****************************************************************************************************/

extern void rp2040_spiinitialize(void);


/****************************************************************************************************
 * Name: rp2040_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

extern void rp2040_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
