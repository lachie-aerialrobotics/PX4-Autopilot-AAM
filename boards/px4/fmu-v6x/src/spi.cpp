/****************************************************************************
 *
 *   Copyright (C) 2020, 2022 PX4 Development Team. All rights reserved.
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

#include <px4_arch/spi_hw_description.h>
#include <drivers/drv_sensor.h>
#include <nuttx/spi/spi.h>

constexpr px4_spi_bus_all_hw_t px4_spi_buses_all_hw[BOARD_NUM_SPI_CFG_HW_VERSIONS] = {
	initSPIFmumID(V6X_0, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20649, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin8}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
			initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin6}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),

	initSPIFmumID(V6X_1, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20649, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin8}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
			initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin4}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),

	initSPIFmumID(V6X_3, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin8}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
			initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin6}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),

	initSPIFmumID(V6X_4, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42670P, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20649, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),

	initSPIFmumID(V6X_6, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_IIM42652, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_IMU_DEVTYPE_ADIS16470, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),
<<<<<<< HEAD
<<<<<<< HEAD
	initSPIHWVersion(V6X0910, {
=======

	initSPIFmumID(V6X_8, {
>>>>>>> 32aa3263a60d48a960eb8a2ccc50073815250889
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM45686, SPI::CS{GPIO::PortI, GPIO::Pin4}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),


	initSPIFmumID(V6X_16, {
		initSPIBus(SPI::Bus::SPI1, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM20602, SPI::CS{GPIO::PortI, GPIO::Pin9}, SPI::DRDY{GPIO::PortF, GPIO::Pin2}),
		}, {GPIO::PortI, GPIO::Pin11}),
		initSPIBus(SPI::Bus::SPI2, {
			initSPIDevice(DRV_IMU_DEVTYPE_ICM42688P, SPI::CS{GPIO::PortH, GPIO::Pin5}, SPI::DRDY{GPIO::PortA, GPIO::Pin10}),
		}, {GPIO::PortF, GPIO::Pin4}),
		initSPIBus(SPI::Bus::SPI3, {
			initSPIDevice(DRV_GYR_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin8}, SPI::DRDY{GPIO::PortI, GPIO::Pin7}),
			initSPIDevice(DRV_ACC_DEVTYPE_BMI088, SPI::CS{GPIO::PortI, GPIO::Pin4}),
		}, {GPIO::PortE, GPIO::Pin7}),
		//  initSPIBus(SPI::Bus::SPI4, {
		//    // no devices
		// TODO: if enabled, remove GPIO_VDD_3V3_SENSORS4_EN from board_config.h
		//  }, {GPIO::PortG, GPIO::Pin8}),
		initSPIBus(SPI::Bus::SPI5, {
			initSPIDevice(SPIDEV_FLASH(0), SPI::CS{GPIO::PortG, GPIO::Pin7})
		}),
		initSPIBusExternal(SPI::Bus::SPI6, {
			initSPIConfigExternal(SPI::CS{GPIO::PortI, GPIO::Pin10}, SPI::DRDY{GPIO::PortD, GPIO::Pin11}),
			initSPIConfigExternal(SPI::CS{GPIO::PortA, GPIO::Pin15}, SPI::DRDY{GPIO::PortD, GPIO::Pin12}),
		}),
	}),
<<<<<<< HEAD
=======
>>>>>>> upstream/stable
=======

>>>>>>> 32aa3263a60d48a960eb8a2ccc50073815250889
};

static constexpr bool unused = validateSPIConfig(px4_spi_buses_all_hw);
