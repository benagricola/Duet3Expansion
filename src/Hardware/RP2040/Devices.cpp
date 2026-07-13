/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if RPXXXX

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <RTOSIface/RTOSIface.h>
#include <TinyUsbInterface.h>
#include <SerialCDC_tusb.h>
#if SUPPORT_CAN && USE_SPICAN
# include <Platform.h>
# include <SharedSpiClient.h>
# include <CanSpi.h>
# include "StepTimer.h"
#endif

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

constexpr size_t UsbDeviceTaskStackWords = 200;
static Task<UsbDeviceTaskStackWords> usbDeviceTask;

SerialCDC serialUSB;

void DeviceInit() noexcept
{
	AnalogIn::Init(DmacChanAdcRx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);

	CoreUsbInit(NvicPriorityUSB);
	usbDeviceTask.Create(CoreUsbDeviceTask, "USBD", nullptr, TaskPriority::UsbPriority);
}

#if SUPPORT_CAN && USE_SPICAN
// SPICAN SPI interface
SharedSpiClient *spiCanHardware;
extern "C" bool DRV_SPI_Initialize()
{
	debugPrintf("SPI init start\n");
	spiCanHardware = new SharedSpiClient(Platform::GetSharedSpi(spiCan_SpiChannel), 15000000, SpiMode::mode0, NoPin, false);
	IoPort::SetPinMode(SPICanCsPin, OUTPUT_HIGH);
#if SPICAN_CORE0_SERVICE
	// Configure the bus without taking its mutex. In core-0 service mode this init runs in a
	// FreeRTOS task (MAIN), and holding the bus mutex forever is not harmless: FreeRTOS only
	// restores a task's base priority when it holds NO mutexes, so any transient priority boost
	// MAIN inherited latched permanently and starved its base-priority peers - seen on the bench
	// as the encoder calibration task never completing (M569.6 hanging the main board).
	// This mode therefore requires the CAN chip to have the SPI bus to itself (true on every
	// current USE_SPICAN board): transactions are serialised by the CAN driver's own mutex, and
	// nothing else may configure or use the bus. If a future board shares the bus, take the bus
	// mutex around each transfer in DRV_SPI_TransferData instead.
	spiCanHardware->SelectNoMutex();
#else
	// Core-1 service (the stock arrangement): reserve the bus permanently. Core 1 performs the
	// transfers outside FreeRTOS and cannot take the mutex per transfer, so the init-time Select
	// is what keeps core-0 bus clients off the bus for good.
	spiCanHardware->Select(1000);
#endif
	debugPrintf("SPI init complete\n");
    return true;
}

extern "C" void DRV_SPI_Select()
{
}

extern "C" void DRV_SPI_Deselect()
{
}

extern "C" int8_t DRV_SPI_TransferData(uint32_t index, uint8_t *SpiTxData, uint8_t *SpiRxData, size_t spiTransferSize)
{
	IoPort::WriteDigital(SPICanCsPin, 0);
	const bool ret = spiCanHardware->TransceivePacket(SpiTxData, SpiRxData, spiTransferSize);
	IoPort::WriteDigital(SPICanCsPin, 1);
	return !ret;
}

extern "C" uint32_t DRV_SPI_GetStepTimerTicks()
{
    return StepTimer::GetTimerTicks();
}

#endif
#endif

// End
