/*
 * TmcDriverTemperatureSensor.cpp
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#include "TmcDriverTemperatureSensor.h"
#include <Movement/Move.h>
#include <CanMessageGenericParser.h>

#if HAS_SMART_DRIVERS

#if SUPPORT_TMC2240
# include <TMC22xx.h>
#endif

#if SUPPORT_TMC51xx
# include <TMC51xx.h>
#endif

// Sensor type descriptors
TemperatureSensor::SensorTypeDescriptor TmcDriverTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TmcDriverTemperatureSensor(sensorNum); } );

TmcDriverTemperatureSensor::TmcDriverTemperatureSensor(unsigned int sensorNum)
	: TemperatureSensor(sensorNum, "TMC temperature warnings")
{
}

void TmcDriverTemperatureSensor::Poll()
{
	SetResult(moveInstance->GetTmcDriversTemperature(), TemperatureError::ok);
}

#if SUPPORT_TMC2240 || (SUPPORT_TMC51xx && TMC_TYPE == 2240)

// TmcDriverActualTemperatureSensor implementation
TemperatureSensor::SensorTypeDescriptor TmcDriverActualTemperatureSensor::typeDescriptor(TypeName, [](unsigned int sensorNum) noexcept -> TemperatureSensor *_ecv_from { return new TmcDriverActualTemperatureSensor(sensorNum); } );

TmcDriverActualTemperatureSensor::TmcDriverActualTemperatureSensor(unsigned int sensorNum) noexcept
	: TemperatureSensor(sensorNum, "TMC driver temperature"), driverNumber(0)
{
}

TmcDriverActualTemperatureSensor::~TmcDriverActualTemperatureSensor() noexcept
{
}

GCodeResult TmcDriverActualTemperatureSensor::Configure(const CanMessageGenericParser& parser, const StringRef& reply)
{
	bool seen = false;
	
	uint8_t driver;
	if (parser.GetUintParam('P', driver))
	{
		if (driver >= NumDrivers)
		{
			reply.printf("Driver number must be 0-%u", NumDrivers - 1);
			return GCodeResult::error;
		}
		driverNumber = driver;
		seen = true;
	}
	
	if (!seen)
	{
		CopyBasicDetails(reply);
		reply.catf(", driver %u", driverNumber);
	}
	
	return GCodeResult::ok;
}

void TmcDriverActualTemperatureSensor::Poll() noexcept
{
	if (driverNumber >= NumDrivers)
	{
		SetResult(TemperatureError::hardwareError);
		return;
	}
	
	const float temp = SmartDrivers::GetDriverTemperature(driverNumber);
	SetResult(temp, TemperatureError::ok);
}

#endif // SUPPORT_TMC2240 || (SUPPORT_TMC51xx && TMC_TYPE == 2240)

#endif // HAS_SMART_DRIVERS

// End
