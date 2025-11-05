/*
 * TmcDriverTemperatureSensor.h
 *
 *  Created on: 8 Jun 2017
 *      Author: David
 */

#ifndef SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_
#define SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_

#include "TemperatureSensor.h"

#if HAS_SMART_DRIVERS

class TmcDriverTemperatureSensor : public TemperatureSensor
{
public:
	TmcDriverTemperatureSensor(unsigned int sensorNum);

	static constexpr const char *TypeName = "drivers";

	void Poll() override;

private:
	static SensorTypeDescriptor typeDescriptor;
};

#if SUPPORT_TMC2240 || (SUPPORT_TMC51xx && TMC_TYPE == 2240)
// This class represents a sensor that reads the actual temperature from a TMC2240 driver
class TmcDriverActualTemperatureSensor : public TemperatureSensor
{
public:
	TmcDriverActualTemperatureSensor(unsigned int sensorNum) noexcept;
	~TmcDriverActualTemperatureSensor() noexcept;

	static constexpr const char *TypeName = "drivertemp";

	void Poll() noexcept override;
	GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) override;

private:
	static SensorTypeDescriptor typeDescriptor;
	uint8_t driverNumber;
};
#endif

#endif

#endif /* SRC_HEATING_SENSORS_TMCDRIVERTEMPERATURESENSOR_H_ */
