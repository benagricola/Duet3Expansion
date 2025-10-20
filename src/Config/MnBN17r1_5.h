/*
 * MnBN17r1_5.h
 *
 *	Created on: 25 Sep 2025
 *		Author: Nine Mile
 */

#ifndef SRC_CONFIG_MNBN17R1_5_H_
#define SRC_CONFIG_MNBN17R1_5_H_

#include <Hardware/PinDescription.h>

#define BOARD_TYPE_NAME			"MNBN17R1_5"
#define BOOTLOADER_NAME			"MNBN17R1_5"

#define BOARD_USES_UF2_BINARY	1

// General features
#define HAS_VREF_MONITOR		0
#define HAS_VOLTAGE_MONITOR		1
#define HAS_12V_MONITOR			0
#define HAS_CPU_TEMP_SENSOR		1
#define HAS_ADDRESS_SWITCHES	0
#define HAS_BUTTONS				0
#define HAS_USB_SERIAL			1
#define USE_SERIAL_DEBUG		0
#define SUPPORT_LED_STRIPS		0
#define SUPPORT_PIO_NEOPIXEL	0
#define SUPPORT_INPUT_SHAPING	1

// Drivers configuration
#define SUPPORT_DRIVERS			1

#if SUPPORT_DRIVERS

#define HAS_SMART_DRIVERS		1
#define HAS_STALL_DETECT		1
#define SINGLE_DRIVER			1
#define SUPPORT_SLOW_DRIVERS	0
#define SUPPORT_DELTA_MOVEMENT	0

#define SUPPORT_TMC51xx			0
#define SUPPORT_TMC2660			0
#define SUPPORT_TMC22xx			1			// TMC2240 UART mode (current default)
#define SUPPORT_TMC2240			1
#define SUPPORT_TMC2209			0

constexpr size_t NumDrivers = 1;
constexpr size_t MaxSmartDrivers = 1;

#define TMC22xx_HAS_MUX					0
#define TMC22xx_SINGLE_DRIVER			1
#define TMC22xx_HAS_ENABLE_PINS			1
#define TMC22xx_VARIABLE_NUM_DRIVERS	0
#define TMC22xx_USE_SLAVEADDR			0

// TMC2240 uses UART communication in current firmware implementation
// Define the baud rate used to send/receive data to/from the drivers.
constexpr uint32_t DriversBaudRate = 200000;		// UART baud rate for TMC2240
constexpr uint32_t TransferTimeout = 10;			// any transfer should complete within 10 ticks @ 1ms/tick

// TMC2240 current sense resistor and scaling
constexpr uint32_t Tmc2240CurrentRange = 0x03;								// which current range we set the TMC2240 to (3A)
constexpr uint32_t Tmc2240SlopeControl = 0x01;								// which slope control we set the TMC2240 to (200V/us)
constexpr float Tmc2240Rref = 12.0;											// TMC2240 reference resistor in Kohms
constexpr float DriverFullScaleCurrent = 36000/Tmc2240Rref;					// in mA, assuming we set the range bits in the DRV_CONF register to 0x03
constexpr float DriverCsMultiplier = 32.0/DriverFullScaleCurrent;

constexpr float MaximumMotorCurrent = 2500.0;		// TMC2240
constexpr float MaximumStandstillCurrent = 1500;

constexpr uint32_t DefaultStandstillCurrentPercent = 75;


constexpr Pin DriverSelectPins[NumDrivers] = { GpioPin(5) }; // GPIO5 DRV_CS
constexpr Pin Tmc22xxUartPins[NumDrivers]  = { GpioPin(1) }; // GPIO1 DRV_UART
constexpr Pin EnablePins[NumDrivers]       = { GpioPin(0) }; // GPIO0 DRV_EN
constexpr Pin StepPins[NumDrivers]         = { GpioPin(3) }; // GPIO3 DRV_STEP
constexpr Pin DirectionPins[NumDrivers]    = { GpioPin(2) }; // GPIO2 DRV_DIR

constexpr Pin GlobalTmc22xxEnablePin = EnablePins[0];
constexpr Pin Tmc22xxUartPin         = Tmc22xxUartPins[0];

#if HAS_STALL_DETECT
constexpr Pin DriverDiagPins[NumDrivers]   = { GpioPin(12) }; // GPIO12 DRV_DIAG
#endif

#define ACTIVE_HIGH_STEP		1		// 1 = active high, 0 = active low
#define ACTIVE_HIGH_DIR			1		// 1 = active high, 0 = active low

#endif

#define SUPPORT_THERMISTORS		0
#define SUPPORT_SPI_SENSORS		0
#define SUPPORT_I2C_SENSORS		0
#define SUPPORT_LIS3DH			0
#define SUPPORT_DHT_SENSOR		0
#define SUPPORT_LDC1612			0

#define USE_MPU					0
#define USE_CACHE				0

#define PIN_TODO				GpioPin(NoPin)	//TEMPORARY! Used when we haven't assigned a pin yet.

constexpr bool UseAlternateCanPins = false;

constexpr size_t MaxPortsPerHeater = 1;

constexpr size_t NumThermistorInputs = 0;	// No thermistor inputs on this board
constexpr float DefaultThermistorSeriesR = 4700.0;

constexpr Pin TempSensePins[1] = { PIN_TODO };	// No temperature sensor pins

// CAN pin assignments
constexpr Pin CanTxPin = GpioPin(15);	// GPIO15 CAN_MCU_TX
constexpr Pin CanRxPin = GpioPin(14);	// GPIO14 CAN_MCU_RX


constexpr Pin ButtonPins[] = { PIN_TODO };

// VIN voltage monitor
#if HAS_VOLTAGE_MONITOR
constexpr Pin VinMonitorPin = GpioPin(26);	// GPIO26 PVIN_REF
constexpr float VinDividerRatio = (100.0 + 5.1)/5.1;
constexpr float VinMonitorVoltageRange = VinDividerRatio * 3.3;
constexpr size_t VinReadingsAveraged = 8;
#endif

// Diagnostic LEDs
constexpr Pin LedPins[] = { GpioPin(23) };		// STATUS -> GPIO23
constexpr bool LedActiveHigh = true;

// Table of pin functions that we are allowed to use
constexpr PinDescription PinTable[] =
{
	//	PWM				ADC				    PinName
	// GPIO0-GPIO29
	{ PwmOutput::pwm0a,	AdcInput::none,		nullptr		},	// GPIO0  DRV_EN
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO1  DRV_UART
	{ PwmOutput::pwm1a,	AdcInput::none,		nullptr		},	// GPIO2  DRV_DIR
	{ PwmOutput::pwm1b,	AdcInput::none,		nullptr		},	// GPIO3  DRV_STEP
	{ PwmOutput::pwm2a,	AdcInput::none,		nullptr		},	// GPIO4  DRV_MISO
	{ PwmOutput::pwm2b,	AdcInput::none,		nullptr		},	// GPIO5  DRV_CS
	{ PwmOutput::pwm3a,	AdcInput::none,		nullptr		},	// GPIO6  DRV_SCK
	{ PwmOutput::pwm3b,	AdcInput::none,		nullptr		},	// GPIO7  DRV_MOSI
	{ PwmOutput::pwm4a,	AdcInput::none,		nullptr		},	// GPIO8  CAN_MISO
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO9  CAN_CS
	{ PwmOutput::pwm5a,	AdcInput::none,		nullptr		},	// GPIO10 CAN_SCK
	{ PwmOutput::pwm5b,	AdcInput::none,		nullptr		},	// GPIO11 CAN_MOSI
	{ PwmOutput::pwm6a,	AdcInput::none,		"diag"		},	// GPIO12 DRV_DIAG
	{ PwmOutput::pwm6b,	AdcInput::none,		nullptr		},	// GPIO13 CAN_INT
	{ PwmOutput::pwm7a,	AdcInput::none,		nullptr		},	// GPIO14 CAN_MCU_RX
	{ PwmOutput::pwm7b,	AdcInput::none,		nullptr		},	// GPIO15 CAN_MCU_TX
	{ PwmOutput::pwm0a,	AdcInput::none,		"aux"		},	// GPIO16 AUX
	{ PwmOutput::pwm0b,	AdcInput::none,		nullptr		},	// GPIO17 NC
	{ PwmOutput::pwm1a,	AdcInput::none,		"stop"		},	// GPIO18 STOP
	{ PwmOutput::pwm1b,	AdcInput::none,		"ext1"		},	// GPIO19 EXT1
	{ PwmOutput::pwm2a,	AdcInput::none,		"ext2"		},	// GPIO20 EXT2
	{ PwmOutput::pwm2b,	AdcInput::none,		"ext3"		},	// GPIO21 EXT3
	{ PwmOutput::pwm3a,	AdcInput::none,		"ext4"		},	// GPIO22 EXT4
	{ PwmOutput::pwm3b,	AdcInput::none,		nullptr		},	// GPIO23 STATUS
	{ PwmOutput::pwm4a,	AdcInput::none,		nullptr		},	// GPIO24 NC
	{ PwmOutput::pwm4b,	AdcInput::none,		nullptr		},	// GPIO25 NC
	{ PwmOutput::pwm5a,	AdcInput::adc0_0,	"vin"		},	// GPIO26 PVIN_REF
	{ PwmOutput::pwm5b,	AdcInput::adc0_1,	nullptr		},	// GPIO27 NC
	{ PwmOutput::pwm6a,	AdcInput::adc0_2,	nullptr		},	// GPIO28 NC
	{ PwmOutput::none,	AdcInput::none, 	nullptr     },	// GPIO29 DRV_UART_ENA
};

constexpr size_t NumPins = ARRAY_SIZE(PinTable);
static constexpr size_t NumRealPins = 30;				// 30 GPIO pins on RP2040
constexpr size_t NumVirtualPins = 0;					// No virtual pins on this board

static_assert(NumPins == NumRealPins + NumVirtualPins);

// Timer/counter used to generate step pulses and other sub-millisecond timings
constexpr unsigned int StepTimerAlarmNumber = 0;
constexpr unsigned int StepTcIRQn = TIMER0_IRQ_0;

// Available UART ports
#define NUM_SERIAL_PORTS		1
//constexpr IRQn Serial0_IRQn = SERCOM5_IRQn;

// DMA channel assignments
constexpr DmaChannel DmacChanCAN   = 0;					// this must match the value used in the RP2040 CAN driver in CoreN2G!
constexpr DmaChannel DmacChanAdcRx = 1;
constexpr DmaChannel DmacChanTmcTx = 2;
constexpr DmaChannel DmacChanTmcRx = 3;					// this must be one higher than DmacChanTmcTx for RP2040 build configurations
constexpr DmaChannel DmacChanCRC   = 4;
constexpr DmaChannel DmaChanWS2812 = 5;
constexpr unsigned int NumDmaChannelsUsed = 6;			// must be at least the number of channels used, may be larger. Max 12 on the RP2040.

// DMA priorities, higher is better. RP2040 has only 0 and 1.
constexpr DmaPriority DmacPrioTmcTx = 0;
constexpr DmaPriority DmacPrioTmcRx = 1;
constexpr DmaPriority DmacPrioAdcRx = 1;

// Interrupt priorities, lower means higher priority. Only 0 to 3 are available.
const NvicPriority NvicPriorityStep = 1;				// step interrupt is next highest, it can preempt most other interrupts
const NvicPriority NvicPriorityUart = 2;				// serial driver makes RTOS calls
const NvicPriority NvicPriorityPins = 2;				// priority for GPIO pin interrupts
const NvicPriority NvicPriorityI2C  = 2;
const NvicPriority NvicPriorityCan  = 3;
const NvicPriority NvicPriorityDmac = 3;				// priority for DMA complete interrupts
const NvicPriority NvicPriorityAdc  = 3;
const NvicPriority NvicPriorityUSB  = 3;

#if SUPPORT_CAN && USE_SPICAN

// CAN controller SPI configuration based on netlist
constexpr uint8_t spiCanSpiInstanceNumber = 1;
constexpr Pin SPICanMosiPin = GpioPin(11);			// CAN_MOSI -> GPIO11
constexpr GpioPinFunction SPICanMosiPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanSclkPin = GpioPin(10);			// CAN_SCK -> GPIO10
constexpr GpioPinFunction SPICanSclkPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanMisoPin = GpioPin(8);			// CAN_MISO -> GPIO8
constexpr GpioPinFunction SPICanMisoPinPeriphMode = GpioPinFunction::Spi;
constexpr Pin SPICanCsPin = GpioPin(9);				// CAN_CS -> GPIO9
constexpr Pin SPICanIntPin = GpioPin(13);			// CAN_INT -> GPIO17

#endif
#endif /* SRC_CONFIG_MNBN17R1_5_H_ */
