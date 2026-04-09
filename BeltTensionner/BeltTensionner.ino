// ---------------------------------------------------------------------
// ---------------------- SETTINGS SECTION -----------------------------
// ---------------------------------------------------------------------

// GLOBAL SETTINGS : How many steps maximum the stepper can travel
const int totalWorkingRange = 10000;

// Sensor sensitivity
// lower value will make it more sensitive, higher less sensitive (default 100)
// tune this setting in case of issue
const int sensorTriggerLevel = 100;

// Simply test the sensor values. Use the arduino serial monitor (250000 bps) and move the lever to check the values and triggers)
// In this mode the motors will always kept disabled and no motion will occur
const bool sensorTestMode = false;

// Position after calibration, either centered or fully released
const bool centerAfterCalibration = false;

// Position after calibration, either centered or fully released
const bool calibrateAtBoot = false;

// Inactivity delay after the actuators will be parked.
const unsigned long idleDelay = 5000;

// STEPPER 1 SETTINGS
const int stepper1_pulsePin = 9; // Must be either 9 or 10
const int stepper1_directionPin = 3;
const int stepper1_enablePin = 2;
const int stepper1_hallSensorPin = A1; // Must be an analog pin
const int stepper1_reverseDirection = false;

// STEPPER 2
#define ENABLE_STEPPER_2;
#if defined(ENABLE_STEPPER_2)
const int stepper2_pulsePin = 10; // Must be either 9 or 10
const int stepper2_directionPin = 5;
const int stepper2_enablePin = 4;
const int stepper2_hallSensorPin = A2; // Must be an analog pin
const int stepper2_reverseDirection = true;
#endif

// ---------------------------------------------------------------------
// ---------------------------------------------------------------------
// ---------------------------------------------------------------------

#include "src/FastAccelStepper/FastAccelStepper.h"
#include "src/FastAccelStepper/AVRStepperPins.h"
#include "AxisDriver.h"
#define FirmwareVersion "2.0"

FastAccelStepperEngine engine = FastAccelStepperEngine();

AxisDriver axisDriver1 = AxisDriver();
AxisSettings axisSettings1 = AxisSettings(
	1,
	stepper1_hallSensorPin,
	stepper1_pulsePin,
	stepper1_directionPin,
	stepper1_enablePin,
	totalWorkingRange,
	stepper1_reverseDirection,
	sensorTriggerLevel,
	centerAfterCalibration);

#if defined(ENABLE_STEPPER_2)
AxisDriver axisDriver2 = AxisDriver();
AxisSettings axisSettings2 = AxisSettings(
	2,
	stepper2_hallSensorPin,
	stepper2_pulsePin,
	stepper2_directionPin,
	stepper2_enablePin,
	totalWorkingRange,
	stepper2_reverseDirection,
	sensorTriggerLevel,
	centerAfterCalibration);
#endif

AxisDriver* axisDrivers[] = {
   &axisDriver1
#if defined(ENABLE_STEPPER_2)
	,&axisDriver2
#endif
};

AxisSettings* axisSettings[] = {
   &axisSettings1
#if defined(ENABLE_STEPPER_2)
	,&axisSettings2
#endif
};

const int axisDriversCount = sizeof(axisDrivers) / sizeof(AxisDriver*);

void setup() {
	Serial.begin(250000);
	Serial.print(String(axisDriversCount));
	Serial.println(F(" steppers enabled"));

	engine.init();

	for (int i = 0; i < axisDriversCount; i++) {
		axisDrivers[i]->begin(&engine, axisSettings[i], calibrateAtBoot);
	}
}

byte readByte() {
	while (!Serial.available()) {}
	return Serial.read();
}

bool CheckSequence(byte b1, byte b2) {
	// 0xFF 0xFF
	if (b1 != readByte()) return false;
	if (b2 != readByte()) return false;

	return true;
}

void DumpSensor(int axis) {
	int sensorValue = axisDrivers[axis]->ReadHallSensor();
	Serial.print(F("Sensor #"));
	Serial.print(axis);
	Serial.print(':');
	Serial.print(sensorValue);
	Serial.print(F(":Trigger level:"));
	Serial.print(sensorTriggerLevel);
	Serial.print(F(":Triggered:"));
	Serial.print(sensorValue > sensorTriggerLevel ? 1 : 0);
	Serial.println();
}

#define read16Bits() ((uint16_t)(((uint16_t)readByte()) << 8) + (uint16_t)readByte())
#define read32Bits() ((uint32_t)( (((uint32_t)read16Bits() )<< 16) + (uint32_t)read16Bits() )   )

void readCommand() {
  if (!CheckSequence(0xFF, 0xFF)) return;

  byte cmd = readByte();

  // ---------------------------------------------------
  // cmd = 1 : POSITION (normal motion)
  // ---------------------------------------------------
  if (cmd == 1) {
    int val1 = read16Bits();
    int val2 = read16Bits();

    if (!CheckSequence(0x0A, 0x0D)) return;

    axisDrivers[0]->setPosition16Bits(val1);
    if (axisDriversCount > 1) {
      axisDrivers[1]->setPosition16Bits(val2);
    }
    return;
  }

  // ---------------------------------------------------
  // cmd = 2 : MAX SPEED (BLOCKED)
  // We consume bytes but ignore the command
  // ---------------------------------------------------
  if (cmd == 2) {
    (void)read16Bits();
    (void)read16Bits();
    (void)CheckSequence(0x0A, 0x0D);
    return;
  }

  // ---------------------------------------------------
  // cmd = 3 : ACCELERATION (BLOCKED)
  // We consume bytes but ignore the command
  // ---------------------------------------------------
  if (cmd == 3) {
    (void)read32Bits();
    (void)read32Bits();
    (void)CheckSequence(0x0A, 0x0D);
    return;
  }

  // ---------------------------------------------------
  // cmd = 10 : REPORT ENABLED MOTORS
  // ---------------------------------------------------
  if (cmd == 10) {
    if (!CheckSequence(0x0A, 0x0D)) return;

    Serial.print(F("Enabled motors:"));
    Serial.println(axisDriversCount);
    return;
  }

  // ---------------------------------------------------
  // cmd = 11 : DUMP SENSOR VALUE
  // ---------------------------------------------------
  if (cmd == 11) {
    int axis = readByte();
    if (!CheckSequence(0x0A, 0x0D)) return;

    if (axis < axisDriversCount) {
      DumpSensor(axis);
    }
    return;
  }

  // ---------------------------------------------------
  // cmd = 12 : PARK NOW
  // ---------------------------------------------------
  if (cmd == 12) {
    if (!CheckSequence(0x0A, 0x0D)) return;

    for (int i = 0; i < axisDriversCount; i++) {
      axisDrivers[i]->parkNow();
    }
    return;
  }

  // ---------------------------------------------------
  // cmd = 13 : DISCARD CALIBRATION
  // ---------------------------------------------------
  if (cmd == 13) {
    if (!CheckSequence(0x0A, 0x0D)) return;

    for (int i = 0; i < axisDriversCount; i++) {
      axisDrivers[i]->discardCalibration();
    }
    return;
  }

  // ---------------------------------------------------
  // cmd = 14 : FIRMWARE VERSION
  // ---------------------------------------------------
  if (cmd == 14) {
    if (!CheckSequence(0x0A, 0x0D)) return;

    Serial.println(F(FirmwareVersion));
    return;
  }
}


void loop() {
	if (sensorTestMode) {
		for (int i = 0; i < axisDriversCount; i++) {
			DumpSensor(i);
		}
		delay(500);
		return;
	}

	if (Serial.available()) {
		readCommand();
	}

	for (int i = 0; i < axisDriversCount; i++) {
		axisDrivers[i]->update();
	}
}