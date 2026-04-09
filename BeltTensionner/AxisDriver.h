#pragma once

#include "src/FastAccelStepper/FastAccelStepper.h"
#include "src/FastAccelStepper/AVRStepperPins.h" // Only required for AVR controllers

#define ZeroOffset 3000

struct AxisSettings {
public:
	int hallSensorPin;
	int pulsePin;
	int directionPin;
	int enablePin;
	int totalWorkingRange;
	bool reverseDirection;
	int stepperId;
	uint32_t maxSpeed = 60000;
	uint32_t acceleration = 120000;
	int sensorTriggerLevel = 100;
	bool centerAfterCalibration;
	/// <summary>
	/// Constructor
	/// </summary>
	/// <param name="_motorId"></param>
	/// <param name="_hallSensorPin"></param>
	/// <param name="_pulsePin"></param>
	/// <param name="_dirPin"></param>
	/// <param name="_enablePin"></param>
	/// <param name="_workingRange"></param>
	/// <param name="_reverseDirection"></param>
	AxisSettings(int _stepperId, int _hallSensorPin, int _pulsePin, int _directionPin, int _enablePin, int _totalWorkingRange, bool _reverseDirection, int _sensorTriggerLevel, bool _centerAfterCalibration) {
		stepperId = _stepperId;
		hallSensorPin = _hallSensorPin;
		pulsePin = _pulsePin;
		directionPin = _directionPin;
		enablePin = _enablePin;
		totalWorkingRange = _totalWorkingRange;
		reverseDirection = _reverseDirection;
		sensorTriggerLevel = _sensorTriggerLevel;
		centerAfterCalibration = _centerAfterCalibration;
	}

	/// <summary>
	/// Converts from 0 - 65535 (5000 being the middle) to 0 - working range;
	/// </summary>
	/// <param name="inputPosition">0-65535 position</param>
	/// <returns>Translated position</returns>
	int32_t convertPosition16Bits(uint16_t inputPosition) {
		return (int32_t)constrain(((double)(inputPosition) * (double)totalWorkingRange / (double)65535), 0, totalWorkingRange);
	}
};

struct AxisState {
	bool motorPowered = false;
	int32_t targetPosition = -1;
	unsigned long lastChange = 0;
	unsigned long firstActivity = 0;
	bool finalVelocityApplied = false;
	bool motorReady = false;
	int calibrationPhase = -1;
};

class AxisDriver {
private:

	FastAccelStepper* stepper = NULL;
	AxisSettings* axisSettings;
	AxisState axisState = AxisState();
	String logHeader;
	void PrintLog(String message) {
		Serial.print(logHeader);
		Serial.println(message);
	}

	bool EnableMotor() {
		if (!axisState.motorPowered) {
			axisState.motorReady = false;
			axisState.calibrationPhase = 0;
			PrintLog(F("Enabling motor"));
			digitalWrite(axisSettings->enablePin, LOW);
			axisState.motorPowered = true;
			axisState.targetPosition = -1;
			axisState.lastChange = millis();
		}

		UpdateCalibration();

		return axisState.motorReady;
	}

	bool UpdateCalibration() {
		if (axisState.calibrationPhase != -1) {
			axisState.lastChange = millis();
			axisState.motorReady = false;
			Calibrate();
			return true;
		}
		else {
			axisState.motorReady = true;
			return false;
		}
	}

	void DisableMotor() {
		if (axisState.motorPowered) {
			axisState.motorReady = false;
			PrintLog(F("Disabling motor"));
			digitalWrite(axisSettings->enablePin, HIGH);
			axisState.motorPowered = false;
			axisState.targetPosition = -1;
			axisState.calibrationPhase = -1;
		}
		axisState.firstActivity = 0;
	}

	void calibrationErrorDeath(String message) {
		// Intentionnal dead end
		while (true) {
			DisableMotor();
			PrintLog(message);
			delay(30000);
		}
	}

	void Calibrate() {
		if (axisState.calibrationPhase == 0) {
			PrintLog(F("Starting stepper calibration"));

			stepper->setSpeedInHz(5000);       // 500 steps/s
			stepper->setAcceleration(20000);    // 100 steps/s�
			stepper->setCurrentPosition(0);
			stepper->moveTo(2000);
			axisState.calibrationPhase = 1;
		}

		if (axisState.calibrationPhase == 1) {
			if (!stepper->isRunning()) {
				PrintLog(F("Initial move finished"));
				axisState.calibrationPhase = 2;
			}
		}

		if (axisState.calibrationPhase == 2) {
			PrintLog(F("Starting calibration"));
			stepper->moveTo(-axisSettings->totalWorkingRange * 3);
			axisState.calibrationPhase = 3;
			delay(10);
			int val = ReadHallSensor();
		}

		if (axisState.calibrationPhase == 3) {
			int val = ReadHallSensor();
			if (val > axisSettings->sensorTriggerLevel) {
				calibrationErrorDeath(F("Calibration ERROR, hall sensor is already sensing the lever, please check hardware and reboot device."));
			}
			axisState.calibrationPhase = 4;
		}

		if (axisState.calibrationPhase == 4) {
			int val = ReadHallSensor();
			if (stepper->isRunning()) {
				if (val > axisSettings->sensorTriggerLevel) {
					Serial.println(val);
					PrintLog(F("Sensor triggered"));
					stepper->forceStopAndNewPosition(ZeroOffset);

					MoveToIdle(false);

					axisState.calibrationPhase = -1;
					PrintLog(F("Calibration successful."));
					return;
				}
			}
			else {
				calibrationErrorDeath(F("Calibration ERROR, lever not found, please check hardware and reboot device."));
			}
		}

		//setSpeedSettings();
	}

	void MoveToIdle(bool blocking)
	{
		stepper->setSpeedInHz(4000);       // 500 steps/s
		stepper->setAcceleration(2000);    // 100 steps/s�

		if (axisSettings->centerAfterCalibration) {
			stepper->moveTo(((int)axisSettings->totalWorkingRange / (double)2) + ZeroOffset, blocking);
		}
		else {
			stepper->moveTo(ZeroOffset, blocking);
		}
	}

	void setSpeedSettings() {
		stepper->setSpeedInHz(axisSettings->maxSpeed);       // 25000 steps/s
		stepper->setAcceleration(axisSettings->acceleration);    // 80000 steps/s�
	}

public:

	int ReadHallSensor() {
		int val = analogRead(axisSettings->hallSensorPin);
		val += analogRead(axisSettings->hallSensorPin);
		val += analogRead(axisSettings->hallSensorPin);
		val = val / 3;
		return abs(val - 512);
	}

	void begin(FastAccelStepperEngine* engine, AxisSettings* _axisSettings, bool calibrateAtBoot) {
		logHeader = "M" + String(axisSettings->stepperId);
		axisSettings = _axisSettings;
		pinMode(axisSettings->hallSensorPin, INPUT);
		pinMode(axisSettings->enablePin, OUTPUT);
		pinMode(axisSettings->directionPin, OUTPUT);

		// Start motor disabled.
		digitalWrite(axisSettings->enablePin, HIGH);

		stepper = engine->stepperConnectToPin(axisSettings->pulsePin);

		if (stepper) {
			stepper->setDirectionPin(axisSettings->directionPin, axisSettings->reverseDirection);
			stepper->setSpeedInHz(25000);       // 500 steps/s
			stepper->setAcceleration(15000);    // 100 steps/s�
		}

		if (calibrateAtBoot) {
			EnableMotor();
		}
	}

	void setPosition16Bits(int value) {
		int32_t newPosition = axisSettings->convertPosition16Bits(value);

		if (EnableMotor()) {
			unsigned long time = millis();

			if (axisState.firstActivity == 0) {
				axisState.firstActivity = time;
				axisState.finalVelocityApplied = false;
			}

			if (time - axisState.firstActivity < 6000) {
				stepper->setSpeedInHz((int)((double)(axisSettings->maxSpeed) * constrain((double)(time - axisState.firstActivity) / ((double)(6000)), (double)0, (double)1)));       // 25000 steps/s
				stepper->setAcceleration((int)((double)(axisSettings->acceleration) * constrain((double)(time - axisState.firstActivity) / ((double)(6000)), (double)0, (double)1)));
				stepper->applySpeedAcceleration();
				axisState.finalVelocityApplied = false;
			}
			else if (!axisState.finalVelocityApplied) {
				stepper->setSpeedInHz(axisSettings->maxSpeed);
				stepper->setAcceleration(axisSettings->acceleration);
				stepper->applySpeedAcceleration();
				axisState.finalVelocityApplied = true;
			}

			axisState.lastChange = time;

			if (newPosition != axisState.targetPosition) {
				axisState.targetPosition = newPosition;
				stepper->moveTo(newPosition + ZeroOffset);
			}
		}
	}

	void setMaxSpeed16Bits(uint16_t value) {
		axisSettings->maxSpeed = (uint32_t)value;
		PrintLog("Speed set to " + String(axisSettings->maxSpeed));
		axisState.finalVelocityApplied = false;
	}

	void setMaxAcceleration32Bits(uint32_t value) {
		axisSettings->acceleration = (uint32_t)value;
		PrintLog("Acceleration set to " + String(axisSettings->acceleration));
		axisState.finalVelocityApplied = false;
	}

	void update() {
		// automatic motor disable after inactivity unless a calibration is in still in progress
		if (!UpdateCalibration()) {
			if (millis() - axisState.lastChange > idleDelay) {
				if (axisState.firstActivity != 0) {
					axisState.firstActivity = 0;
					MoveToIdle(false);
				}
				if (!stepper->isRunning()) {
					DisableMotor();
				}
			}
		}
		else {
			axisState.lastChange = millis();
		}
	}

	void parkNow() {
		axisState.lastChange = 0;
	}

	void discardCalibration() {
		axisState.motorReady = false;
		axisState.calibrationPhase = 0;
		axisState.lastChange = millis();
	}
};
