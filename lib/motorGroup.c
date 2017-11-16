#define numTargets 4

#include "timer.c"
#include "PID.c"

typedef struct {
	tMotor motors[12];
	int numMotors;
	TVexJoysticks posInput, negInput; //inputs. NegInput only assigned if using button control
	//button control
	bool controlActive;
	int upPower, downPower, stillSpeed;
		//complex still speeds
	int stillSpeedSwitchPos, stillSpeedType;	//stillSpeedType is 0 for regular, 1 for posDependent, and 2 for buttonDependent
	bool stillSpeedReversed;
	//execute maneuver
	int targetPos, endPower, maneuverPower, maneuverTimeout;
	bool forward, maneuverExecuting; //forward: whether target is forwad from initial group position
	long maneuverTimer;
	//maintainPos
	PID posPID;	//PID controller which maintains position
	bool activelyMaintining;	//whether position is being maintained
	//sensors
	bool hasEncoder, hasPotentiometer;
	bool encoderReversed, potentiometerReversed;
	bool potentiometerDefault; //whether potentiometer (as opposed to encoder) is default sensor for position measurements
	bool encCorrectionActive;	//drift correction
	int encMax, maxDisp;
	tSensors encoder, potentiometer;
} motorGroup;

//#region initialization
void configureButtonInput(motorGroup *group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->controlActive = true;
	group->posInput = posBtn;
	group->negInput = negBtn;
	group->stillSpeed = stillSpeed;
	group->stillSpeedType = 0;
	group->stillSpeedReversed = false;
	group->upPower = upPower;
	group->downPower = downPower;
}

void initializeGroup(motorGroup *group, int numMotors, tMotor *motors, TVexJoysticks posBtn=-1, TVexJoysticks negBtn=-1, int stillSpeed=0, int upPower=127, int downPower=-127) {
	group->motors = motors;

	if (posBtn >= 0)
		configureButtonInput(group, TVexJoysticks posBtn, TVexJoysticks negBtn, int stillSpeed, int upPower, int downPower);

	group->numMotors = numMotors;
	group->maneuverExecuting = false;
}

void configurePosDependentStillSpeed(motorGroup *group, int stillSpeed, int switchPos) {	//motor will have stillSpeed power when below switchPos, -stillSpeed power when above switchPos
	group->stillSpeed = stillSpeed;
	group->stillSpeedType = 1;
	group->stillSpeedSwitchPos = switchPos;
}

void configureBtnDependentStillSpeed(motorGroup *group, int stillSpeed) {
	group->stillSpeed = stillSpeed;
	group->stillSpeedType = 2;
}
//#endregion

//#region sensors
void addSensor(motorGroup *group, tSensors sensor, bool reversed=false, bool setAsDefault=true) {
	switch (SensorType[sensor]) {
		case sensorPotentiometer:
			group->hasPotentiometer = true;
			group->potentiometer = sensor;
			group->potentiometerReversed = reversed;
			if (setAsDefault) group->potentiometerDefault = true;
			break;
		case sensorQuadEncoder:
			group->hasEncoder = true;
			group->encoder = sensor;
			group->encoderReversed = reversed;
			SensorValue[sensor] = 0;
			if (setAsDefault) group->potentiometerDefault = false;
			break;
	}
}

int encoderVal(motorGroup *group) {
	if (group->hasEncoder) {
		return SensorValue[group->encoder] * (group->encoderReversed ?  -1 : 1);
	} else {
		return 0;
	}
}

int potentiometerVal(motorGroup *group) {
	if (group->hasPotentiometer) {
		return (group->potentiometerReversed ? 4096-SensorValue[group->potentiometer] : SensorValue[group->potentiometer]);
	} else {
		return 0;
	}
}

int getPosition(motorGroup *group) {
	if (group->hasPotentiometer && group->hasEncoder) {
		return group->potentiometerDefault ? potentiometerVal(group) : encoderVal(group);
	} else {
		return (group->hasEncoder ? encoderVal(group) : potentiometerVal(group));
	}
}

void resetEncoder(motorGroup *group, int resetVal=0) {
	if (group->hasEncoder)
		SensorValue[group->encoder] = resetVal * (group->encoderReversed ?  -1 : 1);
}

void configureEncoderCorrection(motorGroup *group, int max) {
	group->encMax = max;
	group->encCorrectionActive = true;
	group->maxDisp = 0;
}

void correctEncVal(motorGroup *group) {
	if (group->encCorrectionActive) {
		int encVal = encoderVal(group);

		if (encVal < group->maxDisp) {
			resetEncoder(group);
			group->maxDisp = 0;
		}
		else if ((encVal - group->encMax) > group->maxDisp) {
			group->maxDisp = encVal - group->encMax;
		}
	}
}
//#endregion

//#region position limiting
void setAbsMax(motorGroup *group, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMax = max;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsMin(motorGroup *group, int min, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->hasAbsMin = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}

void setAbsolutes(motorGroup *group, int min, int max, int defPowerAtAbs=0, int maxPowerAtAbs=20) {
	group->absMin = min;
	group->absMax = max;
	group->hasAbsMin = true;
	group->hasAbsMax = true;
	group->maxPowerAtAbs = maxPowerAtAbs;
	group->defPowerAtAbs = defPowerAtAbs;
}
//#endregion

//#region set and get power
int setPower(motorGroup *group, int power, bool overrideAbsolutes=false) {
	if (!overrideAbsolutes) {
		if (group->hasAbsMin && getPosition(group) <= group->absMin && power < -group->maxPowerAtAbs)
			power = -group->defPowerAtAbs;

		if (group->hasAbsMax && getPosition(group) >= group->absMax && power > group->maxPowerAtAbs)
			power = group->defPowerAtAbs;
	}

	for (int i=0; i<group->numMotors; i++) //set motors
		motor[group->motors[i]] = power;

	return power;
}

int getPower(motorGroup *group) {
	return group->motor[0];
}
//#endregion

//#region position movement
	//#subregion maintainPos
	void initializeTargetingPID(motorGroup *group, float kP, float kI, float kD, int minSampleTime=25, int integralMax=127) {
		initializePID(group->posPID, 0, kP, kI, kD, minSampleTime, integralMax);
	}

	void setTargetingPIDconsts(motorGroup *group, float kP, float kI, float kD) {
		changeGains(group->posPID, kP, kI, kD);
	}

	void setTargetPosition(motorGroup *group, int position, bool resetIntegral=true) {
		changeTarget(group->posPID, position, resetIntegral);
		group->activelyMaintining = true;
	}

	void maintainTargetPos(motorGroup *group, int debugStartCol=-1) {
		if (group->activelyMaintining && group->posPID.kP != 0) {
			setPower(group, PID_runtime(group->posPID, getPosition(group), debugStartCol));
		}
	}

	void stopTargeting(motorGroup *group) { group->activelyMaintining = false; }

	bool errorLessThan(motorGroup *group, int errorMargin) {	//returns true if PID error is less than specified margin
		return abs(group->posPID.target - getPosition(group)) < errorMargin;
	}
	//#endsubregion

int moveTowardPosition(motorGroup *group, int position, int power=127) {
	return setPower(group, power * sgn(position - getPosition(group)));
}

void executeManeuver(motorGroup *group) {
	if (group->maneuverExecuting) {
		if (group->forward == (getPosition(group) < group->targetPos)) {
			group->maneuverTimer = resetTimer();
			setPower(group, group->maneuverPower);
		}

		if (time(group->maneuverTimer) > group->maneuverTimeout) {
			group->maneuverExecuting = false;
			setPower(group, group->endPower);
		}
	}
}

void createManeuver(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=10) {
	group->targetPos = position;
	group->endPower = endPower;
	group->forward = group->targetPos > getPosition(group);
	group->maneuverPower = abs(maneuverPower) * (group->forward ? 1 : -1);
	group->maneuverExecuting = true;
	group->maneuverTimeout = timeout;
	group->maneuverTimer = resetTimer();

	setPower(group, maneuverPower);
}

void goToPosition(motorGroup *group, int position, int endPower=0, int maneuverPower=127, int timeout=100) {
	long posTimer = resetTimer();
	int displacementSign = sgn(position - getPosition(group));
	setPower(group, displacementSign*maneuverPower);

	while (time(posTimer) < timeout) {
		if (sgn(position - getPosition(group)) == displacementSign) posTimer = resetTimer();
	}

	setPower(group, endPower);
}
//#endregion

//#region user input
int takeInput(motorGroup *group, bool setMotors=true) {
	int power = 0;

	if (group->controlActive) {
		if (vexRT[group->posInput] == 1) {
			group->maneuverExecuting = false;
			group->activelyMaintining = false;

			if (group->stillSpeedType == 2)
				group->stillSpeedReversed = false;

			power = group->upPower;
		} else if (vexRT[group->negInput] == 1) {
			group->maneuverExecuting = false;
			group->activelyMaintining = false;

			if (group->stillSpeedType == 2)
				group->stillSpeedReversed = true;

			power = group->downPower;
		} else {
			if (group->stillSpeedType == 1)
				group->stillSpeedReversed = getPosition(group) > group->stillSpeedSwitchPos;

			executeManeuver(group);

			if (group->maneuverExecuting)
				power = group->maneuverPower;
			else
				power = group->stillSpeed * (group->stillSpeedReversed ? -1 : 1);
		}
	}

	if (setMotors) setPower(group, power);

	return power;
}
//#endregion
