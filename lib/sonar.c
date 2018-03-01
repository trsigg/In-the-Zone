#include "coreIncludes.c"


distUnits getSonarType(tSensors sonar) {
  switch (SensorType[sonar]) {
		case sensorSONAR_TwoPins_mm:
			return MM;
			break;
		case sensorSONAR_TwoPins_cm:
			return CM;
			break;
		case sensorSONAR_TwoPins_inch:
			return INCH;
			break;
		default:	//sensorSONAR_TwoPins_raw:
			return RAW_DIST;
			break;
	}
}

bool sonarFartherThan(tSensors sonar, float dist, bool countNegAsFar=true, distUnits units=RAW_DIST) {
  int sonarDist = SensorValue[sonar]; //TODO: convertDist(SensorValue[sonar], units, getSonarType(sonar));

  return (sonarDist == -1) && countNegAsFar || (sonarDist > dist);
}
