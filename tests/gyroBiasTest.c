#pragma config(Sensor, in3,    hyro,           sensorGyro)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

float initBias;
float cumBias = 0;
float calcdBias;


task main() {
	initBias = SensorBias[hyro];
	SensorType[hyro] = sensorNone;

	wait1Msec(500);

	for (int i=0; i<2000; i++) {
		cumBias += SensorValue[hyro];
		wait1Msec(1);
	}

	calcdBias = cumBias / 2000;

	SensorType[hyro] = sensorGyro;
}
