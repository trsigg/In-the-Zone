//#region chain
static float  chain_Kp = 0.14;
static float  chainRequestedValue;
static float  chain_Kd = 0.4;

float chainD;
float chainP;
float lastchainError;
float chainDF;

float  chainSensorCurrentValue;
float  chainError;
float  chainDrive;

bool manual = false;

static int waitChainError = 20;


task chainController() {
	while(true)	{
		while(!manual) {
			// Read the sensor value and scale
			chainSensorCurrentValue = SensorValue[ chainpot ];

			// calculate error
			chainError =  chainRequestedValue - chainSensorCurrentValue;

			// calculate drive
			chainP = (chain_Kp * chainError);

			chainD = chainError- lastchainError;
			chainDF = (chain_Kd * chainD);

			chainDrive = chainP + chainDF;

			// limit drive
			if( chainDrive > 127 )
				chainDrive = 127;
			if( chainDrive < (-127) )
				chainDrive = (-127);

			// send to motor
			motor[rChainMot] = chainDrive;
			motor[lChainMot] = chainDrive;


			lastchainError = chainError;

			// Don't hog cpu
			wait1Msec( 25 );
		}
	}
}
//#endregion


/*static float  lift_Kp = 0.9;
static float  liftRequestedValue;
static float  lift_Kd = 0.005;



float liftD;
float liftP;
float lastliftError;
float liftDF;

float  liftSensorCurrentValue;
float  liftError;
float  liftDrive;

static int waitLiftError = 20;


task liftController()
{

	while( true )
	{
		while(manual == false)
		{
		// Read the sensor value and scale
		liftSensorCurrentValue = SensorValue[ lpot ];

		// calculate error
		liftError =  liftRequestedValue - liftSensorCurrentValue;

		// calculate drive
		liftP = (lift_Kp * liftError);

		liftD = liftError- lastliftError;
		liftDF = (lift_Kd * liftD);

		liftDrive = liftP + liftDF;

		// limit drive
		if( liftDrive > 127 )
			liftDrive = 127;
		if( liftDrive < (-127) )
			liftDrive = (-127);

		// send to motor

		motor[ cbr ] = liftDrive;
		motor[ cbl ] = liftDrive;

		lastliftError = liftError;

		// Don't hog cpu
		wait1Msec( 25 );
		}
	}
}
*/

//#endregion
