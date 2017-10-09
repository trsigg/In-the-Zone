typedef struct {
		float kP, kI, kD; //tuning coefficients
		float target;
		int minSampleTime; //minimum time between sampling input in milliseconds
		float integralMax; //minimum and maximum error value which will be added to integral
		bool hasMax;
		float output; //can be used to refer to most recent output
		//internal variables
		long lastUpdated;
		float integral;
		float prevError;
} PID;

void initializePID(PID *pid, float target, float kP, float kI, float kD, int minSampleTime=30, float integralMax=0) {
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->target = target;
	pid->minSampleTime = minSampleTime;
	pid->hasMax = integralMax!=0;
	pid->integralMax = integralMax;
	pid->integral = 0;
	pid->prevError = 0;
}

void changeTarget(PID *pid, float target, int resetIntegral=true) {
	pid->prevError += target - pid->target;
	if (resetIntegral) pid->integral = 0;	//TODO: add more options?
	pid->lastUpdated = nPgmTime;
	pid->target = target;
}

void changeGains(PID *pid, float kP, float kI, float kD) {
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
}

void setIntegralMax(PID *pid, float max) {
	pid->hasMax = true;
	pid->integralMax = max;
}

float PID_runtime(PID *pid, float input) {
	long now = nPgmTime;
	long elapsed = now - pid->lastUpdated;

	if (elapsed > pid->minSampleTime) {
		pid->lastUpdated = now;

		float error = pid->target - input;

		if (!pid->hasMax || pid->output<pid->integralMax)
			pid->integral += pid->kI*error;

		pid->output = pid->kP*error + pid->integral + pid->kD*(error - pid->prevError);	//kI factored in above (to avoid problems when resetting gain values)
		pid->prevError = error;
	}

	return pid->output;
}
