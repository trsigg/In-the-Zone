#include "lift.c"

int goalDirection = 0;	//0: not moving; -1: intaking; 1: outtaking
long goalTimer;

task moveGoalIntakeTask() {
	while (time(goalTimer) < (goalDirection==1 ? GOAL_OUTTAKE_DURATION : GOAL_INTAKE_DURATION)) EndTimeSlice();
	setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
	goalDirection = 0;
}

void moveGoalIntake(bool in, bool runAsTask=false) {
	if (lift.posPID.target < liftPos[L_SAFE])
		moveLiftToSafePos();

	goalDirection = in ? -1 : 1;
	setPower(goalIntake, 127 * goalDirection);

	if (runAsTask) {
		goalTimer = resetTimer();
		startTask(moveGoalIntakeTask);
	}
	else {
		wait1Msec(in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION);
		setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
		goalDirection = 0;
	}
}
