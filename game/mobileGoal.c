#include "..\config\config.c"

int goalDirection = 0;	//0: not moving; -1: intaking; 1: outtaking

task moveGoalIntakeTask() {
	while (time1(GOAL_TIMER) < (goalDirection==1 ? GOAL_OUTTAKE_DURATION : GOAL_INTAKE_DURATION)) EndTimeSlice();
	setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
	goalDirection = 0;
}

void moveGoalIntake(bool in, bool runAsTask=false) {
	goalDirection = in ? -1 : 1;
	setPower(goalIntake, 127 * goalDirection);

	if (runAsTask) {
		clearTimer(GOAL_TIMER);
		startTask(moveGoalIntakeTask);
	}
	else {
		wait1Msec(in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION);
		setPower(goalIntake, GOAL_STILL_SPEED * goalDirection);
		goalDirection = 0;
	}
}
