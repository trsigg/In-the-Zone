#include "../config/config.c"


goalState currGoalState = IN;

void moveGoalIntake(goalState state, bool runConcurrently=false) {
  if (state != currGoalState) {
    currGoalState = state;
    //moveForDuration(goalIntake, 127*(state==IN ? 1 : -1), (state==IN ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION), runConcurrently);  //temp
    createManeuver(goalIntake, goalPos[state], runConcurrently, GOAL_STILL_SPEED*(state==OUT ? -1 : 1));
  }
}

bool isMobileGoalLoaded() {
  return SensorValue[GOAL_FOLLOWER] < GOAL_FOLL_THRESH || GOAL_FOLLOWER < in1;
}

void updateMotorConfig(int goalPower=0) {
  if (goalPower == 0 && goalIntake.moving == NO) {
    setNumMotors(drive, 3, 3);
		stopPowerLimiting(drive);
  }
  else {
		setNumMotors(drive, 2, 2);

		bool goalOut = goalPower > 0;
		setPowerLimits(drive, (goalOut ? -127 : -30), (goalOut ? 30 : 127));
	}
}

/*void moveGoalIntake(bool in, bool runConcurrently=false, bool checkPos=true) {
  int goalPos = getPosition(goalIntake);
  if (!checkPos || (goalPos<10 && in || goalPos>0 && !in))
    moveForDuration(goalIntake, 127*(in ? 1 : -1), (in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION));

  if (!runConcurrently) waitForMovementToFinish(goalIntake);
}*/
