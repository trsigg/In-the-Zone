#include "../config/config.c"


goalState currGoalState = IN;

void moveGoalIntake(goalState state, bool runConcurrently=false, int power=127) {
  if (state != currGoalState) {
    currGoalState = state;
    //moveForDuration(goalIntake, 127*(state==IN ? 1 : -1), (state==IN ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION), runConcurrently);  //temp
    createManeuver(goalIntake, goalPos[state], runConcurrently, power, goalIntake.stillSpeed*(state==IN ? 1 : -1));
  }
}

bool isMobileGoalLoaded() {
  return SensorValue[ goalLine[robot] ] < goalLineThresh[robot] || goalLine[robot] < in1;
}

void updateMotorConfig(int goalPower=0) {
  if (!SKILLZ_MODE) {
    if (goalPower == 0 && goalIntake.moving == NO) {
      setNumMotors(drive, NUM_LEFT_MOTORS, NUM_RIGHT_MOTORS);
  		stopPowerLimiting(drive);
    }
    else {
  		setNumMotors(drive, NUM_LEFT_MOTORS-1, NUM_RIGHT_MOTORS-1);

  		bool goalOut = goalPower > 0;
      int oppLimit = (bIfiAutonomousMode ? 30 : 0);
  		setPowerLimits(drive, (goalOut ? -127 : -oppLimit), (goalOut ? oppLimit : 127));
  	}
  }
}

/*void moveGoalIntake(bool in, bool runConcurrently=false, bool checkPos=true) {
  int goalPos = getPosition(goalIntake);
  if (!checkPos || (goalPos<10 && in || goalPos>0 && !in))
    moveForDuration(goalIntake, 127*(in ? 1 : -1), (in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION));

  if (!runConcurrently) waitForMovementToFinish(goalIntake);
}*/
