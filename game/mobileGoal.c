#include "../config/config.c"


goalState currGoalState = IN;

void moveGoalIntake(goalState state, bool runConcurrently=false) {
  if (state != currGoalState) {
    currGoalState = state;
    createManeuver(goalIntake, goalPos[state], runConcurrently, GOAL_STILL_SPEED*(state==OUT ? -1 : 1));
  }
}

/*void moveGoalIntake(bool in, bool runConcurrently=false, bool checkPos=true) {
  int goalPos = getPosition(goalIntake);
  if (!checkPos || (goalPos<10 && in || goalPos>0 && !in))
    moveForDuration(goalIntake, 127*(in ? 1 : -1), (in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION));

  if (!runConcurrently) waitForMovementToFinish(goalIntake);
}*/
