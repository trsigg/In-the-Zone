#include "../config/config.c"

void moveGoalIntake(bool in, bool runConcurrently=false) {
  moveForDuration(groups[GOAL], 127*(in ? 1 : -1), (in ? GOAL_INTAKE_DURATION : GOAL_OUTTAKE_DURATION));
}
