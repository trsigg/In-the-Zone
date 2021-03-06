#include "..\config\config.c"

void pulseRollers(bool intake=false, bool runConcurrently=true) {
  moveForDuration(roller, 127 * (intake ? 1 : -1), (intake ? intakeDuration[robot] : outtakeDuration[robot]), runConcurrently);
}
