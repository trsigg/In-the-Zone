#include "buttonTracker.c"

enum pneumaticControlType { P_NONE, TOGGLE, TWO_BTN };

typedef struct {
  tSensors solenoids[10];
  int numSolenoids;
  //button control
  TVexJoysticks openOrToggleBtn, closeBtn;
  pneumaticControlType controlScheme;
  bool isOpen;
} pneumaticGroup;

void initializePneumaticGroup(pneumaticGroup *group, int numSolenoids, tSensors sol1, tSensors sol2=dgtl1, tSensors sol3=dgtl1, tSensors sol4=dgtl1, tSensors sol5=dgtl1, tSensors sol6=dgtl1, tSensors sol7=dgtl1, tSensors sol8=dgtl1, tSensors sol9=dgtl1, tSensors sol10=dgtl1) {
  tSensors solenoids[10] = { sol1, sol2, sol3, sol4, sol5, sol6, sol7, sol8, sol9, sol10 };
  for (int i=0; i<numSolenoids; i++)
    group->solenoids[i] = solenoids[i];

  group->numSolenoids = numSolenoids;
  group->isOpen = false;
}

void configureTwoBtnInput(pneumaticGroup *group, TVexJoysticks openBtn, TVexJoysticks closeBtn) {
  group->openOrToggleBtn = openBtn;
  group->closeBtn = closeBtn;
  group->controlScheme = TWO_BTN;
}

void configureToggleInput(pneumaticGroup *group, TVexJoysticks toggleBtn) {
  group->openOrToggleBtn = toggleBtn;
  group->controlScheme = TOGGLE;
}

void setState(pneumaticGroup *group, bool open) {
  for (int i=0; i<group->numSolenoids; i++)
    SensorValue[ group->solenoids[i] ] = open; //true casts to 1, false to 0

  group->isOpen = open;
}

bool takeInput(pneumaticGroup *group) { //returns whether solenoid is open
  switch (group->controlScheme) {
    case TOGGLE:
      if (newlyPressed(group->openOrToggleBtn))
        setState(group, !group->isOpen);
      break;
    case TWO_BTN:
      if (vexRT[group->openOrToggleBtn] == 1)
        setState(group, true);
      else if (vexRT[group->closeBtn] == 1)
        setState(group, false);
      break;
  }

  return group->isOpen;
}
