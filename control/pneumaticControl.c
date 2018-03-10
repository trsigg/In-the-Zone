void handleFbInput() {
	bool intakeClosed = intake.isOpen;	//Confusing, I know. isOpen refers to the solenoid, intakeClosed refers to the intake.

	if (newlyPressed(Btn6U)) {
		setState(fourBar, !intakeClosed);
		setState(intake, !(fbUp && intakeClosed));
	}
	else if (newlyPressed(Btn6D)) {
		setState(fourBar, intakeClosed);
		setState(intake, fbUp || !intakeClosed);
	}
	else {
		takeInput(fourBar);
	}

	fbUp = !fourBar.isOpen;
}
