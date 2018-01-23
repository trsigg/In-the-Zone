void handleGoalIntakeInput() {
	int goalPower = takeInput(goalIntake, false);

	if (getPosition(lift)>liftPos[L_SAFE] || abs(goalPower)<=GOAL_STILL_SPEED)
		setPower(goalIntake, goalPower);
	else
		moveLiftToSafePos(false);
}
