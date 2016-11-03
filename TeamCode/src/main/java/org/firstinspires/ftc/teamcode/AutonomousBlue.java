package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 10/20/2016.
 */
public class AutonomousBlue extends MyOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();

		moveToWhiteLine(1);
		turnRightToWhiteLine(.25);
		moveForwardToBeacon(.25);
		pushButton("Blue");
		moveAwayFromBeacon(.5, 25);		//needs to be tested
		gyroTurnRightCorrection(.25, 90);
		//shoot();
	}
}