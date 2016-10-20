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
		turnRightToWhiteLine(1);
		//move forward until x distance from wall
		//needs to write code for distance sensor first
		pushButton("Blue");
	}
}