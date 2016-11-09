package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 10/31/2016.
 */
@Autonomous(name = "MyAutonomous Red", group = "Test")
public class AutonomousRed extends MyOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();

		moveToWhiteLine(1);
		turnLeftToWhiteLine(.25);
		moveForwardToBeacon(.25);
		pushButton("Red");
		moveAwayFromBeacon(.5, 25);		//needs to be tested
		gyroTurnLeftCorrection(.25, 90);
		//shoot();
	}
}
