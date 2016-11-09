package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 10/20/2016.
 */
@Autonomous(name = "Autonomous Blue", group = "Test")
public class AutonomousBlue extends MyOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
		initCurtime();

		moveToWhiteLine(0.8);
		pause();
//		turnRightToWhiteLine(.25);
//		moveForwardToBeacon(.25);
//		pushButton("Blue");
//		moveAwayFromBeacon(.5, 25);		//needs to be tested
//		gyroTurnRightCorrection(.25, 90);
//		shoot();
	}
}