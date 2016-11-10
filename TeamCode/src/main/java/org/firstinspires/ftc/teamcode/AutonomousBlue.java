package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;


/**
 * Created by Steven on 10/20/2016.
 */
@Autonomous(name = "Autonomous Blue", group = "Test")
public class AutonomousBlue extends MyAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();

		moveToWhiteLine(0.3);
		pause();
//		turnRightToWhiteLine(.25);
//		moveForwardToBeacon(.25);
//		pushButton("Blue");
//		moveAwayFromBeacon(.5, 25);		//needs to be tested
//		gyroTurnRightCorrection(.25, 90);
//		shoot();
	}
}