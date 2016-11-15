package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/10/2016.
 */

@Autonomous(name = "Autonomous Blue Gyro", group = "Auto")
public class AutoBlueGyroBased extends MyAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		moveWithEncoders(-.2, 500);	//needs to test goal
		pause(1.0);
		gyroTurnRightCorrection(.25, 45);
		pause(1.0);
		moveWithEncoders(1, 200);	//need to test goal
		pause(1.0);
//		moveToWhiteLine(-.085);
//		pause();
//		gyroTurnRightCorrection(-.085, 45);
//		pause();
//		moveForwardToBeacon(-.085);
//		pause();
//		pushButton("Blue");
//		pause();
//		moveAwayFromBeacon(-.085, 25);
//		pause();
//		gyroTurnRightCorrection(-.085, 90);
//		pause();
//		shoot();
	}
}
