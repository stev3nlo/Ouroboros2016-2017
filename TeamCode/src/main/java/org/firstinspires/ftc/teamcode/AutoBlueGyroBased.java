package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 11/10/2016.
 */

@Autonomous(name = "Autonomous Blue Gyro", group = "Auto")
public class AutoBlueGyroBased extends MyOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		initCurtime();

		moveWithEncoders(.5, 40);	//needs to test goal
		pause();
//		gyroTurnRightCorrection(-.085, 45);
//		pause();
//		moveWithEncoders(1, 200);	//need to test goal
//		pause();
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
