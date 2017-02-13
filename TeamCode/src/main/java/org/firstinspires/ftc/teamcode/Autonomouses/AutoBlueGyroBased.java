package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/10/2016.
 */

@Disabled//(name = "Autonomous Blue Gyro", group = "Auto")
public class AutoBlueGyroBased extends MyAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		//telemetry.addData("test1","");
		//telemetry.update();
		waitForStart();
		//telemetry.addData("test2","");
		//telemetry.update();
		while(opModeIsActive())
		{
			//motorL1.setPower(.5);
			telemetry.addData("yaw",gyro.getYaw());
			telemetry.update();
			idle();
		}
		//gyroTurnRight(0.6,40.0);
		/*
		initCurtime();
		moveWithEncoders(-.2, 500);	//needs to test goal
		pause(1.0);
		//gyroTurnRightCorrection(.25, 45);
		pause(1.0);
		moveWithEncoders(1, 200);	//need to test goal
		pause(1.0);
		*/
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
