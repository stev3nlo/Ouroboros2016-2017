package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Steven on 11/10/2016.
 */

@Autonomous(name = "Autonomous Blue Gyro", group = "Auto")
public class AutoBlueGyroBased extends MyAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		initCurtime();
		initTime = getCurTime();
		runSpinner(curPowerOfMotorSpinner);
		moveWithEncoders(.2, 2500);	//needs to test goal
		pause(1.0);
		//gyroTurnRightCorrection(.25, 90);
		//pause(1.0);
		initCurtime();
		double startOfStabilizing = getCurTime();
		while(startOfStabilizing - getCurTime() < 5.0)
		{
			runRPMStabilizationAuto();
			initCurtime();
		}
		openServoDropper();
		pause(0.75);
		closeServoDropper();
		/*initCurtime();
		startOfStabilizing = getCurTime();
		while(startOfStabilizing - getCurTime() < 5.0)
		{
			runRPMStabilizationAuto();
			initCurtime();
		}
		openServoDropper();
		pause(0.75);
		closeServoDropper();*/
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
