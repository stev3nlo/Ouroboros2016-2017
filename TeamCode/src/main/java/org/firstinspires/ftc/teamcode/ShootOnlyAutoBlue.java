package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="Shoot Only Auto Blue", group="Auto")
public class ShootOnlyAutoBlue extends MyAutonomous {
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		initCurtime();
		initTime = getCurTime();
		runSpinner(curPowerOfMotorSpinner);
		moveWithEncoders(.2, 2500);
		pause(1.0);
		initCurtime();
		double startOfStablilizing = getCurTime();
		while (startOfStablilizing - getCurTime() < 5.0) {
			runRPMStabilizationAuto();
			initCurtime();
		}
		openServoDropper();
		pause(0.75);
		closeServoDropper();
	}
}
