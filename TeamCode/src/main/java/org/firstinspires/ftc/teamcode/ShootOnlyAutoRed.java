package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by Steven on 11/11/2016.
 */

@Autonomous(name="Shoot Only Auto Red", group="Auto")
public class ShootOnlyAutoRed extends MyAutonomous
{
	@Override
	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		initTime = getCurTime();
		simpleStabilizingLoop(10.0);
		moveWithEncoders(.3, 2920);
		simpleStabilizingLoop(1.0);
		openServoDropper();
		//initTime = getCurTime();
		simpleStabilizingLoop(0.7);
		closeServoDropper();
		simpleStabilizingLoop(10.0);
		openServoDropper();
		simpleStabilizingLoop(0.7);
		closeServoDropper();
		simpleStabilizingLoop(6.0);
	}
}
