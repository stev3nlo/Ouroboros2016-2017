package org.firstinspires.ftc.teamcode.Autonomouses;

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
		waitForStart();
		initCurtime();
		initTime = getCurTime();
		simpleStabilizingLoop(1.0);
		moveWithEncoders(-.3, 600);
		simpleStabilizingLoop(1.0);
		openServoDropper();
		initTime = getCurTime();
		simpleStabilizingLoop(1.5);
		closeServoDropper();
		/*simpleStabilizingLoop(5.0);
		openServoDropper();
		simpleStabilizingLoop(0.7);
		closeServoDropper();
		simpleStabilizingLoop(6.0);*/
	}
}
