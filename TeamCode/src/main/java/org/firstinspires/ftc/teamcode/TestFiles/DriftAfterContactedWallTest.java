package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

/**
 * Created by stev3 on 2/21/2017.
 */

@Autonomous (name="DriftAfterContactedWallTest", group="Test")
public class DriftAfterContactedWallTest extends MyAutonomous{

	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		initCurtime();
		double startTime = getCurTime();
		while(getCurTime() - startTime < 2.0)
		{
			super.runOpMode();
			double startAngle = gyro.getYaw();
			double yawDiff = 0.0;
			while(!opModeIsActive() && !isStopRequested())
			{
				yawDiff = getAngleDiff(startAngle,gyro.getYaw());
				boolean isGyroGood = yawDiff < 60.0;
				telemetry.addData("isGyroGood",isGyroGood);
				telemetry.update();
				idle();
			}
			waitForStart();
			initCurtime();
			startTime = getCurTime();
			moveRollersDown();
			pause(rollerMovementTimeDown);
			stopRollers();
			moveWithEncodersCoastWithMaxTimeWithDriftAfterContact(.36, 4300, 8.0, 1.0, 0.8, 15);
		}
	}
}
