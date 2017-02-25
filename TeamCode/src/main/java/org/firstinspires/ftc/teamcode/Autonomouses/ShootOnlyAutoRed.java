package org.firstinspires.ftc.teamcode.Autonomouses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
		pause(10.0);
		runSpinner(curPowerOfMotorSpinner + .03);
		moveWithEncodersCoast(0.4, 3100, 1.0, 1.0);
		pause(1.5);
		openServoDropper();
		pause(5.0);
		closeServoDropper();
	}
}
