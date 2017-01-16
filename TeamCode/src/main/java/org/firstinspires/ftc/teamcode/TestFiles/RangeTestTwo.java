package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Morgannanez on 10/25/16.
 */


@Autonomous(name="RangeTest2", group="Test")
public class RangeTestTwo extends MyAutonomous {

	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		while (opModeIsActive())
		{

			telemetry.addData("Range F", rangeF.getOpticDistance());
			telemetry.addData("Range B", rangeB.getOpticDistance());
			telemetry.update();

			idle();
		}
	}
}