package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

/**
 * Created by Steven on 11/1/2016.
 */
@Autonomous(name = "BeaconColorSensorTest", group = "Test")
public class BeaconColorSensorTest extends MyAutonomous {

	public void runOpMode() throws InterruptedException {
		super.runOpMode();
		waitForStart();
		while (opModeIsActive()) {
			telemetry.addData("Color Center Values", colorB);
			telemetry.addData("Color Center Beacon", colorB.beaconColor());
			telemetry.update();
			idle();
		}
	}
}
