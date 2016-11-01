package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

/**
 * Created by Steven on 11/1/2016.
 */
@Autonomous(name = "BeaconColorSensorTest", group = "Test")
public class BeaconColorSensorTest extends MyOpMode {

	SensorMRColor colorB;


	public void initialize(){
		colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));

		colorB.sensorSetup(0x2c);
		colorB.lightOff();
	}

	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
		while (opModeIsActive()) {
			telemetry.addData("Color Center Values", colorB);
			telemetry.addData("Color Center Beacon", colorB.beaconColor());
			telemetry.update();

			idle();
		}
	}
}
