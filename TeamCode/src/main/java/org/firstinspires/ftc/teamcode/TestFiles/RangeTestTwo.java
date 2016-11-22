package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Morgannanez on 10/25/16.
 */


@Autonomous(name="RangeTest2", group="Test")
public class RangeTestTwo extends MyOpMode {

	SensorMRRange rangeF;
	SensorMRRange rangeB;
	
	public void initialize()
	{

		 rangeF = new SensorMRRange(hardwareMap.i2cDevice.get("rangeF"));
		 rangeB = new SensorMRRange(hardwareMap.i2cDevice.get("rangeB"));
		rangeF.sensorSetup(0x4a);
		rangeB.sensorSetup(0x4c);
	}

	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
		while (opModeIsActive())
		{

			telemetry.addData("Range F", rangeF);
			telemetry.addData("Range B", rangeB);
			telemetry.update();

			idle();
		}
	}
}