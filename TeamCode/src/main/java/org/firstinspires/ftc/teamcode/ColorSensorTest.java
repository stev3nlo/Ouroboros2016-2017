package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;


/**
 * Created by Morgannanez on 10/10/16.
 */
@Autonomous(name = "ColorSensorTest", group = "Test")
public class ColorSensorTest extends MyOpMode{

    SensorMRColor groundSensor;
	SensorMRColor beaconSensor;


    public void initialize(){
		groundSensor = new SensorMRColor(hardwareMap.colorSensor.get("groundSensor"));
		beaconSensor = new SensorMRColor(hardwareMap.colorSensor.get("beaconSensor"));
    }

    public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Ground Sensor\n", groundSensor.toString());
			telemetry.addData("Beacon Sensor\n", beaconSensor.toString());
			telemetry.addData("Ground Color", groundSensor.groundColor());
			telemetry.addData("Beacon Color", beaconSensor.beaconColor());
			telemetry.update();

			idle();
		}
    }
}
