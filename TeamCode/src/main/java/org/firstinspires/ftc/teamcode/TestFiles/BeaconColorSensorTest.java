package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;

/**
 * Created by Steven on 11/1/2016.
 */
@Autonomous(name = "BeaconColorSensorTest", group = "Test")
public class BeaconColorSensorTest extends MyOpMode {

	public void initialize(){
		colorB = new SensorMRColor(hardwareMap.colorSensor.get("colorB"));

		colorB.sensorSetup(0x2c);
		colorB.lightOff();

		servoBeaconPusher = hardwareMap.servo.get(("servoBeaconPusher"));
		resetButtonPress();		//this is where it returns null pointer exception
	}

	public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
		telemetry.addData("Color Center Values", colorB);
		telemetry.addData("Color Center Beacon", colorB.beaconColor());
		telemetry.update();

//		pushButton("Blue");
//		resetButtonPress();
	}
}
