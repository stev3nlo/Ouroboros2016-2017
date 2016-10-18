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

    SensorMRColor colorCenter;
	SensorMRColor colorRear;


    public void initialize(){
		colorCenter = new SensorMRColor(hardwareMap.colorSensor.get("colorCenter"));
		colorRear = new SensorMRColor(hardwareMap.colorSensor.get("colorRear"));

		colorCenter.sensorSetup(0x20);
		colorRear.sensorSetup(0x2a);

		colorCenter.lightOn();
		colorRear.lightOn();
    }

    public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
        while (opModeIsActive()) {
			telemetry.addData("Color Center Values", colorCenter);
            telemetry.addData("Color Center Color", colorCenter.groundColor());
			telemetry.addData("Color Rear Values", colorRear);
			telemetry.addData("Color Rear Color", colorRear.groundColor());
//			if (colorCenter.groundColor().equals("White") && colorRear.groundColor().equals("White")) {
//				telemetry.addData("Aligned", "Yes");
//			} else {
//				telemetry.addData("Aligned", "No");
//			}
			telemetry.update();

			idle();
		}
    }
}
