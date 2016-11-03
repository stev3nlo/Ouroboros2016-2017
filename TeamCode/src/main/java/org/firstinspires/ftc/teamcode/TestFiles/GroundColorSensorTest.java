package org.firstinspires.ftc.teamcode.TestFiles;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRColor;


/**
 * Created by Morgannanez on 10/10/16.
 */
@Autonomous(name = "GroundColorSensorTest", group = "Test")
public class GroundColorSensorTest extends MyOpMode{


    public void initialize(){
		colorC = new SensorMRColor(hardwareMap.colorSensor.get("colorC"));
		colorR = new SensorMRColor(hardwareMap.colorSensor.get("colorR"));

		colorC.sensorSetup(0x2e);
		colorR.sensorSetup(0x2a);

		colorC.lightOn();
		colorR.lightOn();
    }

    public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
        while (opModeIsActive()) {
			telemetry.addData("Color Center Values", colorC);
            telemetry.addData("Color Center Color", colorC.groundColor());
			telemetry.addData("Color Rear Values", colorR);
			telemetry.addData("Color Rear Color", colorR.groundColor());
			if (colorC.groundColor().equals("White") && colorR.groundColor().equals("White")) {
				telemetry.addData("Aligned", "Yes");
			} else {
				telemetry.addData("Aligned", "No");
			}
			telemetry.update();

			idle();
		}
    }
}
