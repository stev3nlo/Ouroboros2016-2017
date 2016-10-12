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

    SensorMRColor colorTest;


    public void initialize(){
		colorTest = new SensorMRColor(hardwareMap.colorSensor.get("colorTest"));
    }

    public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Color Values\n", colorTest.toString());
			telemetry.addData("Approx Color", colorTest.getApproxColor());
			telemetry.update();

			idle();
		}
    }
}
