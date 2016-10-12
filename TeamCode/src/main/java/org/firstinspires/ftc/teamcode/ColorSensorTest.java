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

    ColorSensor colorTest;

    public void initialize(){

        colorTest = hardwareMap.colorSensor.get("colorTest");
    }
    public void runOpMode() throws InterruptedException {
		initialize();
		waitForStart();
        while (opModeIsActive()) {
			telemetry.addData("Test", "worked");
            telemetry.addData("Red", colorTest.red());
			telemetry.addData("Green", colorTest.green());
			telemetry.addData("Blue", colorTest.blue());
			telemetry.addData("Alpha", colorTest.alpha());
			telemetry.update();
			idle();
		}
    }

}
