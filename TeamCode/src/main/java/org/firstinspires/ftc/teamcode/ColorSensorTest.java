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

        colorTest = new SensorMRColor("colorTest");
    }
    public void runOpMode() {

        while (opModeIsActive()) {
            telemetry.addData("RGB", colorTest.getColorAndAlpha());
        }
    }

}
