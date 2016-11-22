package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Libraries.MROpticalDistanceSensor;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by Morgannanez on 11/22/16.
 */
@Autonomous(name="OpticalDistanceTest", group="Test")
public class OpticalDistanceTest extends MyOpMode {


    public void initialize() {
        ods = new MROpticalDistanceSensor(hardwareMap.opticalDistanceSensor.get("ods"));
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("Light Detected", ods.lightDetected());
            telemetry.addData("Raw Light Detected", ods.rawLightDetected());
            telemetry.addData("Raw Light Detected Max", ods.rawLightDetectedMax());
            telemetry.update();
            idle();
        }
    }
}

