package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Morgannanez on 10/25/16.
 */


@Autonomous(name="RangeTest", group="Test")
public class RangeTest extends MyOpMode {

    SensorMRRange rangeTest;

    public void initialize()
    {
        rangeTest = new SensorMRRange(hardwareMap.i2cDevice.get("range"));
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("", rangeTest);
            telemetry.update();

			idle();
        }
    }
}