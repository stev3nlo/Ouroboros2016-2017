package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;


/**
 * Created by Morgannanez on 10/25/16.
 */


@Disabled//(name="RangeTest", group="Test")
public class RangeTest extends MyOpMode {

    SensorMRRange range;

    public void initialize()
    {
        //range = new SensorMRRange(hardwareMap.i2cDevice.get("range"));
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive())
        {
			telemetry.addData("", range);
            telemetry.update();

			idle();
        }
    }
}