package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;
import org.firstinspires.ftc.teamcode.Libraries.SensorMRRange;

/**
 * Created by Morgannanez on 11/15/16.
 */

public class AutoRangeTest extends MyOpMode {

    SensorMRRange rangeF;
    SensorMRRange rangeB;

    public void initialize()
    {

        rangeF = new SensorMRRange(hardwareMap.i2cDevice.get("rangeF"));
        rangeB = new SensorMRRange(hardwareMap.i2cDevice.get("rangeB"));
    }

    public void runOpMode() throws InterruptedException {
        initialize();
        waitForStart();
        while (opModeIsActive())
        {
            moveForwards(1.0);
            telemetry.addData("Range F ", rangeF);
            telemetry.addData("Range B ", rangeB);
            telemetry.update();

            idle();
        }

        while (opModeIsActive())
        {
            while(!(rangeF.getUltraSonicDistance() == rangeB.getUltraSonicDistance()))
            {
                move(0.0, 1.0);
                telemetry.addData("Range F ", rangeF);
                telemetry.addData("Range B ", rangeB);
                telemetry.update();

            }
            telemetry.addData("Range F ", rangeF);
            telemetry.addData("Range B ", rangeB);
            telemetry.update();

            idle();
        }



        while (opModeIsActive())
        {
            moveBackToWhiteLineODS(1.0);
            telemetry.addData("Range F ", rangeF);
            telemetry.addData("Range B ", rangeB);
            telemetry.update();
        }
    }
}
