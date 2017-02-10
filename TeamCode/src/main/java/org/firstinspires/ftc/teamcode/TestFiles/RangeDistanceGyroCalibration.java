package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

@Disabled//(name="RangeDistanceGyroCalibration", group="Test")
public class RangeDistanceGyroCalibration extends MyAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        double USDF = -1;
        double USDB = -1;
        while(!opModeIsActive() && !isStopRequested())
        {
            USDF = rangeF.getUltraSonicDistance();
            USDB = rangeB.getUltraSonicDistance();
            telemetry.addData("USDF",USDF);
            telemetry.addData("USDB",USDB);
            telemetry.update();
            idle();
        }
        waitForStart();
        double startAngle = gyro.getYaw();
        while(opModeIsActive())
        {
            double newAngle = gyro.getYaw();
            telemetry.addData("yawDiff",getAngleDiff(startAngle,newAngle));
            USDF = rangeF.getUltraSonicDistance();
            USDB = rangeB.getUltraSonicDistance();
            telemetry.addData("USDF",USDF);
            telemetry.addData("USDB",USDB);
            telemetry.update();
        }
    }
}
