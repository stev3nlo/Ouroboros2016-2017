package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

@Disabled//(name="Turn Parallel to Wall Test", group="Test")
public class TurnToWallTest extends MyAutonomous
{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        double USDF;
        double USDB;
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
        turnParallelToWallWithGyro(0.19,2);
        //if(USDF > USDB)
        //odd turn number (aka 1)
    }
}
