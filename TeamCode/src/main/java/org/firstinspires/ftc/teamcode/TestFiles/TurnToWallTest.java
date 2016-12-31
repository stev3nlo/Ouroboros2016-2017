package org.firstinspires.ftc.teamcode.TestFiles;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Turn Parallel to Wall Test", group="Test")
public class TurnToWallTest extends MyAutonomous
{
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        while(!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("USDF",rangeF.getUltraSonicDistance());
            telemetry.addData("USDB",rangeB.getUltraSonicDistance());
            telemetry.update();
            idle();
        }
        waitForStart();
        turnParallelToWall(0.185);
    }
}
