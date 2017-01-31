package org.firstinspires.ftc.teamcode.TestFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Libraries.MyAutonomous;
import org.firstinspires.ftc.teamcode.Libraries.MyOpMode;

/**
 * Created by spencersharp on 1/16/17.
 */
@Autonomous(name="Beacon Drift Test", group="Test")
public class MoveWithEncodersCoastExtraTest extends MyAutonomous
{
    public void runOpMode() throws InterruptedException
    {
        super.runOpMode();
        waitForStart();
        initCurtime();
        double startTime = getCurTime();
        while(getCurTime() - startTime < 2.0)
        {
            telemetry.addData("USDF",rangeF.getUltraSonicDistance());
            telemetry.addData("USDB",rangeB.getUltraSonicDistance());
            telemetry.update();
            initCurtime();
            idle();
        }
        driveToNextBeacon(-0.25,false,2000,1.0,0.91);
        pause(5.0);
    }
}
